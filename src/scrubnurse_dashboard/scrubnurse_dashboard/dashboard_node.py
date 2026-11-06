"""
Scrubnurse dashboard: a web page to watch and control the cell.

Subscribes to the controller's cycle status, /joint_states, the camera image,
tool detections (2D for the overlay, 3D for robot coordinates) and the voice
node's events. Serves (FastAPI, see app.py) the state as JSON and over a
WebSocket, the camera with the detections drawn in as MJPEG, and pause, resume,
abort, recover and tool requests, which call the controller's services or
publish on `tool_request`. Voice: mute, push-to-talk and the microphone (parameters
and ~/listen of the voice node); camera: which Azure Kinect streams (parameter
`serial` of kinect_camera). The calibration page records camera-to-table pairs
(see calibration.py). In mode replay it shows the recorded run `run` (see replay.py and
replay.launch.py) and drives the bag player: pause, resume, jump and speed.

The page is built separately and installed with the package if present:
`cd web && npm ci && npm run build`, then colcon build.
"""

import collections
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rosbag2_interfaces.srv import Burst, Pause, Resume, Seek, SetRate
from rosgraph_msgs.msg import Clock
from scrubnurse_dashboard import replay, state
from scrubnurse_dashboard.app import create_app
from scrubnurse_dashboard.calibration import Calibration, find_block
from scrubnurse_interfaces.msg import CameraStatus, CycleStatus, VoiceEvent, VoiceStatus
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import uvicorn
from vision_msgs.msg import Detection2DArray, Detection3DArray

CONTROLLER = '/scrubnurse_controller'
VOICE = '/voice_command'
CAMERA = '/kinect_camera'
PLAYER = '/rosbag2_player'
BURST = 200  # messages played after a jump while paused: on the real cell about 0.2 s, 5 frames
STALE_AFTER = {'joint_states': 1.0, 'camera': 2.0, 'detections': 2.0}  # s
STREAM_WIDTH = 1280  # px, camera stream


def encode(image):
    """Scale a BGR image to the stream width and encode it as JPEG."""
    scale = STREAM_WIDTH / image.shape[1]
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


class DashboardNode(Node):

    def __init__(self):
        super().__init__('scrubnurse_dashboard')
        self.mode = self.declare_parameter('mode', 'real').value  # real, sim, mock or replay
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.port = self.declare_parameter('port', 8080).value
        self._call_timeout = self.declare_parameter('service_timeout', 3.0).value
        tool_ids = self.declare_parameter('tool_ids', ['A', 'B', 'C', 'D', 'E', 'F']).value
        tool_names = self.declare_parameter('tool_names', [
            'pointed scissors', 'needle holder', 'mosquito', 'scalpel', 'anatomical forceps',
            'surgical forceps']).value
        self.tool_names = dict(zip(tool_ids, tool_names))
        folder = self.declare_parameter('calibration_dir', '~/scrubnurse_calibration').value
        self.calibration = Calibration(os.path.expanduser(folder))
        # Expected block side in pixels and where to look for it (the table work area).
        self._block_side = self.declare_parameter('calibration_block_px', 44.0).value
        self._block_roi = self.declare_parameter(
            'calibration_roi', Parameter.Type.INTEGER_ARRAY).value  # None: whole image
        self._base_frame = self.declare_parameter('base_frame', 'base').value
        self._tip_frame = self.declare_parameter('tip_frame', 'tool0').value
        self._tf = None  # started by the calibration page: /tf is busy and only needed there

        self._lock = threading.Lock()
        self._cycle = None
        self._joints = None
        self._joints_before = 0.0, []  # (time, positions) to tell whether the arm moves
        self._moving = False
        self._detections = []
        self._positions = {}    # tool id -> [x, y] in the robot base frame (m)
        self._voice = collections.deque(maxlen=10)
        self._voice_status = None
        self._camera_status = None
        self._events = collections.deque(maxlen=200)
        self._received = dict.fromkeys(STALE_AFTER)  # topic -> time.monotonic() of the last one
        self._image = None
        self._jpeg = None, None  # (image message, its encoded overlay)
        self._scene = None
        self._scene_jpeg = None, None
        self._bridge = CvBridge()
        self._health = {}
        self._stale = {}

        self._run = None
        if self.mode == 'replay':
            self._run = replay.Run.read(os.path.expanduser(
                self.declare_parameter('run', '').value), self.tool_names)
            self._playhead = self._run.start
            self._paused, self._rate = False, 1.0
            self._seeked = 0.0  # time.monotonic() of the last jump
            self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
            self._player = {name: self.create_client(srv, f'{PLAYER}/{name}') for name, srv in (
                ('pause', Pause), ('resume', Resume), ('seek', Seek), ('set_rate', SetRate),
                ('burst', Burst))}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self._run is None:  # replay: read from the bag (replay.Run)
            self.create_subscription(CycleStatus, f'{CONTROLLER}/cycle', self._on_cycle, latched)
            self.create_subscription(VoiceEvent, f'{VOICE}/events', self._on_voice, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, 'image', self._on_image, qos_profile_sensor_data)
        if self.mode == 'sim':  # side view of the simulated cell (scrubnurse_sim scene_camera)
            self.create_subscription(Image, '/scene_camera/image', self._on_scene,
                                     qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/tool_detections_2d', self._on_detections, 10)
        self.create_subscription(Detection3DArray, '/tool_detections', self._on_positions, 10)
        self.create_subscription(VoiceStatus, f'{VOICE}/status', self._on_voice_status, latched)
        self.create_subscription(CameraStatus, f'{CAMERA}/status', self._on_camera_status, latched)
        self._listen = self.create_client(Trigger, f'{VOICE}/listen')
        self._parameters = {node: self.create_client(SetParameters, f'{node}/set_parameters')
                            for node in (VOICE, CAMERA)}
        self._request_pub = self.create_publisher(String, '/tool_request', 10)
        self._triggers = {action: self.create_client(Trigger, f'{CONTROLLER}/{action}')
                          for action in ('pause', 'resume', 'abort', 'recover')}
        self._update_health()
        self.create_timer(1.0, self._update_health)

    # ROS side

    def _on_cycle(self, msg):
        cycle = state.cycle_dict(msg, self.tool_names)
        with self._lock:
            changed = self._cycle is None or self._cycle['description'] != msg.description
            self._cycle = cycle
        if changed:
            self.log(state.cycle_level(cycle), 'controller', msg.description)

    def _on_joints(self, msg):
        # Moving: from the positions 0.2 s ago, since fake hardware reports no velocities.
        now, positions = time.monotonic(), list(msg.position)
        then, before = self._joints_before
        if now - then >= 0.2 or len(before) != len(positions):
            self._moving = len(before) == len(positions) and any(
                abs(a - b) > 0.002 for a, b in zip(positions, before))
            self._joints_before = now, positions
        with self._lock:
            self._joints = {'names': list(msg.name), 'positions': positions,
                            'moving': self._moving}
            self._received['joint_states'] = time.monotonic()

    def _on_image(self, msg):
        with self._lock:
            self._image = msg
            self._received['camera'] = time.monotonic()

    def _on_scene(self, msg):
        with self._lock:
            self._scene = msg

    def _on_detections(self, msg):
        detections = [state.detection_dict(d, self.tool_names) for d in msg.detections]
        with self._lock:
            self._detections = detections
            self._received['detections'] = time.monotonic()

    def _on_positions(self, msg):
        positions = {d.results[0].hypothesis.class_id:
                     [round(d.bbox.center.position.x, 4), round(d.bbox.center.position.y, 4)]
                     for d in msg.detections if d.results}
        with self._lock:
            self._positions = positions

    def _on_voice(self, msg):
        event = state.voice_dict(msg, time.time())
        with self._lock:
            self._voice.appendleft(event)
        if msg.kind == 'error':
            self.log('error', 'voice', msg.text)
        elif msg.kind == 'ready':
            self.log('info', 'voice', 'Microphone on, listening')
        elif msg.tool_id:
            self.log('info', 'voice', f'Heard "{msg.text}": requesting the {msg.tool_name}')
        elif msg.wake_word:
            self.log('warn', 'voice', f'Heard "{msg.text}": no instrument recognised')

    def _on_voice_status(self, msg):
        status = {name: getattr(msg, name) for name in (
            'state', 'message', 'live', 'muted', 'input_device')}
        status['devices'], status['wake_words'] = list(msg.devices), list(msg.wake_words)
        with self._lock:
            self._voice_status = status

    def _on_camera_status(self, msg):
        with self._lock:
            self._camera_status = {'serial': msg.serial, 'serials': list(msg.serials),
                                   'configured_serial': msg.configured_serial}
        if msg.serial != msg.configured_serial and msg.configured_serial:
            self.log('warn', 'camera', f'streaming {msg.serial}, not the calibrated camera '
                                       f'{msg.configured_serial}: picks will miss')
        else:
            self.log('info', 'camera', f'streaming {msg.serial}')

    def _on_clock(self, msg):
        t = Time.from_msg(msg.clock).nanoseconds / 1e9
        with self._lock:
            wrapped = t < self._playhead - 1.0 and time.monotonic() - self._seeked > 2.0
            self._playhead = t
        if wrapped and not self._paused:  # the player loops: stop at the start, not replay it
            self._paused = True
            self._player['pause'].call_async(Pause.Request())

    def _stale_after(self, topic):
        # A paused replay sends nothing: its last messages are still the current ones.
        return math.inf if self._run is not None and self._paused else STALE_AFTER[topic]

    def _update_health(self):
        now = time.monotonic()
        with self._lock:
            topics = {name: state.freshness(self._received[name], now, self._stale_after(name))
                      for name in STALE_AFTER}
        health = {
            'mode': self.mode,
            'node': True,
            'controller': (self._triggers['pause'].service_is_ready() if self._run is None
                           else bool(self._run.cycles)),
            'voice': self.count_publishers(f'{VOICE}/events') > 0,
            'camera_node': self.count_publishers(f'{CAMERA}/status') > 0,
            'topics': topics,
        }
        for name, topic in topics.items():
            if topic['age'] is None:  # never received: not "stale" news, the panel shows it
                continue
            if topic['stale'] != self._stale.get(name, False):
                self.log('warn' if topic['stale'] else 'info', 'health',
                         f'{name} {"stale" if topic["stale"] else "back"}')
            self._stale[name] = topic['stale']
        for name in ('controller', 'voice', 'camera_node'):
            if name in self._health and health[name] != self._health[name]:
                label = name.replace('_', ' ')
                self.log('info' if health[name] else 'warn', 'health',
                         f'{label} {"running" if health[name] else "not running"}')
        self._health = health

    # Used by the web app (other threads)

    def log(self, level, source, text):
        with self._lock:
            self._events.appendleft({'time': time.time(), 'level': level, 'source': source,
                                     'text': text})

    def health(self):
        return self._health

    def refusal(self, action):
        if self.mode == 'replay':
            return 'replay: the controls are off'
        with self._lock:
            cycle = self._cycle
        return state.refusal(action, cycle, self._health.get('controller', False))

    def snapshot(self):
        allowed = {action: self.refusal(action) for action in state.ACTIONS}
        with self._lock:
            snapshot = {
                'mode': self.mode,
                'health': self._health,
                'cycle': self._cycle,
                'allowed': allowed,  # action -> why it is refused, or None
                'arm': self._joints,
                'detections': [dict(d, position=self._positions.get(d['tool_id']))
                               for d in self._detections],
                'voice': list(self._voice),
                'voice_status': self._voice_status if self._health.get('voice') else None,
                'camera_status': self._camera_status if self._health.get('camera_node') else None,
                'events': list(self._events),
                'tools': [{'tool_id': k, 'tool_name': v} for k, v in self.tool_names.items()],
                'replay': None,
            }
            if self._run is None:
                return snapshot
            run, playhead = self._run, self._playhead
            snapshot['replay'] = {'start': run.start, 'duration': run.end - run.start,
                                  'time': playhead - run.start, 'paused': self._paused,
                                  'rate': self._rate, 'cycles': run.markers}
        snapshot.update(run.at(playhead))
        return snapshot

    def replay(self, action, value=None):
        """Pause, resume, seek (s from the start) or set_rate of the bag player."""
        if self._run is None:
            return False, 'not a replay'
        if action == 'seek':
            self._seeked = time.monotonic()
            request = Seek.Request(
                time=Time(nanoseconds=int((self._run.start + value) * 1e9)).to_msg())
        elif action == 'set_rate':
            request = SetRate.Request(rate=value)
        else:
            request = self._player[action].srv_type.Request()
        response = self._call(self._player[action], request, 'the bag player')
        if getattr(response, 'success', True):  # pause and resume answer nothing
            if action in ('pause', 'resume'):
                self._paused = action == 'pause'
            elif action == 'set_rate':
                self._rate = value
            elif self._paused:  # seek: show the camera and the arm of the new moment too
                self._call(self._player['burst'], Burst.Request(num_messages=BURST),
                           'the bag player')
            return True, ''
        return False, f'{action} refused'

    def call(self, action):
        """Call a controller Trigger service; return (success, message)."""
        response = self._call(self._triggers[action], Trigger.Request(), 'the controller')
        return response.success, response.message

    def voice_listen(self):
        """Push-to-talk: the voice node takes the next phrase without a wake word."""
        response = self._call(self._listen, Trigger.Request(), 'the voice node',
                              timeout=self._call_timeout + 10.0)  # without live it records first
        return response.success, response.message

    def set_parameter(self, node, name, value, timeout=None):
        """Set a parameter of another node; return (success, reason)."""
        parameter = Parameter(name, value=value).to_parameter_msg()
        request = SetParameters.Request(parameters=[parameter])
        result = self._call(self._parameters[node], request, node, timeout).results[0]
        return result.successful, result.reason

    def voice_status(self):
        """Return the voice node's last status, or None while it is not running."""
        with self._lock:
            return self._voice_status if self._health.get('voice') else None

    def camera_status(self):
        """Return the camera node's last status, or None while it is not running."""
        with self._lock:
            return self._camera_status if self._health.get('camera_node') else None

    def _call(self, client, request, who, timeout=None):
        timeout = timeout or self._call_timeout
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise TimeoutError(f'no answer from {who} within {timeout:.0f} s')
        return future.result()

    def request_tool(self, tool_id):
        self._request_pub.publish(String(data=tool_id))
        return True, f'requested the {self.tool_names[tool_id]}'

    def block_now(self):
        """Find the calibration block in the newest frame: {'u', 'v', 'corners'} or {'error'}."""
        with self._lock:
            msg = self._image
        if msg is None:
            return {'error': 'no camera image'}
        try:
            (u, v), corners = self._find_block(msg)
        except ValueError as e:
            return {'error': str(e)}
        return {'u': u, 'v': v, 'corners': corners}

    def block_pixel(self, seconds=1.0):
        """Median block centre over the frames of the next `seconds`; store it as pending."""
        centres, frames, last, error = [], 0, None, ''
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self._lock:
                msg = self._image
            if msg is not None and msg is not last:
                last, frames = msg, frames + 1
                try:
                    centres.append(self._find_block(msg)[0])
                except ValueError as e:
                    error = str(e)
            time.sleep(0.02)
        if frames < 3 or len(centres) < 0.8 * frames:
            raise ValueError(f'block found in {len(centres)} of {frames} frames'
                             + (f': {error}' if error else ''))
        centres = np.array(centres)
        u, v = np.median(centres, axis=0)
        spread = float(np.abs(centres - (u, v)).max())
        self.calibration.set_pixel(float(u), float(v), len(centres), spread)
        return self.calibration.state()['pending']

    def tool0(self):
        """Return the current tool0 x/y (m) in the base frame, or None."""
        with self._lock:
            if self._tf is None:
                self._tf = Buffer()
                self._tf_listener = TransformListener(self._tf, self)
        try:
            t = self._tf.lookup_transform(self._base_frame, self._tip_frame, Time()).transform
        except TransformException:
            return None
        return {'x': t.translation.x, 'y': t.translation.y}

    def image_size(self):
        with self._lock:
            msg = self._image
        return None if msg is None else [msg.width, msg.height]

    def _find_block(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        roi = None if self._block_roi is None else np.reshape(self._block_roi, (-1, 2))
        return find_block(image, self._block_side, roi)

    def jpeg(self):
        """Newest camera frame with the detections drawn in, as JPEG; None before the first."""
        with self._lock:
            msg, detections = self._image, self._detections
            cached_msg, cached = self._jpeg
            fresh = not state.freshness(self._received['detections'], time.monotonic(),
                                        self._stale_after('detections'))['stale']
        if msg is None or msg is cached_msg:
            return cached
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if fresh:
            state.draw_detections(image, detections)
        jpeg = encode(image)
        with self._lock:
            self._jpeg = msg, jpeg
        return jpeg

    def scene_jpeg(self):
        """Newest frame of the simulation's side view as JPEG; None without one."""
        with self._lock:
            msg, (cached_msg, cached) = self._scene, self._scene_jpeg
        if msg is None or msg is cached_msg:
            return cached
        jpeg = encode(self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        with self._lock:
            self._scene_jpeg = msg, jpeg
        return jpeg


def main(args=None):
    # uvicorn handles Ctrl-C; rclpy is shut down after it returns.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    web_dir = os.path.join(get_package_share_directory('scrubnurse_dashboard'), 'web')
    node.get_logger().info(f'Dashboard on http://{node.host}:{node.port} ({node.mode})')
    try:
        uvicorn.run(create_app(node, web_dir), host=node.host, port=node.port,
                    log_level='warning')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        spinner.join(2.0)  # exiting while it is inside rclpy aborts the process
        node.destroy_node()
        rclpy.try_shutdown()
