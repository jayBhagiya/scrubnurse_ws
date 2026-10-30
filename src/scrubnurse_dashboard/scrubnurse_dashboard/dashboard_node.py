"""
Scrubnurse dashboard: a web page to watch and control the cell.

Subscribes to the controller's cycle status, /joint_states, the camera image,
tool detections (2D for the overlay, 3D for robot coordinates) and the voice
node's events. Serves (FastAPI, see app.py) the state as JSON and over a
WebSocket, the camera with the detections drawn in as MJPEG, and pause, resume,
abort, recover and tool requests, which call the controller's services or
publish on `tool_request`.
"""

import collections
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from scrubnurse_dashboard import state
from scrubnurse_dashboard.app import create_app
from scrubnurse_interfaces.msg import CycleStatus, VoiceEvent
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import uvicorn
from vision_msgs.msg import Detection2DArray, Detection3DArray

CONTROLLER = '/scrubnurse_controller'
STALE_AFTER = {'joint_states': 1.0, 'camera': 2.0, 'detections': 2.0}  # s
STREAM_WIDTH = 960  # px, camera stream


class DashboardNode(Node):

    def __init__(self):
        super().__init__('scrubnurse_dashboard')
        self.mode = self.declare_parameter('mode', 'real').value  # 'sim' or 'real', for the badge
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.port = self.declare_parameter('port', 8080).value
        self._call_timeout = self.declare_parameter('service_timeout', 3.0).value
        tool_ids = self.declare_parameter('tool_ids', ['A', 'B', 'C', 'D', 'E', 'F']).value
        tool_names = self.declare_parameter('tool_names', [
            'pointed scissors', 'needle holder', 'mosquito', 'scalpel', 'anatomical forceps',
            'surgical forceps']).value
        self.tool_names = dict(zip(tool_ids, tool_names))

        self._lock = threading.Lock()
        self._cycle = None
        self._joints = None
        self._detections = []
        self._positions = {}    # tool id -> [x, y] in the robot base frame (m)
        self._voice = collections.deque(maxlen=10)
        self._events = collections.deque(maxlen=200)
        self._received = dict.fromkeys(STALE_AFTER)  # topic -> time.monotonic() of the last one
        self._image = None
        self._jpeg = None, None  # (image message, its encoded overlay)
        self._bridge = CvBridge()
        self._health = {}
        self._stale = {}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CycleStatus, f'{CONTROLLER}/cycle', self._on_cycle, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, 'image', self._on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/tool_detections_2d', self._on_detections, 10)
        self.create_subscription(Detection3DArray, '/tool_detections', self._on_positions, 10)
        self.create_subscription(VoiceEvent, '/voice_command/events', self._on_voice, 10)
        self._request_pub = self.create_publisher(String, '/tool_request', 10)
        self._triggers = {action: self.create_client(Trigger, f'{CONTROLLER}/{action}')
                          for action in ('pause', 'resume', 'abort', 'recover')}
        self._update_health()
        self.create_timer(1.0, self._update_health)

    # ROS side

    def _on_cycle(self, msg):
        cycle = {name: getattr(msg, name) for name in (
            'cycle_id', 'kind', 'tool_id', 'tool_name', 'phase', 'description', 'step',
            'step_count', 'paused', 'aborted', 'error')}
        cycle['queue'] = [{'tool_id': t, 'tool_name': self.tool_names.get(t, '')}
                          for t in msg.queue]
        with self._lock:
            changed = self._cycle is None or self._cycle['description'] != msg.description
            self._cycle = cycle
        if changed:
            level = 'error' if msg.error else 'warn' if msg.aborted else 'info'
            self.log(level, 'controller', msg.description)

    def _on_joints(self, msg):
        moving = any(abs(v) > 0.01 for v in msg.velocity)
        with self._lock:
            self._joints = {'names': list(msg.name), 'positions': list(msg.position),
                            'moving': moving}
            self._received['joint_states'] = time.monotonic()

    def _on_image(self, msg):
        with self._lock:
            self._image = msg
            self._received['camera'] = time.monotonic()

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
        event = {'time': time.time(), 'kind': msg.kind, 'text': msg.text,
                 'wake_word': msg.wake_word, 'tool_id': msg.tool_id, 'tool_name': msg.tool_name}
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

    def _update_health(self):
        now = time.monotonic()
        with self._lock:
            topics = {name: state.freshness(self._received[name], now, limit)
                      for name, limit in STALE_AFTER.items()}
        health = {
            'mode': self.mode,
            'node': True,
            'controller': self._triggers['pause'].service_is_ready(),
            'voice': self.count_publishers('/voice_command/events') > 0,
            'topics': topics,
        }
        for name, topic in topics.items():
            if topic['age'] is None:  # never received: not "stale" news, the panel shows it
                continue
            if topic['stale'] != self._stale.get(name, False):
                self.log('warn' if topic['stale'] else 'info', 'health',
                         f'{name} {"stale" if topic["stale"] else "back"}')
            self._stale[name] = topic['stale']
        for name in ('controller', 'voice'):
            if name in self._health and health[name] != self._health[name]:
                self.log('info' if health[name] else 'warn', 'health',
                         f'{name} {"running" if health[name] else "not running"}')
        self._health = health

    # Used by the web app (other threads)

    def log(self, level, source, text):
        with self._lock:
            self._events.appendleft({'time': time.time(), 'level': level, 'source': source,
                                     'text': text})

    def health(self):
        return self._health

    def refusal(self, action):
        with self._lock:
            cycle = self._cycle
        return state.refusal(action, cycle, self._health.get('controller', False))

    def snapshot(self):
        allowed = {action: self.refusal(action) for action in state.ACTIONS}
        with self._lock:
            return {
                'mode': self.mode,
                'health': self._health,
                'cycle': self._cycle,
                'allowed': allowed,  # action -> why it is refused, or None
                'arm': self._joints,
                'detections': [dict(d, position=self._positions.get(d['tool_id']))
                               for d in self._detections],
                'voice': list(self._voice),
                'events': list(self._events),
                'tools': [{'tool_id': k, 'tool_name': v} for k, v in self.tool_names.items()],
            }

    def call(self, action):
        """Call a controller Trigger service; return (success, message)."""
        client = self._triggers[action]
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._call_timeout):
            client.remove_pending_request(future)
            raise TimeoutError(f'no answer from the controller within {self._call_timeout:.0f} s')
        response = future.result()
        return response.success, response.message

    def request_tool(self, tool_id):
        self._request_pub.publish(String(data=tool_id))
        return True, f'requested the {self.tool_names[tool_id]}'

    def jpeg(self):
        """Newest camera frame with the detections drawn in, as JPEG; None before the first."""
        with self._lock:
            msg, detections = self._image, self._detections
            cached_msg, cached = self._jpeg
            fresh = not state.freshness(self._received['detections'], time.monotonic(),
                                        STALE_AFTER['detections'])['stale']
        if msg is None or msg is cached_msg:
            return cached
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if fresh:
            state.draw_detections(image, detections)
        scale = STREAM_WIDTH / image.shape[1]
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        with self._lock:
            self._jpeg = msg, jpeg
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
