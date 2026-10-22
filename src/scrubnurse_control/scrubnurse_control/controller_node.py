"""
Scrubnurse task controller: hands requested tools over and returns used ones.

Inputs
  tool_request     std_msgs/String, tool id from the voice node (queued FIFO)
  tool_detections  vision_msgs/Detection3DArray, tools on the table in the UR base frame

Behaviour (ROS 1 `scrubnurse_combined_loop`): requested tools are handed over
first; when no request is queued, a tool lying in the return area is picked up
and put back into the magazine.

Operator control (replaces the ROS 1 keyboard prompts): `~/resume` and
`~/pause` (std_srvs/Trigger). With start_paused the node waits for `~/resume`
before the first motion; with step_mode it pauses again after every task.
A pause takes effect between tasks. State is published on `~/status`.

If `home_pose` names a pose with recorded joints, the arm first makes a
joint-space move there, so straight-line moves start from a known
configuration (used in simulation; the ROS 1 setup relied on the operator).
"""

from collections import deque
import signal
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scrubnurse_control.gripper import ActionGripper, MockGripper, RobotiqSocketGripper
from scrubnurse_control.motion import Arm
from scrubnurse_control.poses import DetectedTool, load_poses
from scrubnurse_control.tasks import HANDOVER, MoveToJoints, RETURN, TaskError, TaskRunner
from scrubnurse_control.workcell import select_return_candidate, Workcell
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray


class ScrubnurseController(Node):

    def __init__(self, stop_event):
        super().__init__('scrubnurse_controller')
        self._stop = stop_event
        param = self._param
        poses_file = self.declare_parameter('poses_file', Parameter.Type.STRING).value
        if not poses_file:
            raise ValueError('parameter "poses_file" is required')
        self._poses = load_poses(poses_file)
        self._tool_ids = set(param('tool_ids', ['A', 'B', 'C', 'D', 'E', 'F']))
        self._base_frame = param('base_frame', 'base')
        self._start_paused = param('start_paused', True)
        self._step_mode = param('step_mode', True)
        home_pose = param('home_pose', '')
        self._home = (MoveToJoints(home_pose),) if home_pose else None
        self._detection_timeout = Duration(seconds=param('detection_timeout', 1.0))
        self._settle_time = Duration(seconds=param('settle_time', 0.5))
        defaults = Workcell()
        self._workcell = Workcell(
            return_area=tuple(param('return_area', list(defaults.return_area))),
            approach_height=param('approach_height', defaults.approach_height),
            grasp_x_range=tuple(param('grasp_height.x_range', list(defaults.grasp_x_range))),
            grasp_y_range=tuple(param('grasp_height.y_range', list(defaults.grasp_y_range))),
            grasp_z_at_y0=tuple(param('grasp_height.z_at_y0', list(defaults.grasp_z_at_y0))),
            grasp_z_at_y1=tuple(param('grasp_height.z_at_y1', list(defaults.grasp_z_at_y1))),
        )

        callback_group = ReentrantCallbackGroup()
        arm = Arm(
            self, callback_group,
            group_name=param('group_name', 'ur_manipulator'),
            base_frame=self._base_frame,
            end_effector=param('end_effector_link', 'tool0'),
            velocity_scaling=param('velocity_scaling', 0.1),
            acceleration_scaling=param('acceleration_scaling', 0.1),
            min_cartesian_fraction=param('min_cartesian_fraction', 0.95),
            motion_timeout=param('motion_timeout', 60.0),
            should_stop=stop_event.is_set)
        self._gripper = self._make_gripper(callback_group)
        self._runner = TaskRunner(arm, self._gripper, self._poses, self._workcell,
                                  log=self.get_logger().info, should_stop=stop_event.is_set)

        self._lock = threading.Lock()
        self._requests = deque()
        self._detections = []
        self._detections_stamp = None
        self._ignore_before = self.get_clock().now()
        self._paused = True

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '~/status', latched)
        self.create_subscription(String, 'tool_request', self._on_request, 10,
                                 callback_group=callback_group)
        self.create_subscription(Detection3DArray, 'tool_detections', self._on_detections, 10,
                                 callback_group=callback_group)
        self.create_service(Trigger, '~/resume', self._on_resume, callback_group=callback_group)
        self.create_service(Trigger, '~/pause', self._on_pause, callback_group=callback_group)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _make_gripper(self, callback_group):
        backend = self._param('gripper.backend', 'robotiq_socket')
        if backend == 'robotiq_socket':
            return RobotiqSocketGripper(self._param('gripper.host', ''),
                                        self._param('gripper.port', 63352),
                                        log=self.get_logger().info)
        if backend == 'action':
            return ActionGripper(
                self, self._param('gripper.action_name', '/gripper_controller/gripper_cmd'),
                self._param('gripper.open_position', 0.0),
                self._param('gripper.closed_position', 0.8),
                self._param('gripper.max_effort', 50.0), callback_group)
        if backend == 'mock':
            return MockGripper(log=self.get_logger().info)
        raise ValueError(f'unknown gripper.backend "{backend}" '
                         '(expected robotiq_socket, action or mock)')

    def run(self):
        """Task loop; runs in the main thread while the executor spins in another."""
        try:
            self._gripper.activate()
        except (TimeoutError, OSError, RuntimeError) as e:
            self.get_logger().fatal(f'gripper activation failed: {e}')
            self._publish_status(f'error: gripper activation failed: {e}')
            return
        self._set_paused(self._start_paused, 'waiting for ~/resume')
        while not self._stop.is_set():
            if self._home is not None and not self._paused:
                self._go_home()
                continue
            task = None if self._paused else self._next_task()
            if task is None:
                time.sleep(0.1)
                continue
            sequence, tool, detected = task
            name = 'handover' if sequence is HANDOVER else 'return'
            self._publish_status(f'{name} {tool}')
            try:
                self._runner.run(sequence, tool, detected)
            except (TaskError, TimeoutError, OSError) as e:
                self.get_logger().error(f'{name} {tool} aborted: {e}')
                self._set_paused(True, f'error in {name} {tool}: {e}')
                continue
            finally:
                self._ignore_before = self.get_clock().now() + self._settle_time
            self.get_logger().info(f'{name} {tool} done')
            if self._step_mode:
                self._set_paused(True, 'step mode, waiting for ~/resume')
            else:
                self._publish_status('idle')

    def _go_home(self):
        self._publish_status('moving home')
        try:
            self._runner.run(self._home, tool='')
        except (TaskError, TimeoutError, OSError) as e:
            self.get_logger().error(f'homing failed: {e}')
            self._set_paused(True, f'error while homing: {e}')
            return
        self._home = None
        self._publish_status('idle')

    def _next_task(self):
        with self._lock:
            if self._requests:
                return HANDOVER, self._requests.popleft(), None
            stamp = self._detections_stamp
            if stamp is None or stamp < self._ignore_before:
                return None
            if self.get_clock().now() - stamp > self._detection_timeout:
                return None
            candidate = select_return_candidate(self._detections, self._workcell)
            if candidate is None:
                return None
            # Wait for a fresh image after this task instead of reusing these poses.
            self._detections = []
            return RETURN, candidate.tool_id, candidate

    def _on_request(self, msg):
        tool = msg.data.strip()
        if tool not in self._tool_ids:
            self.get_logger().warn(f'ignoring request for unknown tool "{tool}"')
            return
        with self._lock:
            # Like ROS 1, a repeated request for the tool already last in the queue is dropped.
            if not self._requests or self._requests[-1] != tool:
                self._requests.append(tool)
            queue = list(self._requests)
        self.get_logger().info(f'requested tools: {queue}')

    def _on_detections(self, msg):
        if msg.header.frame_id != self._base_frame:
            self.get_logger().warn(
                f'tool detections in frame "{msg.header.frame_id}", expected '
                f'"{self._base_frame}"; ignored', throttle_duration_sec=5.0)
            return
        tools = []
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id not in self._tool_ids:
                continue
            p, q = det.bbox.center.position, det.bbox.center.orientation
            tools.append(DetectedTool(det.results[0].hypothesis.class_id,
                                      (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        with self._lock:
            self._detections = tools
            self._detections_stamp = Time.from_msg(msg.header.stamp,
                                                   clock_type=self.get_clock().clock_type)

    def _on_resume(self, request, response):
        self._set_paused(False, 'running')
        response.success = True
        return response

    def _on_pause(self, request, response):
        self._set_paused(True, 'paused, waiting for ~/resume')
        response.success = True
        return response

    def _set_paused(self, paused, status):
        self._paused = paused
        self._publish_status(status)

    def _publish_status(self, status):
        self.get_logger().info(f'status: {status}')
        self._status_pub.publish(String(data=status))


def main(args=None):
    # Handle Ctrl-C / SIGTERM ourselves: stop the task loop and cancel the running
    # trajectory while ROS is still up, then shut down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        node = ScrubnurseController(stop)
    except (ValueError, OSError, RuntimeError) as e:
        rclpy.logging.get_logger('scrubnurse_controller').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    finally:
        node.get_logger().info('shutting down')
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
