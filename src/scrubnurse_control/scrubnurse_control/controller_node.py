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
A pause takes effect between tasks. `~/abort` stops the arm at once, keeps the
gripper as it is and clears the request queue; resume is then refused until
`~/recover` (call it once the arm is clear), which makes a joint move to the
rest pose and waits paused.

State: `~/cycle` (scrubnurse_interfaces/CycleStatus, latched) and the same as
one sentence on `~/status` (std_msgs/String, latched).

If `home_pose` names a pose with recorded joints, the arm first makes a
joint-space move there, so straight-line moves start from a known
configuration. The cell uses HD, the rest pose where every task starts and ends
(the ROS 1 setup relied on the operator).
"""

from collections import deque
from dataclasses import replace
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
from scrubnurse_control.status import Status
from scrubnurse_control.tasks import HANDOVER, MoveToJoints, RETURN, TaskError, TaskRunner
from scrubnurse_control.workcell import select_return_candidate, Workcell
from scrubnurse_interfaces.msg import CycleStatus
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
        tool_ids = param('tool_ids', ['A', 'B', 'C', 'D', 'E', 'F'])
        self._tool_ids = set(tool_ids)
        self._tool_names = dict(zip(tool_ids, param('tool_names', [
            'pointed scissors', 'needle holder', 'mosquito', 'scalpel', 'anatomical forceps',
            'surgical forceps'])))
        self._base_frame = param('base_frame', 'base')
        self._start_paused = param('start_paused', True)
        self._step_mode = param('step_mode', True)
        home_pose = param('home_pose', '')
        self._home_steps = (MoveToJoints(home_pose),) if home_pose else None
        self._home = self._home_steps
        self._pause_after_home = False
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

        self._abort = threading.Event()

        def stopping():
            return stop_event.is_set() or self._abort.is_set()

        callback_group = ReentrantCallbackGroup()
        arm = Arm(
            self, callback_group,
            group_name=param('group_name', 'ur_manipulator'),
            base_frame=self._base_frame,
            end_effector=param('end_effector_link', 'tool0'),
            velocity_scaling=param('velocity_scaling', 0.1),
            acceleration_scaling=param('acceleration_scaling', 0.1),
            min_cartesian_fraction=param('min_cartesian_fraction', 0.95),
            motion_timeout=param('motion_timeout', 10.0),
            should_stop=stopping)
        self._gripper = self._make_gripper(callback_group)
        self._runner = TaskRunner(arm, self._gripper, self._poses, self._workcell,
                                  log=self.get_logger().info, should_stop=stopping,
                                  on_step=self._on_step)

        self._lock = threading.Lock()
        self._requests = deque()
        self._detections = []
        self._detections_stamp = None
        self._ignore_before = self.get_clock().now()
        self._status = Status()
        self._status_lock = threading.Lock()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, '~/status', latched)
        self._cycle_pub = self.create_publisher(CycleStatus, '~/cycle', latched)
        self.create_subscription(String, 'tool_request', self._on_request, 10,
                                 callback_group=callback_group)
        self.create_subscription(Detection3DArray, 'tool_detections', self._on_detections, 10,
                                 callback_group=callback_group)
        self.create_service(Trigger, '~/resume', self._on_resume, callback_group=callback_group)
        self.create_service(Trigger, '~/pause', self._on_pause, callback_group=callback_group)
        self.create_service(Trigger, '~/abort', self._on_abort, callback_group=callback_group)
        self.create_service(Trigger, '~/recover', self._on_recover, callback_group=callback_group)

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
            self._update(error=f'gripper activation failed: {e}')
            return
        self._update(paused=self._start_paused)
        while not self._stop.is_set():
            if self._home is not None and not self._status.paused:
                self._go_home()
                continue
            task = None if self._status.paused else self._next_task()
            if task is None:
                time.sleep(0.1)
                continue
            sequence, tool, detected = task
            name = 'handover' if sequence is HANDOVER else 'return'
            self._update(cycle_id=self._status.cycle_id + 1, kind=name, tool_id=tool,
                         tool_name=self._tool_names.get(tool, ''), step=0, step_count=0,
                         error='')
            try:
                self._runner.run(sequence, tool, detected)
            except (TaskError, TimeoutError, OSError) as e:
                self.get_logger().error(f'{name} {tool} failed: {e}')
                self._end_cycle(paused=True, error=f'{name} of tool {tool} failed: {e}')
                continue
            finally:
                self._ignore_before = self.get_clock().now() + self._settle_time
            self.get_logger().info(f'{name} {tool} done')
            self._end_cycle(paused=self._step_mode or self._status.paused)

    def _go_home(self):
        self._update(phase='homing', step=0, step_count=0)
        try:
            self._runner.run(self._home, tool='')
        except (TaskError, TimeoutError, OSError) as e:
            self.get_logger().error(f'homing failed: {e}')
            self._update(phase='idle', step=0, step_count=0, paused=True,
                         error=f'homing failed: {e}')
            return
        self._home = None
        self._update(phase='idle', step=0, step_count=0,
                     paused=self._pause_after_home or self._status.paused)
        self._pause_after_home = False

    def _on_step(self, phase, step, count):
        self._update(step=step, step_count=count, **({'phase': phase} if phase else {}))

    def _end_cycle(self, **changes):
        self._update(kind='', tool_id='', tool_name='', phase='idle', step=0, step_count=0,
                     **changes)

    def _next_task(self):
        with self._lock:
            if self._requests:
                tool = self._requests.popleft()
                self._update(queue=tuple(self._requests))
                return HANDOVER, tool, None
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
        if self._status.aborted:
            self.get_logger().warn(f'aborted: ignoring request for tool {tool}')
            return
        with self._lock:
            # Like ROS 1, a repeated request for the tool already last in the queue is dropped.
            if not self._requests or self._requests[-1] != tool:
                self._requests.append(tool)
            queue = tuple(self._requests)
            self._update(queue=queue)
        self.get_logger().info(f'requested tools: {list(queue)}')

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
        if self._status.aborted:
            response.message = 'aborted: call recover once the arm is clear'
        else:
            homing = {'phase': 'homing'} if self._home is not None else {}
            self._update(paused=False, error='', **homing)
            response.success = True
        return response

    def _on_pause(self, request, response):
        self._update(paused=True)
        response.success = True
        if self._status.kind:
            response.message = 'pausing after the current cycle'
        return response

    def _on_abort(self, request, response):
        self._abort.set()  # the running motion is cancelled within one control period
        with self._lock:
            self._requests.clear()
            self._update(aborted=True, paused=True, queue=())
        self._home = None
        response.success = True
        return response

    def _on_recover(self, request, response):
        if not self._status.aborted:
            response.message = 'not aborted'
            return response
        self._abort.clear()
        if self._home_steps is None:
            self._update(aborted=False, error='')
            response.message = 'no home_pose: resume when ready'
        else:
            self._home = self._home_steps
            self._pause_after_home = True
            self._update(aborted=False, paused=False, error='', phase='homing')
            response.message = 'moving to the rest pose, then paused'
        response.success = True
        return response

    def _update(self, **changes):
        """Change the published state; every change goes out on ~/cycle and ~/status."""
        with self._status_lock:
            old, self._status = self._status, replace(self._status, **changes)
            s = self._status
        if s.description != old.description:
            self.get_logger().info(f'status: {s.description}')
        self._status_pub.publish(String(data=s.description))
        self._cycle_pub.publish(CycleStatus(
            stamp=self.get_clock().now().to_msg(), cycle_id=s.cycle_id, kind=s.kind,
            tool_id=s.tool_id, tool_name=s.tool_name, phase=s.phase,
            description=s.description, step=s.step, step_count=s.step_count, paused=s.paused,
            aborted=s.aborted, error=s.error, queue=list(s.queue)))


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
