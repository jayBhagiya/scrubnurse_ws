"""
Simulated gripper: a control_msgs/GripperCommand action server like a ros2_control gripper.

Goal positions are per-finger travel (0 open .. `stroke` closed). The fingers
are moved through finger_controller. When the gripper closes narrower than a
tool and a tool lies between the fingertips, that tool is attached to the
gripper with its Gazebo DetachableJoint (the fingers stop on it and the result
reports `stalled`); opening wider than a tool detaches it again.
"""

import threading
import time

from control_msgs.action import GripperCommand
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scrubnurse_sim.cell import tool_class
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

TOOLS = ['tool_A', 'tool_B', 'tool_C', 'tool_D', 'tool_E', 'tool_F', 'tool_C_table']


def rotation_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def in_grasp_region(tool_in_tcp, half_x=0.015, half_y=0.03, z_range=(0.11, 0.20)):
    """
    Return True if a tool centre given in the tool0 frame lies between the fingertips.

    Flat tools are grasped at their centre (0.146 m along tool0 z), upright magazine tools
    2.5 cm above it (centre at 0.171 m).
    """
    x, y, z = tool_in_tcp
    return abs(x) <= half_x and abs(y) <= half_y and z_range[0] <= z <= z_range[1]


class SimGripper(Node):

    def __init__(self):
        super().__init__('sim_gripper')
        self._stroke = self.declare_parameter('stroke', 0.025).value
        self._tool_width = self.declare_parameter('tool_width', 0.012).value
        self._tools = list(self.declare_parameter('tools', TOOLS).value)
        group = ReentrantCallbackGroup()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._lock = threading.Lock()
        self._tool_poses = {}
        self._finger = 0.0
        self._attached = None

        self._released_pub = self.create_publisher(String, '/scrubnurse_sim/released', 10)
        self._command_pub = self.create_publisher(
            Float64MultiArray, '/finger_controller/commands', 10)
        self._attach_pubs = {t: self.create_publisher(Empty, f'/scrubnurse_sim/{t}/attach', 10)
                             for t in self._tools}
        self._detach_pubs = {t: self.create_publisher(Empty, f'/scrubnurse_sim/{t}/detach', 10)
                             for t in self._tools}
        self.create_subscription(TFMessage, 'tool_poses', self._on_poses, 10,
                                 callback_group=group)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10,
                                 callback_group=group)
        ActionServer(self, GripperCommand, '/gripper_controller/gripper_cmd', self._execute,
                     callback_group=group)
        # Gazebo creates every detachable joint attached when the robot spawns. Release all
        # tools once the robot is up (first finger joint state), a few times to be safe.
        self._robot_up = False
        self._release_count = 0
        self._release_timer = self.create_timer(0.5, self._release_all, callback_group=group)

    def _release_all(self):
        if not self._robot_up:
            return
        for pub in self._detach_pubs.values():
            pub.publish(Empty())
        self._release_count += 1
        if self._release_count >= 6:
            self._release_timer.cancel()
            self.get_logger().info('Released all tools; gripper ready')

    def _on_poses(self, msg):
        with self._lock:
            for t in msg.transforms:
                if tool_class(t.child_frame_id):
                    p = t.transform.translation
                    self._tool_poses[t.child_frame_id] = np.array([p.x, p.y, p.z])

    def _on_joints(self, msg):
        if 'finger_left_joint' in msg.name:
            self._finger = msg.position[msg.name.index('finger_left_joint')]
            self._robot_up = True

    def _tool_between_fingers(self):
        try:
            tf = self._buffer.lookup_transform('world', 'tool0', Time()).transform
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f'no tool0 pose: {e}')
            return None
        r = rotation_matrix(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
        t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        with self._lock:
            poses = dict(self._tool_poses)
        for name, position in poses.items():
            if name in self._attach_pubs and in_grasp_region(r.T @ (position - t)):
                return name
        return None

    def _execute(self, goal_handle):
        target = min(max(goal_handle.request.command.position, 0.0), self._stroke)
        gap = 2 * (self._stroke - target)
        stalled = False
        if gap < self._tool_width:
            if self._attached is None:
                self._attached = self._tool_between_fingers()
                if self._attached is not None:
                    self._attach_pubs[self._attached].publish(Empty())
                    self.get_logger().info(f'grasped {self._attached}')
            if self._attached is not None:
                target = self._stroke - self._tool_width / 2
                stalled = True
        elif self._attached is not None:
            self._detach_pubs[self._attached].publish(Empty())
            self.get_logger().info(f'released {self._attached}')
            self._released_pub.publish(String(data=self._attached))
            self._attached = None

        self._command_pub.publish(Float64MultiArray(data=[target, target]))
        deadline = time.monotonic() + 3.0
        while abs(self._finger - target) > 1e-3 and time.monotonic() < deadline:
            time.sleep(0.02)

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = float(self._finger)
        result.stalled = stalled
        result.reached_goal = not stalled
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimGripper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
