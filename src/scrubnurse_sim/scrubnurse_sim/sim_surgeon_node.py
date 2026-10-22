"""
Simulated surgeon for the Gazebo cell.

Takes an instrument as soon as the gripper releases it at the handover pose.
The hand is only a visual marker, as the real surgeon's hand is gone by the
time the robot retreats. After `use_time` seconds the surgeon lays the
instrument down in the table's return area, where the camera sees it and the
controller puts it back into the return rack. Tools are moved with Gazebo's
set_pose service.
"""

import itertools
import math
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scrubnurse_sim.cell import base_to_world, tool_class, world_to_base, yaw_of
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# Where used instruments are put down: inside the return area, flat on the table (base frame).
TABLE_SPOTS = [(0.08, -0.85, 0.4), (-0.06, -1.05, -0.6), (0.14, -1.08, 1.2), (-0.10, -0.85, 0.0)]


def set_pose(world, name, x, y, z, yaw):
    """Move a Gazebo model (world frame); return True on success."""
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    request = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
               f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
    try:
        result = subprocess.run(
            ['ign', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000', '--req', request],
            capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return 'data: true' in result.stdout


class SimSurgeon(Node):

    def __init__(self):
        super().__init__('sim_surgeon')
        self._world = self.declare_parameter('world', 'scrubnurse_cell').value
        self._hand = self.declare_parameter('hand_position', [-0.379, -0.463, 0.290]).value
        self._hand_radius = self.declare_parameter('hand_radius', 0.06).value
        self._use_time = self.declare_parameter('use_time', 15.0).value
        self._table_z = self.declare_parameter('table_height', 0.095).value + 0.006
        self._spots = itertools.cycle(TABLE_SPOTS)
        self._held = 0
        self._poses = {}
        self._lay_down_timers = {}
        self.create_subscription(TFMessage, 'tool_poses', self._on_poses, 10)
        self.create_subscription(String, '/scrubnurse_sim/released', self._on_released, 10)

    def _on_poses(self, msg):
        for t in msg.transforms:
            if tool_class(t.child_frame_id):
                p, q = t.transform.translation, t.transform.rotation
                self._poses[t.child_frame_id] = world_to_base(
                    p.x, p.y, p.z, yaw_of(q.x, q.y, q.z, q.w))

    def _on_released(self, msg):
        name = msg.data
        pose = self._poses.get(name)
        if pose is None or math.dist(pose[:3], self._hand) > self._hand_radius:
            return
        # Off the cell (on the floor next to the table) while the surgeon uses it.
        self._held += 1
        if not self._move(name, 0.55, -0.40 - 0.08 * (self._held % 5), 0.006, 0.0):
            return
        self.get_logger().info(f'took {name}; putting it back on the table in '
                               f'{self._use_time:.0f} s')
        self._lay_down_timers[name] = self.create_timer(
            self._use_time, lambda: self._lay_down(name))

    def _lay_down(self, name):
        self._lay_down_timers.pop(name).cancel()
        x, y, yaw = next(self._spots)
        if self._move(name, x, y, self._table_z, yaw):
            self.get_logger().info(f'put {name} down at ({x:+.2f}, {y:+.2f}) for return')

    def _move(self, name, x, y, z, yaw):
        if set_pose(self._world, name, *base_to_world(x, y, z, yaw)):
            return True
        self.get_logger().error(f'could not move {name} (Gazebo set_pose failed)')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SimSurgeon()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
