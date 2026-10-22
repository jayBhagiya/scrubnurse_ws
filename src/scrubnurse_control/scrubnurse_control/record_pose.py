"""
Record named tool0 poses and joint angles into a poses.yaml file.

Replaces the ROS 1 `scrubnurse_position_storing_loop`: jog the robot (pendant
or freedrive), press Enter, then give the pose a name and a description.

    ros2 run scrubnurse_control record_pose -o new_poses.yaml
"""

import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scrubnurse_control.motion import UR_JOINTS
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml


class PoseRecorder(Node):

    def __init__(self, base_frame, tip_frame):
        super().__init__('pose_recorder')
        self._base_frame, self._tip_frame = base_frame, tip_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._joints = None
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in UR_JOINTS):
            self._joints = [positions[j] for j in UR_JOINTS]

    def current(self):
        t = self._buffer.lookup_transform(self._base_frame, self._tip_frame, Time()).transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
                self._joints)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('-o', '--output', required=True, help='poses.yaml file to write')
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--tip-frame', default='tool0')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = PoseRecorder(args.base_frame, args.tip_frame)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    poses = {}
    try:
        while True:
            input('\nMove the robot, then press Enter to record (Ctrl-C to quit)... ')
            try:
                position, orientation, joints = node.current()
            except Exception as e:  # tf2 raises several unrelated exception types
                print(f'Cannot read {args.tip_frame} pose: {e}')
                continue
            print(f'position {position}\norientation {orientation}\njoints {joints}')
            name = input('Pose name (e.g. PA, RB; empty to discard): ').strip()
            if not name:
                continue
            entry = {'description': input('Description: ').strip(),
                     'position': position, 'orientation': orientation}
            if joints is not None:
                entry['joints'] = joints
            poses[name] = entry
            with open(args.output, 'w') as f:
                yaml.safe_dump({'poses': poses}, f, sort_keys=False, default_flow_style=None)
            print(f'Saved {len(poses)} pose(s) to {args.output}')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
