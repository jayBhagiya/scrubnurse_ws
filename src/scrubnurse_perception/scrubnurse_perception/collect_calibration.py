"""
Record pixel/robot point pairs for the camera-to-table homography.

The ROS 2 port of od_homography/calibration.py, in the terminal instead of a
Tk window. For each pair: put one tool on the table, press Enter to record
where the detector sees it, then move the gripper (tool0) onto the tool and
press Enter to record its x/y. Pairs are appended to a CSV for
compute_homography; the fit is printed as you go.

    ros2 run scrubnurse_perception collect_calibration --tool A -o pairs.csv
"""

import argparse
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scrubnurse_perception import homography
from scrubnurse_perception.compute_homography import to_params_yaml
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray


class CalibrationRecorder(Node):

    def __init__(self, tool, base_frame, tip_frame):
        super().__init__('calibration_recorder')
        self._tool = tool
        self._base_frame, self._tip_frame = base_frame, tip_frame
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._detections = None
        self.create_subscription(Detection2DArray, 'tool_detections_2d', self._on_detections, 10)

    def _on_detections(self, msg):
        self._detections = msg

    def pixel(self):
        """Return (u, v) of the single detected tool, or raise ValueError."""
        if self._detections is None:
            raise ValueError('no detections received yet (is tool_detector running?)')
        hits = [d for d in self._detections.detections
                if d.results and d.results[0].hypothesis.class_id == self._tool]
        if len(hits) != 1:
            raise ValueError(f'expected exactly one tool {self._tool} in view, found {len(hits)}')
        center = hits[0].bbox.center.position
        return center.x, center.y

    def robot_xy(self):
        t = self._buffer.lookup_transform(self._base_frame, self._tip_frame, Time()).transform
        return t.translation.x, t.translation.y


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('-o', '--output', required=True, help='CSV file to append u,v,x,y rows to')
    parser.add_argument('--tool', default='A', help='class id of the tool used as target')
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--tip-frame', default='tool0')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = CalibrationRecorder(args.tool, args.base_frame, args.tip_frame)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    if not os.path.exists(args.output):
        with open(args.output, 'w') as f:
            f.write('u,v,x,y\n')
    pairs = np.loadtxt(args.output, delimiter=',', skiprows=1, ndmin=2).reshape(-1, 4).tolist()
    print(f'{len(pairs)} pair(s) in {args.output}. Spread the points over the whole work area.')
    try:
        while True:
            input(f'\nPut one tool {args.tool} on the table, then press Enter '
                  '(Ctrl-C to finish)... ')
            try:
                u, v = node.pixel()
            except ValueError as e:
                print(e)
                continue
            print(f'pixel ({u:.1f}, {v:.1f})')
            input('Move the gripper (tool0) onto the tool, then press Enter... ')
            try:
                x, y = node.robot_xy()
            except Exception as e:  # tf2 raises several unrelated exception types
                print(f'cannot read {args.tip_frame}: {e}')
                continue
            pairs.append([u, v, x, y])
            with open(args.output, 'a') as f:
                f.write(f'{u},{v},{x},{y}\n')
            print(f'robot ({x:.4f}, {y:.4f}); {len(pairs)} pair(s) saved')
            if len(pairs) >= 4:
                data = np.array(pairs)
                h = homography.estimate(data[:, :2], data[:, 2:])
                errors = homography.reprojection_errors(h, data[:, :2], data[:, 2:])
                print(f'fit: mean {errors.mean() * 1000:.1f} mm, max {errors.max() * 1000:.1f} mm')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if len(pairs) >= 4:
        data = np.array(pairs)
        print(to_params_yaml(homography.estimate(data[:, :2], data[:, 2:])), end='')
