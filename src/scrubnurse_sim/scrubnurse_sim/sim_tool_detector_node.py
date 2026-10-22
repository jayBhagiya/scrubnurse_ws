"""
Ground-truth tool detector for the simulated cell.

Stands in for the YOLO detector, which is trained on real instruments and does
not recognise the simulated stand-ins. Tools lying on the work table are
projected into the overhead camera and published like tool_detector does
(vision_msgs/Detection2DArray on `tool_detections_2d`, heading in
bbox.center.theta), so tool_localizer and the controller run unchanged.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scrubnurse_sim.cell import OverheadCamera, tool_class, world_to_base, yaw_of
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class SimToolDetector(Node):

    def __init__(self):
        super().__init__('sim_tool_detector')
        self._camera = OverheadCamera(
            position=tuple(self.declare_parameter('camera_position', [0.016, -1.0, 0.671]).value),
            hfov=self.declare_parameter('camera_hfov', math.pi / 2).value,
            width=self.declare_parameter('camera_width', 1920).value,
            height=self.declare_parameter('camera_height', 1080).value)
        self._table_z = self.declare_parameter('table_height', 0.095).value
        self._frame_id = self.declare_parameter('frame_id', 'overhead_camera').value
        rate = self.declare_parameter('rate', 5.0).value
        self._poses = {}
        self.create_subscription(TFMessage, 'tool_poses', self._on_poses, 10)
        self._pub = self.create_publisher(Detection2DArray, 'tool_detections_2d', 10)
        self.create_timer(1.0 / rate, self._publish)

    def _on_poses(self, msg):
        for t in msg.transforms:
            if tool_class(t.child_frame_id):
                self._poses[t.child_frame_id] = t.transform

    def _publish(self):
        out = Detection2DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        for name, tf in self._poses.items():
            p, q = tf.translation, tf.rotation
            x, y, z, yaw = world_to_base(p.x, p.y, p.z, yaw_of(q.x, q.y, q.z, q.w))
            # Only tools lying on the table count: not in the magazine, trays or gripper.
            if not self._table_z <= z <= self._table_z + 0.03:
                continue
            u, v = self._camera.project(x, y, z)
            if not (0 <= u < self._camera.width and 0 <= v < self._camera.height):
                continue
            det = Detection2D(header=out.header)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = tool_class(name)
            hypothesis.hypothesis.score = 1.0
            det.results.append(hypothesis)
            det.bbox.center.position.x, det.bbox.center.position.y = u, v
            det.bbox.center.theta = self._camera.heading(yaw)
            det.bbox.size_x = det.bbox.size_y = 0.08 * self._camera.focal / (
                self._camera.position[2] - z)
            out.detections.append(det)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimToolDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
