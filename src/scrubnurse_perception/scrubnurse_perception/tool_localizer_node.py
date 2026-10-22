"""
Turn pixel tool detections into grasp poses on the table, in the robot base frame.

Subscribes to `tool_detections_2d` (vision_msgs/Detection2DArray) and publishes
`tool_detections` (vision_msgs/Detection3DArray) with one top-down tool0 grasp
pose per tool, plus a TF frame `<tf_prefix><class>_<n>` for each.
"""

import math

from geometry_msgs.msg import Pose, TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from scrubnurse_perception import homography
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class ToolLocalizer(Node):

    def __init__(self):
        super().__init__('tool_localizer')
        values = self.declare_parameter('homography', Parameter.Type.DOUBLE_ARRAY).value
        if values is None or len(values) != 9:
            raise ValueError('parameter "homography" must hold the 9 values of a 3x3 matrix')
        self._h = homography.as_matrix(values)
        self._frame_id = self.declare_parameter('frame_id', 'base').value
        self._roll = self.declare_parameter('grasp_roll', -math.pi).value
        self._tf_prefix = self.declare_parameter('tf_prefix', 'tool_').value
        publish_tf = self.declare_parameter('publish_tf', True).value

        self._tf = TransformBroadcaster(self) if publish_tf else None
        self._pub = self.create_publisher(Detection3DArray, 'tool_detections', 10)
        self.create_subscription(Detection2DArray, 'tool_detections_2d', self._on_detections, 10)

    def _on_detections(self, msg):
        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        counts = {}
        transforms = []
        for det in msg.detections:
            if not det.results:
                continue
            hypothesis = det.results[0].hypothesis
            theta = det.bbox.center.theta
            if math.isnan(theta):
                self.get_logger().warn(
                    f'{hypothesis.class_id}: no orientation, skipped', throttle_duration_sec=5.0)
                continue

            x, y = homography.pixel_to_plane(
                self._h, det.bbox.center.position.x, det.bbox.center.position.y)
            pose = Pose()
            pose.position.x, pose.position.y = x, y
            q = homography.grasp_orientation(theta, self._roll)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

            index = counts.get(hypothesis.class_id, 0)
            counts[hypothesis.class_id] = index + 1
            tool = Detection3D(header=out.header, id=f'{hypothesis.class_id}_{index}')
            result = ObjectHypothesisWithPose(hypothesis=hypothesis)
            result.pose.pose = pose
            tool.results.append(result)
            tool.bbox.center = pose
            out.detections.append(tool)

            if self._tf is not None:
                t = TransformStamped(header=out.header, child_frame_id=self._tf_prefix + tool.id)
                t.transform.translation.x, t.transform.translation.y = x, y
                t.transform.rotation = pose.orientation
                transforms.append(t)

        self._pub.publish(out)
        if transforms:
            self._tf.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ToolLocalizer()
    except ValueError as e:
        rclpy.logging.get_logger('tool_localizer').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
