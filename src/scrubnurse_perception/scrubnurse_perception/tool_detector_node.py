"""
Segment surgical tools with YOLO and publish their pixel position and heading.

Publishes vision_msgs/Detection2DArray on `tool_detections_2d`. Each detection
carries the class id (A..F) and score; `bbox` is the axis-aligned box and
`bbox.center.theta` is the tool heading in the image (radians), or NaN when the
mask was too small to fit an ellipse.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scrubnurse_perception.orientation import estimate_orientation
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Work area on the real cell's table (pixels of the 1920x1080 overhead camera).
DEFAULT_ROI = [563, 834, 1241, 838, 1252, 228, 562, 222]


class ToolDetector(Node):

    def __init__(self):
        super().__init__('tool_detector')
        share = get_package_share_directory('scrubnurse_perception')
        weights = self.declare_parameter(
            'weights', os.path.join(share, 'models', 'tool_seg_best_train3.pt')).value
        self._device = self.declare_parameter('device', 'cuda:0').value
        self._image_size = self.declare_parameter('image_size', 640).value
        self._confidence = self.declare_parameter('confidence_threshold', 0.5).value
        self._class_ids = list(
            self.declare_parameter('class_ids', ['A', 'B', 'C', 'D', 'E', 'F']).value)
        self._flip_ids = set(self.declare_parameter('flip_orientation_ids', ['E', 'F']).value)
        use_roi = self.declare_parameter('use_roi', True).value
        roi = self.declare_parameter('roi_polygon', DEFAULT_ROI).value
        self._roi = np.array(roi, dtype=np.int32).reshape(-1, 2) if use_roi else None
        rate = self.declare_parameter('rate', 15.0).value
        self._annotate = self.declare_parameter('publish_annotated', True).value
        self._display = self.declare_parameter('display', False).value

        import torch
        from ultralytics import YOLO

        if self._device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('CUDA not available, running YOLO on CPU')
            self._device = 'cpu'
        self._model = YOLO(weights)
        self.get_logger().info(f'Loaded {weights} on {self._device}')

        self._bridge = CvBridge()
        self._latest = None
        self.create_subscription(Image, 'image', self._on_image, qos_profile_sensor_data)
        self._pub = self.create_publisher(Detection2DArray, 'tool_detections_2d', 10)
        if self._annotate:
            self._annotated_pub = self.create_publisher(Image, '~/annotated_image', 1)
        self.create_timer(1.0 / rate, self._process)

    def _on_image(self, msg):
        self._latest = msg

    def _process(self):
        msg, self._latest = self._latest, None
        if msg is None:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self._model.predict(
            frame, imgsz=self._image_size, conf=self._confidence, device=self._device,
            verbose=False)[0]

        out = Detection2DArray(header=msg.header)
        boxes = result.boxes
        outlines = result.masks.xy if result.masks is not None else [None] * len(boxes)
        for xyxy, cls, score, outline in zip(
                boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist(), outlines):
            x1, y1, x2, y2 = xyxy
            center = ((x1 + x2) / 2, (y1 + y2) / 2)
            if self._roi is not None and cv2.pointPolygonTest(self._roi, center, False) < 0:
                continue
            class_id = self._class_name(int(cls))
            theta = None
            if outline is not None and len(outline) > 0:
                theta = estimate_orientation(outline, center, flip=class_id in self._flip_ids)
            out.detections.append(self._detection(msg.header, class_id, score, xyxy, theta))
        self._pub.publish(out)

        if self._annotate or self._display:
            image = self._draw(frame, out)
            if self._annotate:
                annotated = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
                annotated.header = msg.header
                self._annotated_pub.publish(annotated)
            if self._display:
                cv2.imshow('scrubnurse tool detector', cv2.resize(image, (960, 540)))
                cv2.waitKey(1)

    def _class_name(self, index):
        return self._class_ids[index] if index < len(self._class_ids) else str(index)

    @staticmethod
    def _detection(header, class_id, score, xyxy, theta):
        x1, y1, x2, y2 = xyxy
        det = Detection2D(header=header)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = float(score)
        det.results.append(hypothesis)
        det.bbox.center.position.x = (x1 + x2) / 2
        det.bbox.center.position.y = (y1 + y2) / 2
        det.bbox.center.theta = math.nan if theta is None else theta
        det.bbox.size_x = x2 - x1
        det.bbox.size_y = y2 - y1
        return det

    def _draw(self, frame, detections):
        image = frame.copy()
        if self._roi is not None:
            cv2.polylines(image, [self._roi], True, (100, 0, 100), 2)
        for det in detections.detections:
            c = det.bbox.center
            cx, cy = int(c.position.x), int(c.position.y)
            hw, hh = int(det.bbox.size_x / 2), int(det.bbox.size_y / 2)
            cv2.rectangle(image, (cx - hw, cy - hh), (cx + hw, cy + hh), (255, 0, 0), 2)
            cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)
            label = det.results[0].hypothesis.class_id
            if not math.isnan(c.theta):
                tip = (int(cx + 150 * math.cos(c.theta)), int(cy + 150 * math.sin(c.theta)))
                cv2.arrowedLine(image, (cx, cy), tip, (0, 255, 255), 2)
                label += f' {math.degrees(c.theta):.0f}deg'
            cv2.putText(image, label, (cx - hw, cy - hh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2, cv2.LINE_AA)
        return image


def main(args=None):
    rclpy.init(args=args)
    node = ToolDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
