"""
Azure Kinect RGB-D camera, the ROS 2 port of media_streamer/kinect_streamer.py (pyk4a).

Publishes `image` (bgr8), `depth` (16UC1, millimetres, transformed into the
colour camera) and, when `camera_info_file` is set, `camera_info` from a ROS
camera calibration YAML. Needs the Azure Kinect SDK and pyk4a
(requirements-camera.txt).
"""

import threading

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml


def to_bgr(color, color_format):
    """Convert a raw Azure Kinect colour frame to a 3-channel BGR image."""
    if color_format == 'COLOR_MJPG':
        image = cv2.imdecode(np.frombuffer(color, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('MJPG frame could not be decoded')
        return image
    if color_format == 'COLOR_NV12':
        return cv2.cvtColor(color, cv2.COLOR_YUV2BGR_NV12)
    if color_format == 'COLOR_YUY2':
        return cv2.cvtColor(color, cv2.COLOR_YUV2BGR_YUY2)
    if color_format == 'COLOR_BGRA32':
        return np.ascontiguousarray(color[:, :, :3])
    raise ValueError(f'unsupported color format {color_format}')


def camera_info_from_yaml(path):
    """Read a ROS camera calibration YAML (camera_calibration format) into CameraInfo."""
    with open(path) as f:
        calib = yaml.safe_load(f)
    info = CameraInfo()
    info.width = int(calib['image_width'])
    info.height = int(calib['image_height'])
    info.distortion_model = calib.get('distortion_model', 'plumb_bob')
    info.k = [float(v) for v in calib['camera_matrix']['data']]
    info.d = [float(v) for v in calib['distortion_coefficients']['data']]
    info.r = [float(v) for v in calib.get('rectification_matrix', {}).get(
        'data', [1, 0, 0, 0, 1, 0, 0, 0, 1])]
    projection = calib.get('projection_matrix', {}).get('data')
    if projection is None:
        k = info.k
        projection = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
    info.p = [float(v) for v in projection]
    return info


class KinectCamera(Node):

    def __init__(self):
        super().__init__('kinect_camera')
        serial = self.declare_parameter('serial', '').value
        self._color_format = self.declare_parameter('color_format', 'COLOR_MJPG').value
        resolution = self.declare_parameter('color_resolution', 'RES_1080P').value
        depth_mode = self.declare_parameter('depth_mode', 'WFOV_2X2BINNED').value
        fps = self.declare_parameter('fps', 'FPS_30').value
        synchronized = self.declare_parameter('synchronized_images_only', True).value
        self._publish_depth = self.declare_parameter('publish_depth', True).value
        self._frame_id = self.declare_parameter('frame_id', 'rgb_camera_link').value
        info_file = self.declare_parameter('camera_info_file', '').value

        import pyk4a

        config = pyk4a.Config(
            color_resolution=pyk4a.ColorResolution[resolution],
            depth_mode=pyk4a.DepthMode[depth_mode],
            synchronized_images_only=synchronized,
            color_format=pyk4a.ImageFormat[self._color_format],
            camera_fps=pyk4a.FPS[fps])
        self._camera = self._open(pyk4a, config, serial)
        self._camera.start()

        self._camera_info = camera_info_from_yaml(info_file) if info_file else None
        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, 'image', 10)
        self._depth_pub = self.create_publisher(Image, 'depth', 10)
        self._info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self._running = True
        self._thread = threading.Thread(target=self._stream, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Streaming Azure Kinect {self._camera.serial}')

    def _open(self, pyk4a, config, serial):
        count = pyk4a.connected_device_count()
        if count == 0:
            raise RuntimeError('no Azure Kinect connected')
        for device_id in range(count):
            device = pyk4a.PyK4A(config=config, device_id=device_id)
            device.open()
            if not serial or device.serial == serial:
                return device
            device.close()
        raise RuntimeError(f'no Azure Kinect with serial {serial} among {count} device(s)')

    def _stream(self):
        try:
            self._stream_frames()
        except Exception as e:  # without this the thread dies silently and the node keeps running
            self.get_logger().fatal(f'camera stream stopped: {e}')
            rclpy.try_shutdown()

    def _stream_frames(self):
        while self._running and rclpy.ok():
            capture = self._camera.get_capture()
            if capture.color is None:
                continue
            try:
                bgr = to_bgr(capture.color, self._color_format)
            except ValueError as e:
                self.get_logger().warn(f'skipping frame: {e}', throttle_duration_sec=5.0)
                continue
            stamp = self.get_clock().now().to_msg()
            image = self._bridge.cv2_to_imgmsg(bgr, 'bgr8')
            image.header.stamp, image.header.frame_id = stamp, self._frame_id
            self._image_pub.publish(image)
            if self._camera_info is not None:
                self._camera_info.header = image.header
                self._info_pub.publish(self._camera_info)
            if self._publish_depth and capture.depth is not None:
                depth = self._bridge.cv2_to_imgmsg(capture.transformed_depth, '16UC1')
                depth.header = image.header
                self._depth_pub.publish(depth)

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._camera.stop()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KinectCamera()
    except (ImportError, RuntimeError) as e:
        rclpy.logging.get_logger('kinect_camera').fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
