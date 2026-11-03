"""
Azure Kinect RGB-D camera, the ROS 2 port of media_streamer/kinect_streamer.py (pyk4a).

Publishes `image` (bgr8), `depth` (16UC1, millimetres, transformed into the
colour camera) and, when `camera_info_file` is set, `camera_info` from a ROS
camera calibration YAML. With several cameras connected, `serial` chooses one; setting
it at runtime switches cameras. `~/status` (scrubnurse_interfaces/CameraStatus,
latched) tells which camera streams and which are connected. Needs the Azure Kinect
SDK and pyk4a (requirements-camera.txt).
"""

import threading

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scrubnurse_interfaces.msg import CameraStatus
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


def open_camera(pyk4a, config, serial):
    """
    Open the Azure Kinect with this serial; with "" only if exactly one is connected.

    Return the opened device and its device index.

    The device order changes between boots, so with several cameras "the first one" is
    a different camera on different days.
    """
    count = pyk4a.connected_device_count()
    if count == 0:
        raise RuntimeError('no Azure Kinect connected')
    serials = []
    for device_id in range(count):
        device = pyk4a.PyK4A(config=config, device_id=device_id)
        try:
            device.open()
        except Exception:  # in use by another program
            serials.append(f'(device {device_id} in use)')
            continue
        if device.serial == serial or (not serial and count == 1):
            return device, device_id
        serials.append(device.serial)
        device.close()
    if not serial:
        raise RuntimeError(f'{count} Azure Kinects connected ({", ".join(serials)}): '
                           'set kinect_camera.serial to the overhead one')
    raise RuntimeError(f'no Azure Kinect with serial {serial}; connected: {", ".join(serials)}')


def connected_serials(pyk4a, config, open_id, open_serial):
    """
    List the serials of the Azure Kinects this node can use.

    `open_id`/`open_serial` is the device it streams from. Devices opened by another
    program cannot be opened to read their serial and are left out.
    """
    serials = []
    for device_id in range(pyk4a.connected_device_count()):
        if device_id == open_id:
            serials.append(open_serial)
            continue
        device = pyk4a.PyK4A(config=config, device_id=device_id)
        try:
            device.open()
        except Exception:  # in use by another program
            continue
        serials.append(device.serial)
        device.close()
    return serials


class KinectCamera(Node):

    def __init__(self):
        super().__init__('kinect_camera')
        self._configured_serial = serial = self.declare_parameter('serial', '').value
        self._color_format = self.declare_parameter('color_format', 'COLOR_MJPG').value
        resolution = self.declare_parameter('color_resolution', 'RES_1080P').value
        depth_mode = self.declare_parameter('depth_mode', 'WFOV_2X2BINNED').value
        fps = self.declare_parameter('fps', 'FPS_30').value
        synchronized = self.declare_parameter('synchronized_images_only', True).value
        self._publish_depth = self.declare_parameter('publish_depth', True).value
        self._frame_id = self.declare_parameter('frame_id', 'rgb_camera_link').value
        info_file = self.declare_parameter('camera_info_file', '').value

        import pyk4a

        self._pyk4a = pyk4a
        self._config = config = pyk4a.Config(
            color_resolution=pyk4a.ColorResolution[resolution],
            depth_mode=pyk4a.DepthMode[depth_mode],
            synchronized_images_only=synchronized,
            color_format=pyk4a.ImageFormat[self._color_format],
            camera_fps=pyk4a.FPS[fps])
        self._camera, self._device_id = open_camera(pyk4a, config, serial)
        self._camera.start()

        self._camera_info = camera_info_from_yaml(info_file) if info_file else None
        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, 'image', 10)
        self._depth_pub = self.create_publisher(Image, 'depth', 10)
        self._info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(CameraStatus, '~/status', latched)
        self._start_streaming()
        self.add_on_set_parameters_callback(self._on_parameters)

    def _start_streaming(self):
        self._running = True
        self._thread = threading.Thread(target=self._stream, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Streaming Azure Kinect {self._camera.serial}')
        self._status_pub.publish(CameraStatus(
            stamp=self.get_clock().now().to_msg(), serial=self._camera.serial,
            configured_serial=self._configured_serial,
            serials=connected_serials(self._pyk4a, self._config, self._device_id,
                                      self._camera.serial)))

    def _on_parameters(self, params):
        serial = next((p.value for p in params if p.name == 'serial'), None)
        if serial is None or serial == self._camera.serial:
            return SetParametersResult(successful=True)
        old = self._camera.serial
        self.close()  # pyk4a's stop() also closes the device
        try:
            self._camera, self._device_id = open_camera(self._pyk4a, self._config, serial)
            result = SetParametersResult(successful=True)
        except Exception as e:  # keep streaming from the previous camera
            self._camera, self._device_id = open_camera(self._pyk4a, self._config, old)
            result = SetParametersResult(successful=False, reason=str(e))
        self._camera.start()
        self._start_streaming()
        return result

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
