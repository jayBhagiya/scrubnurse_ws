import os

import cv2
import numpy as np
import pytest
from scrubnurse_perception.kinect_camera_node import camera_info_from_yaml, connected_serials
from scrubnurse_perception.kinect_camera_node import open_camera, to_bgr

CELL_CAMERA = os.path.join(os.path.dirname(__file__), '..', '..', 'scrubnurse_bringup', 'config',
                           'azure_kinect_rgb_1080P.yaml')


def test_mjpg_frame_decodes_to_bgr():
    image = np.zeros((108, 192, 3), np.uint8)
    image[:, :96] = (255, 0, 0)
    ok, jpeg = cv2.imencode('.jpg', image)
    assert ok
    decoded = to_bgr(jpeg.ravel(), 'COLOR_MJPG')
    assert decoded.shape == (108, 192, 3)
    assert decoded[50, 10, 0] > 200 and decoded[50, 150, 0] < 50


def test_bgra_frame_drops_alpha():
    bgra = np.dstack([np.full((4, 6), c, np.uint8) for c in (1, 2, 3, 255)])
    assert to_bgr(bgra, 'COLOR_BGRA32').shape == (4, 6, 3)


def test_unknown_format_rejected():
    with pytest.raises(ValueError):
        to_bgr(np.zeros(4, np.uint8), 'COLOR_RAW')


def test_cell_camera_info():
    if not os.path.exists(CELL_CAMERA):
        pytest.skip('scrubnurse_bringup sources not next to this package')
    info = camera_info_from_yaml(CELL_CAMERA)
    assert (info.width, info.height) == (1920, 1080)
    assert info.k[0] == pytest.approx(928.4355687562106)
    assert len(info.d) == 5 and len(info.p) == 12


class FakeDevice:
    def __init__(self, serial):
        self.serial, self.opened = serial, False

    def open(self):
        if self.opened:
            raise RuntimeError('device already open')
        self.opened = True

    def close(self):
        self.opened = False


class FakePyK4A:
    """Stand-in for the pyk4a module with cameras of the given serials."""

    def __init__(self, *serials):
        self.devices = [FakeDevice(s) for s in serials]

    def connected_device_count(self):
        return len(self.devices)

    def PyK4A(self, config, device_id):  # noqa: N802 (pyk4a's class name)
        return self.devices[device_id]


def test_open_camera_by_serial():
    pyk4a = FakePyK4A('000848193812', '000527700312')
    camera, device_id = open_camera(pyk4a, None, '000527700312')
    assert camera.serial == '000527700312' and camera.opened and device_id == 1
    assert not pyk4a.devices[0].opened  # the other one is closed again


def test_open_the_only_camera_without_serial():
    assert open_camera(FakePyK4A('000527700312'), None, '')[0].serial == '000527700312'


@pytest.mark.parametrize('serial, message', [
    ('', '2 Azure Kinects connected (000848193812, 000527700312)'),
    ('123', 'no Azure Kinect with serial 123'),
])
def test_open_camera_refuses_to_guess(serial, message):
    with pytest.raises(RuntimeError, match=message.replace('(', r'\(').replace(')', r'\)')):
        open_camera(FakePyK4A('000848193812', '000527700312'), None, serial)


def test_connected_serials_include_the_streaming_camera():
    pyk4a = FakePyK4A('000848193812', '000527700312')
    streaming, device_id = open_camera(pyk4a, None, '000527700312')
    assert connected_serials(pyk4a, None, device_id, streaming.serial) == [
        '000848193812', '000527700312']
    assert streaming.opened and not pyk4a.devices[0].opened


def test_connected_serials_skip_cameras_used_by_other_programs():
    pyk4a = FakePyK4A('000848193812', '000527700312')
    pyk4a.devices[1].open()  # another program streams from it
    streaming, device_id = open_camera(pyk4a, None, '000848193812')
    assert connected_serials(pyk4a, None, device_id, streaming.serial) == ['000848193812']


def test_open_camera_skips_cameras_used_by_other_programs():
    pyk4a = FakePyK4A('000848193812', '000527700312')
    pyk4a.devices[0].open()  # another program streams from it
    assert open_camera(pyk4a, None, '000527700312')[1] == 1
    with pytest.raises(RuntimeError, match='device 0 in use'):
        open_camera(pyk4a, None, '000848193812')
