import os

import cv2
import numpy as np
import pytest
from scrubnurse_perception.kinect_camera_node import camera_info_from_yaml, to_bgr

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
