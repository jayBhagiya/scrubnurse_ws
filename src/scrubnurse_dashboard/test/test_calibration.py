import cv2
import numpy as np
import pytest
from scrubnurse_dashboard.calibration import Calibration, find_block
from scrubnurse_perception import homography

H = homography.as_matrix([[2e-5, -9.8e-4, 0.52], [-1.07e-3, -6e-5, 0.157], [7.9e-5, 3.9e-5, 1.0]])


def table(*blocks, size=(1080, 1920)):
    """Light wood-coloured frame with dark squares given as (u, v, side, angle in degrees)."""
    image = np.full((*size, 3), (120, 170, 210), np.uint8)
    for u, v, side, angle in blocks:
        corners = cv2.boxPoints(((u, v), (side, side), angle))
        cv2.fillPoly(image, [corners.astype(np.int32)], (25, 25, 25))
    return image


@pytest.mark.parametrize('angle', [0, 30, 45])
def test_find_block(angle):
    (u, v), corners = find_block(table((1100.0, 500.0, 44, angle)), 44)
    assert abs(u - 1100) < 1 and abs(v - 500) < 1
    assert len(corners) == 4


def test_find_block_only_in_roi():
    image = table((1100, 500, 44, 0), (300, 300, 44, 0))
    with pytest.raises(ValueError, match='2 dark blocks'):
        find_block(image, 44)
    roi = [[875, 750], [1384, 750], [1384, 298], [875, 298]]
    assert find_block(image, 44, roi)[0] == pytest.approx((1100, 500), abs=1)


@pytest.mark.parametrize('blocks', [(), ((1100, 500, 120, 0),), ((1100, 500, 15, 0),)])
def test_no_block_of_that_size(blocks):
    with pytest.raises(ValueError, match='no dark block'):
        find_block(table(*blocks), 44)


def test_dark_rectangle_is_not_the_block():
    image = table()
    cv2.rectangle(image, (1000, 400), (1060, 440), (25, 25, 25), -1)  # 60 x 40 px
    with pytest.raises(ValueError):
        find_block(image, 44)


def record(calibration, pixels):
    for u, v in pixels:
        calibration.set_pixel(u, v)
        calibration.add_reference(*homography.pixel_to_plane(H, u, v))


PIXELS = [(900, 320), (1350, 320), (900, 720), (1350, 720), (1120, 520), (1000, 600)]


def test_pairs_and_fit(tmp_path):
    calibration = Calibration(str(tmp_path))
    with pytest.raises(ValueError):
        calibration.add_reference(0.1, -0.9)  # no pixel yet
    record(calibration, PIXELS[:3])
    assert calibration.state()['fit'] is None
    record(calibration, PIXELS[3:])
    state = calibration.state()
    np.testing.assert_allclose(state['fit']['homography'], H.ravel(), rtol=1e-5, atol=1e-8)
    assert state['fit']['unseen_max'] < 1e-6
    assert state['pairs'][0]['error'] < 1e-6
    calibration.remove(5)
    assert len(calibration.state()['pairs']) == 5
    with pytest.raises(ValueError):
        calibration.remove(5)


def test_save_and_load(tmp_path):
    calibration = Calibration(str(tmp_path / 'calibration'))
    assert calibration.files() == []
    with pytest.raises(ValueError):
        calibration.save()
    record(calibration, PIXELS)
    pairs_path, params_path = calibration.save()
    assert 'homography: [' in open(params_path).read()
    loaded = Calibration(str(tmp_path / 'calibration'))
    loaded.load(calibration.files()[0])
    assert len(loaded.state()['pairs']) == len(PIXELS)
    with pytest.raises(ValueError):
        loaded.load('../../etc/passwd')


def test_degenerate_pairs_report_instead_of_failing(tmp_path):
    calibration = Calibration(str(tmp_path))
    for u, v in PIXELS:
        calibration.set_pixel(u, v)
        calibration.add_reference(0.05, -0.95)  # the arm was never moved
    state = calibration.state()
    assert state['fit'] is None and state['fit_error']
