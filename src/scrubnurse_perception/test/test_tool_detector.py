import numpy as np
from scrubnurse_perception.tool_detector_node import zoom_roi


def test_zoomed_pixel_maps_back_to_frame():
    frame = np.zeros((1080, 1920, 3), np.uint8)
    frame[500, 1000] = 255
    roi = np.array([[875, 750], [1384, 750], [1384, 298], [875, 298]], np.int32)
    image, corner = zoom_roi(frame, roi, 1.35)
    assert image.shape == frame.shape
    y, x = np.argwhere(image[:, :, 0] > 0).mean(axis=0)
    assert np.allclose(np.array([x, y]) / 1.35 + corner, [1000, 500], atol=1)
