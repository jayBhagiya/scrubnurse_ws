import math

import cv2
import numpy as np
import pytest
from scrubnurse_perception.orientation import estimate_orientation


def tool_outline(alpha, cx=960, cy=540):
    """Outline of a scissors-like tool: thin blade toward alpha, ring handle opposite."""
    img = np.zeros((1080, 1920), np.uint8)
    d = np.array([math.cos(alpha), math.sin(alpha)])
    tip = np.array([cx, cy]) + 220 * d
    handle = np.array([cx, cy]) - 160 * d
    cv2.line(img, tuple(map(int, handle)), tuple(map(int, tip)), 255, 14)
    cv2.circle(img, tuple(map(int, handle)), 60, 255, -1)
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    points = contours[0].reshape(-1, 2)
    x, y, w, h = cv2.boundingRect(points)
    return points, (x + w / 2, y + h / 2)


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


@pytest.mark.parametrize('degrees', range(0, 360, 15))
def test_heading_points_to_heavier_end(degrees):
    alpha = math.radians(degrees)
    theta = estimate_orientation(*tool_outline(alpha))
    assert angle_diff(theta, alpha + math.pi) < math.radians(10)


@pytest.mark.parametrize('degrees', range(0, 360, 15))
def test_flip_points_to_lighter_end(degrees):
    alpha = math.radians(degrees)
    theta = estimate_orientation(*tool_outline(alpha), flip=True)
    assert angle_diff(theta, alpha) < math.radians(10)


def test_degenerate_outline_has_no_orientation():
    assert estimate_orientation([[0, 0], [10, 0], [10, 10]], (5, 5)) is None
    assert estimate_orientation([], (0, 0)) is None
