"""In-image orientation of a surgical tool from its segmentation outline."""

import math

import cv2
import numpy as np


def estimate_orientation(outline, center, flip=False):
    """
    Return the tool heading in image coordinates (radians), or None.

    An ellipse fitted to the outline gives the tool axis up to 180 degrees. The
    outline is split across that axis at `center`, and the heading points
    toward the half with more area (e.g. the finger rings of scissors). `flip`
    makes it point toward the half with less area instead; the forceps classes
    need that because their shape is weighted the other way round.
    """
    points = np.asarray(outline, dtype=np.int32).reshape(-1, 2)
    if len(points) < 3:
        return None
    approx = cv2.approxPolyDP(points, 0.0001 * cv2.arcLength(points, True), True)
    if len(approx) <= 5:
        return None
    _, _, angle_deg = cv2.fitEllipse(approx)
    theta = math.radians(angle_deg)

    cx, cy = center
    dx, dy = math.cos(theta), math.sin(theta)
    # Signed side of each outline point w.r.t. the line through center along theta
    # (same sign convention as the ROS 1 detector's line equation).
    side = dx * (points[:, 1] - cy) - dy * (points[:, 0] - cx)
    area_pos = _area(points[side > 0])
    area_neg = _area(points[side <= 0])

    smaller_is_positive = area_pos < area_neg
    if flip:
        smaller_is_positive = not smaller_is_positive
    return theta - math.pi / 2 if smaller_is_positive else theta + math.pi / 2


def _area(points):
    return cv2.contourArea(points) if len(points) > 0 else float('inf')
