"""Planar homography between camera pixels and the table plane in the robot base frame."""

import math

import numpy as np


def as_matrix(values):
    """Return a normalised 3x3 homography from nine row-major numbers or a 3x3 array."""
    h = np.asarray(values, dtype=float).reshape(3, 3)
    return h / h[2, 2]


def pixel_to_plane(h, u, v):
    """Map pixel (u, v) to (x, y) on the calibrated plane."""
    p = h @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def estimate(pixels, points):
    """Least-squares homography from N >= 4 pixel/plane correspondences."""
    import cv2

    pixels = np.asarray(pixels, dtype=float)
    points = np.asarray(points, dtype=float)
    if len(pixels) < 4 or len(pixels) != len(points):
        raise ValueError('need at least 4 matching pixel/point pairs')
    h, _ = cv2.findHomography(pixels, points, 0)
    if h is None:
        raise ValueError('homography estimation failed (degenerate points?)')
    return as_matrix(h)


def reprojection_errors(h, pixels, points):
    """Euclidean error of each mapped pixel against its measured plane point."""
    mapped = np.array([pixel_to_plane(h, u, v) for u, v in np.asarray(pixels, dtype=float)])
    return np.linalg.norm(mapped - np.asarray(points, dtype=float), axis=1)


def quaternion_from_euler(roll, pitch, yaw):
    """Return the (x, y, z, w) quaternion for static-axis roll, pitch, yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def grasp_orientation(image_theta, roll=-math.pi):
    """
    Top-down tool0 orientation for a tool lying at `image_theta` in the image.

    The yaw is the negated image angle: on the calibrated cell the camera looks
    down with its image axes mirrored with respect to the robot base. This is the
    empirical rule from the ROS 1 node, kept so existing calibrations still work.
    """
    return quaternion_from_euler(roll, 0.0, -image_theta)
