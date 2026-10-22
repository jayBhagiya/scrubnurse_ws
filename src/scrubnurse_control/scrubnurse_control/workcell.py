"""Calibrated geometry of the table area watched by the camera (UR base frame, metres)."""

from dataclasses import dataclass


def _lerp(x, x0, x1, y0, y1):
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


@dataclass(frozen=True)
class Workcell:
    """
    Return area and grasp heights of the real cell.

    The defaults are the values measured on the ROS 1 setup: tools put down in
    `return_area` are picked up again, from `approach_height` down to a grasp
    height interpolated between four measured corner heights of the table.
    """

    return_area: tuple = (-0.18, -1.20, 0.22, -0.75)  # x_min, y_min, x_max, y_max
    approach_height: float = 0.38
    grasp_x_range: tuple = (-0.18, 0.22)
    grasp_y_range: tuple = (-0.75, -1.2)
    grasp_z_at_y0: tuple = (0.24615, 0.24691)  # z at (x0, y0) and (x1, y0)
    grasp_z_at_y1: tuple = (0.24810, 0.24900)  # z at (x0, y1) and (x1, y1)

    def contains(self, x, y):
        x_min, y_min, x_max, y_max = self.return_area
        return x_min <= x <= x_max and y_min <= y <= y_max

    def grasp_height(self, x, y):
        """Bilinear interpolation of the measured table heights at (x, y)."""
        x0, x1 = self.grasp_x_range
        y0, y1 = self.grasp_y_range
        z_y0 = _lerp(x, x0, x1, *self.grasp_z_at_y0)
        z_y1 = _lerp(x, x0, x1, *self.grasp_z_at_y1)
        return _lerp(y, y0, y1, z_y0, z_y1)


def select_return_candidate(tools, workcell):
    """Return the first tool inside the return area, ordered by (x, y), or None."""
    inside = [t for t in tools if workcell.contains(t.position[0], t.position[1])]
    return min(inside, key=lambda t: (t.position[0], t.position[1]), default=None)
