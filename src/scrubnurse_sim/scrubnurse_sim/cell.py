"""
Geometry of the simulated cell, shared by the simulation helper nodes.

Gazebo reports poses in its world frame, which is the robot's base_link. Cell
coordinates (poses.yaml, the homography, the return area) are in the UR `base`
frame, rotated 180 degrees about z.
"""

from dataclasses import dataclass
import math

import numpy as np

TOOL_PREFIX = 'tool_'
# Height of a tool's centre lying on the work table (table top 0.095 + half of 12 mm).
TOOL_ON_TABLE_Z = 0.101


def tool_class(model_name):
    """Return the tool id of a Gazebo tool model ('tool_C_table' -> 'C'), else None."""
    if not model_name.startswith(TOOL_PREFIX):
        return None
    return model_name[len(TOOL_PREFIX):].split('_')[0] or None


def world_to_base(x, y, z, yaw):
    """Convert a Gazebo world position and yaw into the UR base frame."""
    return -x, -y, z, math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))


def base_to_world(x, y, z, yaw):
    """Convert a UR base frame position and yaw into Gazebo's world frame."""
    return world_to_base(x, y, z, yaw)


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


@dataclass(frozen=True)
class OverheadCamera:
    """
    Pinhole camera looking straight down, as in worlds/scrubnurse_cell.sdf.

    Mounted like the real one: image right is base -y, image down is base -x.
    """

    position: tuple = (0.016, -1.0, 0.671)  # in base
    hfov: float = math.pi / 2
    width: int = 1920
    height: int = 1080

    @property
    def focal(self):
        return (self.width / 2) / math.tan(self.hfov / 2)

    def project(self, x, y, z):
        """Pixel (u, v) of a point given in base."""
        cx, cy, cz = self.position
        depth = cz - z
        return (self.width / 2 - self.focal * (y - cy) / depth,
                self.height / 2 - self.focal * (x - cx) / depth)

    def heading(self, yaw):
        """Image angle atan2(dv, du) of a direction with the given yaw in base."""
        return math.atan2(-math.cos(yaw), -math.sin(yaw))

    def homography(self, plane_z=TOOL_ON_TABLE_Z):
        """Matrix mapping pixels to (x, y) in base on the plane z = plane_z."""
        cx, cy, cz = self.position
        s = (cz - plane_z) / self.focal
        return np.array([[0.0, -s, cx + s * self.height / 2],
                         [-s, 0.0, cy + s * self.width / 2],
                         [0.0, 0.0, 1.0]])
