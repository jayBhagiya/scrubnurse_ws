"""Named robot poses (poses.yaml) and detected tool poses."""

from dataclasses import dataclass, field

import yaml


@dataclass(frozen=True)
class NamedPose:
    """A tool0 pose in the base frame: position (x, y, z), orientation (x, y, z, w)."""

    position: tuple
    orientation: tuple
    description: str = ''
    joints: tuple = field(default=())


@dataclass(frozen=True)
class DetectedTool:
    """A tool seen by the camera, with its top-down grasp pose in the base frame."""

    tool_id: str
    position: tuple
    orientation: tuple


def load_poses(path):
    """Load the `poses:` mapping of a poses.yaml file into NamedPose objects."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    poses = {}
    for name, entry in (data.get('poses') or {}).items():
        position, orientation = entry.get('position'), entry.get('orientation')
        if position is None or len(position) != 3 or orientation is None or len(orientation) != 4:
            raise ValueError(f'{path}: pose "{name}" needs a 3-value position and '
                             '4-value orientation')
        poses[str(name)] = NamedPose(
            position=tuple(float(v) for v in position),
            orientation=tuple(float(v) for v in orientation),
            description=entry.get('description', ''),
            joints=tuple(float(v) for v in entry.get('joints', ())),
        )
    return poses
