import math
import os

import pytest
from scrubnurse_control.poses import load_poses
from scrubnurse_control.tasks import HANDOVER, required_poses, RETURN

CELL_POSES = os.path.join(os.path.dirname(__file__), '..', '..', 'scrubnurse_bringup', 'config',
                          'poses.yaml')


@pytest.fixture(scope='module')
def cell_poses():
    if not os.path.exists(CELL_POSES):
        pytest.skip('scrubnurse_bringup sources not next to this package')
    return load_poses(CELL_POSES)


def test_cell_poses_are_valid(cell_poses):
    assert len(cell_poses) == 38
    for name, pose in cell_poses.items():
        norm = math.sqrt(sum(v * v for v in pose.orientation))
        assert math.isclose(norm, 1.0, abs_tol=1e-3), name
        assert len(pose.joints) == 6, name


@pytest.mark.parametrize('tool', 'ABCDEF')
def test_cell_has_every_pose_the_sequences_need(cell_poses, tool):
    for steps in (HANDOVER, RETURN):
        assert set(required_poses(steps, tool)) <= set(cell_poses)


def test_malformed_pose_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('poses:\n  X:\n    position: [0, 0]\n    orientation: [0, 0, 0, 1]\n')
    with pytest.raises(ValueError, match='"X"'):
        load_poses(str(path))
