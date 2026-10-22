import math
import os

import numpy as np
import pytest
from scrubnurse_sim.cell import base_to_world, OverheadCamera, tool_class, TOOL_ON_TABLE_Z
from scrubnurse_sim.cell import world_to_base
from scrubnurse_sim.sim_surgeon_node import TABLE_SPOTS
from scrubnurse_sim.sim_gripper_node import in_grasp_region
import yaml

SIM_PARAMS = os.path.join(os.path.dirname(__file__), '..', '..', 'scrubnurse_bringup', 'config',
                          'sim.yaml')


def test_tool_class():
    assert tool_class('tool_C_table') == 'C'
    assert tool_class('tool_A') == 'A'
    assert tool_class('overhead_camera') is None


def test_world_to_base_rotates_half_turn():
    x, y, z, yaw = world_to_base(0.8, 0.15, 0.3, 0.0)
    assert (x, y, z) == (-0.8, -0.15, 0.3)
    assert abs(abs(yaw) - math.pi) < 1e-12


def test_base_world_round_trip():
    x, y, z, yaw = base_to_world(*world_to_base(0.3, -0.2, 0.1, 0.7))
    assert (x, y, z, yaw) == pytest.approx((0.3, -0.2, 0.1, 0.7))


@pytest.mark.parametrize('spot', TABLE_SPOTS)
def test_surgeon_puts_tools_in_the_return_area(spot):
    x, y, _ = spot
    assert -0.18 <= x <= 0.22 and -1.20 <= y <= -0.75


def test_image_axes_match_the_real_camera():
    cam = OverheadCamera()
    u0, v0 = cam.project(0.0, -1.0, TOOL_ON_TABLE_Z)
    u1, v1 = cam.project(0.0, -1.1, TOOL_ON_TABLE_Z)   # base -y
    u2, v2 = cam.project(-0.1, -1.0, TOOL_ON_TABLE_Z)  # base -x
    assert u1 > u0 and v1 == pytest.approx(v0)
    assert v2 > v0 and u2 == pytest.approx(u0)
    # About 0.6 mm per pixel, like the real cell.
    assert 0.0005 < 0.1 / (u1 - u0) < 0.0007


@pytest.mark.parametrize('x, y', [(-0.18, -0.75), (0.22, -1.2), (0.02, -0.95), (0.1, -0.8)])
def test_homography_inverts_projection(x, y):
    cam = OverheadCamera()
    u, v = cam.project(x, y, TOOL_ON_TABLE_Z)
    p = cam.homography() @ np.array([u, v, 1.0])
    assert p[:2] / p[2] == pytest.approx((x, y), abs=1e-9)


def test_sim_params_use_the_camera_homography():
    if not os.path.exists(SIM_PARAMS):
        pytest.skip('scrubnurse_bringup sources not next to this package')
    with open(SIM_PARAMS) as f:
        values = yaml.safe_load(f)['tool_localizer']['ros__parameters']['homography']
    np.testing.assert_allclose(np.reshape(values, (3, 3)), OverheadCamera().homography(),
                               atol=1e-9)


@pytest.mark.parametrize('tool_yaw', np.linspace(-math.pi, math.pi, 9))
def test_localizer_grasp_closes_fingers_across_the_tool(tool_yaw):
    # tool_localizer uses tool0 yaw = -theta; the simulated fingers close along tool0 x.
    theta = OverheadCamera().heading(tool_yaw)
    finger_axis = -theta
    assert abs(math.cos(finger_axis - tool_yaw)) < 1e-9


def test_grasp_region():
    assert in_grasp_region((0.0, 0.0, 0.146))
    assert in_grasp_region((0.01, -0.02, 0.13))
    assert not in_grasp_region((0.03, 0.0, 0.146))
    assert in_grasp_region((0.0, 0.0, 0.171))
    assert not in_grasp_region((0.0, 0.0, 0.25))
