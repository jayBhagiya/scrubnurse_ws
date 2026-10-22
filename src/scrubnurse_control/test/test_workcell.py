import itertools

import pytest
from scrubnurse_control.poses import DetectedTool
from scrubnurse_control.workcell import select_return_candidate, Workcell


def ros1_z_mapping(x, y):
    """Verbatim logic of the ROS 1 controller's z_mapping()."""
    def lerp(x, x0, x1, y0, y1):
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    value_y_0 = lerp(x, -0.18, 0.22, 0.24615, 0.24691)
    value_y_1 = lerp(x, -0.18, 0.22, 0.24810, 0.24900)
    return lerp(y, -0.75, -1.2, value_y_0, value_y_1)


@pytest.mark.parametrize('x, y', itertools.product((-0.18, -0.05, 0.1, 0.22),
                                                   (-0.75, -0.9, -1.05, -1.2)))
def test_grasp_height_matches_ros1(x, y):
    assert Workcell().grasp_height(x, y) == pytest.approx(ros1_z_mapping(x, y), abs=1e-12)


def test_return_area_bounds():
    cell = Workcell()
    assert cell.contains(-0.18, -1.20) and cell.contains(0.22, -0.75)
    assert not cell.contains(0.23, -0.9) and not cell.contains(0.0, -0.74)


def tool(tool_id, x, y):
    return DetectedTool(tool_id, (x, y, 0.0), (0.0, 1.0, 0.0, 0.0))


def test_candidate_skips_tools_outside_area_and_orders_by_x_then_y():
    tools = [tool('A', -0.5, -0.9), tool('B', 0.1, -1.0), tool('C', 0.0, -0.8),
             tool('D', 0.0, -1.1)]
    assert select_return_candidate(tools, Workcell()).tool_id == 'D'


def test_no_candidate():
    assert select_return_candidate([], Workcell()) is None
    assert select_return_candidate([tool('A', 1.0, 1.0)], Workcell()) is None
