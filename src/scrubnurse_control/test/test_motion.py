from builtin_interfaces.msg import Duration
import pytest
from scrubnurse_control.motion import slow_down
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_slow_down_stretches_time_and_scales_velocity_and_acceleration():
    trajectory = JointTrajectory(points=[
        JointTrajectoryPoint(velocities=[0.0], accelerations=[0.0],
                             time_from_start=Duration(sec=0)),
        JointTrajectoryPoint(velocities=[1.0], accelerations=[2.0],
                             time_from_start=Duration(sec=1, nanosec=250_000_000)),
    ])
    slow_down(trajectory, 0.1)
    end = trajectory.points[-1]
    assert end.time_from_start.sec + end.time_from_start.nanosec * 1e-9 == pytest.approx(12.5)
    assert end.velocities == pytest.approx([0.1])
    assert end.accelerations == pytest.approx([0.02])
    assert trajectory.points[0].time_from_start == Duration(sec=0)
