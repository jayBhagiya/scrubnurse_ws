from builtin_interfaces.msg import Duration
from unittest.mock import MagicMock

import pytest
from rclpy.time import Time
from scrubnurse_control.motion import Arm, slow_down
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


class FakeMoveIt2:

    def __init__(self, busy_polls):
        self.busy_polls, self.cancelled, self.reset = busy_polls, False, False

    def execute(self, trajectory):
        return True

    def query_state(self):
        from pymoveit2 import MoveIt2State
        self.busy_polls -= 1
        return MoveIt2State.EXECUTING if self.busy_polls >= 0 else MoveIt2State.IDLE

    def cancel_execution(self):
        self.cancelled = True

    def wait_until_executed(self, timeout_sec=None):
        return not self.cancelled

    def force_reset_executing_state(self):
        self.reset = True


class SecondPerCallClock:

    def __init__(self):
        self.t = 0

    def now(self):
        self.t += 1
        return Time(seconds=self.t)


def arm_with(speed_scaling, busy_polls):
    """Build an Arm whose execution stays busy for `busy_polls` polls, one second apart."""
    arm = Arm.__new__(Arm)
    arm._moveit2 = FakeMoveIt2(busy_polls)
    arm._clock = SecondPerCallClock()
    arm._logger = MagicMock()
    arm._should_stop = lambda: False
    arm._motion_timeout = 1.0
    arm._speed_scaling = speed_scaling
    return arm


def one_second_trajectory():
    return JointTrajectory(points=[JointTrajectoryPoint(time_from_start=Duration(sec=1))])


def test_slowed_robot_does_not_time_out():
    # 10 s of wall time at a 15 % slider is 1.5 s of trajectory time, within 1 s + 1 s.
    assert Arm._execute(arm_with(0.15, busy_polls=10), one_second_trajectory())


def test_stuck_robot_at_full_speed_times_out():
    arm = arm_with(1.0, busy_polls=10)
    assert not Arm._execute(arm, one_second_trajectory())
    assert arm._moveit2.cancelled


def test_stop_resets_a_goal_move_group_never_finishes():
    # Abort: the motion is cancelled, but the fake stays EXECUTING like move_group does.
    arm = arm_with(1.0, busy_polls=100)
    arm._should_stop = lambda: True
    assert not Arm._execute(arm, one_second_trajectory())
    assert arm._moveit2.cancelled and arm._moveit2.reset


def test_stop_after_finished_cancel_needs_no_reset():
    arm = arm_with(1.0, busy_polls=1)
    arm._should_stop = lambda: True
    assert not Arm._execute(arm, one_second_trajectory())
    assert arm._moveit2.cancelled and not arm._moveit2.reset
