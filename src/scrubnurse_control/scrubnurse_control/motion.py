"""Straight-line tool0 motions through MoveIt 2 (move_group), using pymoveit2 >= 5.0."""

import math
import time

from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RosDuration

UR_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def slow_down(trajectory, scale):
    """
    Retime a JointTrajectory to `scale` (0..1] of its speed, in place.

    Stretching time by 1/scale scales velocities by scale and accelerations by scale^2.
    """
    for point in trajectory.points:
        t = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) / scale
        sec = int(t)
        point.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        point.velocities = [v * scale for v in point.velocities]
        point.accelerations = [a * scale * scale for a in point.accelerations]
    return trajectory


class Arm:
    """
    Cartesian moves of the UR arm, the MoveIt 2 equivalent of ROS 1 compute_cartesian_path.

    Unlike the ROS 1 node, a path is only executed when at least
    `min_cartesian_fraction` of it could be planned.
    """

    def __init__(self, node, callback_group, group_name='ur_manipulator',
                 base_frame='base', end_effector='tool0', joint_names=UR_JOINTS,
                 velocity_scaling=0.1, acceleration_scaling=0.1,
                 min_cartesian_fraction=0.95, cartesian_max_step=0.005,
                 motion_timeout=10.0, should_stop=lambda: False):
        from pymoveit2 import MoveIt2

        self._moveit2 = MoveIt2(
            node=node, joint_names=joint_names, base_link_name=base_frame,
            end_effector_name=end_effector, group_name=group_name,
            callback_group=callback_group)
        # Joint-space plans get this scaling from MoveIt.
        self._moveit2.max_velocity = velocity_scaling
        self._moveit2.max_acceleration = acceleration_scaling
        # Humble's GetCartesianPath has no scaling fields and returns full-speed paths.
        self._cartesian_scale = min(velocity_scaling, math.sqrt(acceleration_scaling))
        self._should_stop = should_stop
        self._motion_timeout = motion_timeout
        self._clock = node.get_clock()
        self._logger = node.get_logger()
        self._min_fraction = min_cartesian_fraction
        self._max_step = cartesian_max_step

    def move_linear(self, position, orientation):
        """Move tool0 in a straight line to the pose; return True on success."""
        trajectory = self._moveit2.plan(
            position=list(position), quat_xyzw=list(orientation), cartesian=True,
            cartesian_max_step=self._max_step,
            cartesian_fraction_threshold=self._min_fraction, timeout_sec=self._motion_timeout)
        if trajectory is None:
            return False
        return self._execute(slow_down(trajectory, self._cartesian_scale))

    def move_joints(self, joints):
        """Plan and execute a joint-space move; return True on success."""
        trajectory = self._moveit2.plan(joint_positions=list(joints),
                                        timeout_sec=self._motion_timeout)
        return trajectory is not None and self._execute(trajectory)

    def _execute(self, trajectory):
        """
        Execute and wait; cancel the motion (the arm stops) on stop or timeout.

        The UR MoveIt configuration disables MoveIt's execution-duration monitoring, so a
        blocked arm would otherwise wait forever.
        """
        from pymoveit2 import MoveIt2State

        if not self._moveit2.execute(trajectory):
            return False
        end = trajectory.points[-1].time_from_start
        allowed = end.sec + end.nanosec * 1e-9 + self._motion_timeout
        # ROS time, so the limit also holds when Gazebo runs slower than real time.
        deadline = self._clock.now() + RosDuration(seconds=allowed)
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            timed_out = self._clock.now() > deadline
            if timed_out:
                self._logger.error(f'motion did not finish within {allowed:.0f} s')
            if timed_out or self._should_stop():
                self._moveit2.cancel_execution()
                self._moveit2.wait_until_executed(timeout_sec=2.0)
                return False
            time.sleep(0.05)
        # The motion has finished; this returns its recorded outcome.
        return bool(self._moveit2.wait_until_executed())
