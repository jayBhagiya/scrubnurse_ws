"""Straight-line tool0 motions through MoveIt 2 (move_group), using pymoveit2."""

import time

from pymoveit2 import MoveIt2, MoveIt2State

UR_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


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
                 motion_timeout=60.0, should_stop=lambda: False):
        self._moveit2 = MoveIt2(
            node=node, joint_names=joint_names, base_link_name=base_frame,
            end_effector_name=end_effector, group_name=group_name,
            callback_group=callback_group)
        self._moveit2.max_velocity = velocity_scaling
        self._moveit2.max_acceleration = acceleration_scaling
        # pymoveit2 applies the scaling only to joint-space plans; its Cartesian
        # requests keep 0.0, which MoveIt treats as full speed. Set them here and
        # refuse to run if that private request ever disappears.
        cartesian_request = getattr(self._moveit2, '_MoveIt2__cartesian_path_request', None)
        if cartesian_request is None:
            raise RuntimeError('cannot limit Cartesian speed: pymoveit2 internals changed')
        cartesian_request.max_velocity_scaling_factor = float(velocity_scaling)
        cartesian_request.max_acceleration_scaling_factor = float(acceleration_scaling)
        self._should_stop = should_stop
        self._motion_timeout = motion_timeout
        self._logger = node.get_logger()
        self._min_fraction = min_cartesian_fraction
        self._max_step = cartesian_max_step

    def move_linear(self, position, orientation):
        """Move tool0 in a straight line to the pose; return True on success."""
        if not self._moveit2.move_to_pose(
                position=list(position), quat_xyzw=list(orientation), cartesian=True,
                cartesian_max_step=self._max_step,
                cartesian_fraction_threshold=self._min_fraction):
            return False
        return self._wait()

    def move_joints(self, joints):
        """Plan and execute a joint-space move; return True on success."""
        if not self._moveit2.move_to_configuration(list(joints)):
            return False
        return self._wait()

    def _wait(self):
        """
        Wait for the running motion; cancel it (the arm stops) on stop or timeout.

        The UR MoveIt configuration disables MoveIt's execution-duration monitoring, so a
        blocked arm would otherwise wait forever.
        """
        deadline = time.monotonic() + self._motion_timeout
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            timed_out = time.monotonic() > deadline
            if timed_out:
                self._logger.error(f'motion did not finish within {self._motion_timeout:.0f} s')
            if timed_out or self._should_stop():
                self._moveit2.cancel_execution()
                self._moveit2.wait_until_executed(timeout_sec=2.0)
                return False
            time.sleep(0.05)
        # The motion has finished; this returns its recorded outcome.
        return bool(self._moveit2.wait_until_executed())
