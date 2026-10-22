"""
Pick-and-place sequences of the Scrubnurse, ported from the ROS 1 controller.

A sequence is a tuple of steps. Pose names may contain `{tool}`, replaced by the
tool id (A..F); see poses.yaml for the naming scheme. Gripper positions use the
Robotiq scale: 0 is fully open, 255 fully closed.
"""

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class MoveTo:
    """Straight-line move of tool0 to a named pose."""

    pose: str


@dataclass(frozen=True)
class MoveToJoints:
    """Joint-space move to the joint angles recorded with a named pose."""

    pose: str


@dataclass(frozen=True)
class AboveTool:
    """Straight-line move to the detected tool's grasp pose at the approach height."""


@dataclass(frozen=True)
class AtTool:
    """Straight-line move down to the detected tool at the calibrated grasp height."""


@dataclass(frozen=True)
class Grip:
    """Move the gripper to `position` with the given speed and force (all 0..255)."""

    position: int
    speed: int = 10
    force: int = 25


@dataclass(frozen=True)
class Wait:
    seconds: float


# Magazine -> handover position (ROS 1 `full_pick_and_place_operation`).
HANDOVER = (
    Grip(153), MoveTo('P{tool}'), MoveTo('{tool}'), Grip(255), MoveTo('P{tool}'),
    MoveTo('PP'), MoveTo('P'), Wait(2.0), Grip(153), MoveTo('PP'), MoveTo('HD'), Grip(255),
)

# Return area on the table -> magazine (ROS 1 `place_after_detection`).
RETURN = (
    Grip(127, force=0), AboveTool(), AtTool(), Grip(230, force=0), AboveTool(),
    MoveTo('HD'), MoveTo('RW{tool}'), MoveTo('RP{tool}'), MoveTo('R{tool}'), Grip(153, force=0),
    MoveTo('RP{tool}'), MoveTo('RW{tool}'), MoveTo('HD'), Grip(230, force=0),
)


class TaskError(RuntimeError):
    pass


def required_poses(steps, tool):
    return [s.pose.format(tool=tool) for s in steps if isinstance(s, (MoveTo, MoveToJoints))]


def describe(step, tool):
    if isinstance(step, MoveTo):
        return f'move to {step.pose.format(tool=tool)}'
    if isinstance(step, MoveToJoints):
        return f'joint move to {step.pose.format(tool=tool)}'
    if isinstance(step, AboveTool):
        return f'move above tool {tool}'
    if isinstance(step, AtTool):
        return f'move down to tool {tool}'
    if isinstance(step, Grip):
        return f'gripper to {step.position}'
    return f'wait {step.seconds:.1f} s'


class TaskRunner:
    """
    Execute step sequences with an arm and a gripper.

    `arm.move_linear(position, orientation)`, `arm.move_joints(joints)` and
    `gripper.move(position, speed, force)` must block until done and return
    True on success. `poses` maps names to NamedPose; `workcell` gives the
    heights for detected-tool steps.
    """

    def __init__(self, arm, gripper, poses, workcell, log=print, sleep=time.sleep,
                 should_stop=lambda: False):
        self._arm = arm
        self._gripper = gripper
        self._poses = poses
        self._workcell = workcell
        self._log = log
        self._sleep = sleep
        self._should_stop = should_stop

    def check(self, steps, tool, detected=None):
        """Raise TaskError if the sequence cannot run (missing poses or detection)."""
        missing = [p for p in required_poses(steps, tool) if p not in self._poses]
        if missing:
            raise TaskError(f'unknown poses for tool {tool}: {", ".join(missing)}')
        joint_poses = [s.pose.format(tool=tool) for s in steps if isinstance(s, MoveToJoints)]
        no_joints = [p for p in joint_poses if not self._poses[p].joints]
        if no_joints:
            raise TaskError(f'poses without recorded joints: {", ".join(no_joints)}')
        if detected is None and any(isinstance(s, (AboveTool, AtTool)) for s in steps):
            raise TaskError(f'sequence for tool {tool} needs a detected tool pose')

    def run(self, steps, tool, detected=None):
        """Run all steps in order; raise TaskError at the first failing step."""
        self.check(steps, tool, detected)
        for i, step in enumerate(steps, 1):
            if self._should_stop():
                raise TaskError(f'stopped before step {i}')
            self._log(f'[{tool}] step {i}/{len(steps)}: {describe(step, tool)}')
            if not self._execute(step, tool, detected):
                raise TaskError(f'step {i} ({describe(step, tool)}) failed')

    def _execute(self, step, tool, detected):
        if isinstance(step, MoveTo):
            pose = self._poses[step.pose.format(tool=tool)]
            return self._arm.move_linear(pose.position, pose.orientation)
        if isinstance(step, MoveToJoints):
            return self._arm.move_joints(self._poses[step.pose.format(tool=tool)].joints)
        if isinstance(step, (AboveTool, AtTool)):
            x, y = detected.position[0], detected.position[1]
            z = (self._workcell.approach_height if isinstance(step, AboveTool)
                 else self._workcell.grasp_height(x, y))
            return self._arm.move_linear((x, y, z), detected.orientation)
        if isinstance(step, Grip):
            return self._gripper.move(step.position, step.speed, step.force)
        self._sleep(step.seconds)
        return True
