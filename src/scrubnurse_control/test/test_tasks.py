import pytest
from scrubnurse_control.poses import DetectedTool, NamedPose
from scrubnurse_control.tasks import HANDOVER, MoveToJoints, RETURN, TaskError, TaskRunner
from scrubnurse_control.workcell import Workcell

TOOLS = 'ABCDEF'
NAMES = (['H', 'H1', 'HD', 'HW', 'PP', 'P']
         + [f'{prefix}{t}' for t in TOOLS for prefix in ('', 'P', 'R', 'RP', 'RW')])
# One distinct position per pose name, so recorded moves can be mapped back to names.
POSES = {name: NamedPose((float(i), 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
         for i, name in enumerate(NAMES)}
NAME_AT = {pose.position: name for name, pose in POSES.items()}
TOOL = DetectedTool('C', (0.02, -0.9, 0.0), (0.0, 1.0, 0.0, 0.0))


class Recorder:
    """Fake arm + gripper + sleep that records calls; fails the call number `fail_at`."""

    def __init__(self, fail_at=None):
        self.calls = []
        self.fail_at = fail_at

    def move_linear(self, position, orientation):
        position = tuple(position)
        self.calls.append(('move', NAME_AT.get(position, position), tuple(orientation)))
        return len(self.calls) != self.fail_at

    def move(self, position, speed, force):
        self.calls.append(('grip', position, speed, force))
        return len(self.calls) != self.fail_at

    def sleep(self, seconds):
        self.calls.append(('wait', seconds))


def run(steps, tool, detected=None, fail_at=None):
    recorder = Recorder(fail_at)
    runner = TaskRunner(recorder, recorder, POSES, Workcell(), log=lambda _: None,
                        sleep=recorder.sleep)
    runner.run(steps, tool, detected)
    return recorder.calls


def ros1_full_pick_and_place(t):
    """Return the expected calls, written from the ROS 1 path_flow and gripper constants."""
    flow = ['OG', f'P{t}', t, 'CG', f'P{t}', 'PP', 'P', 'EOG', 'PP', 'HD', 'CG']
    special = {'OG': [('grip', 153, 10, 25)], 'CG': [('grip', 255, 10, 25)],
               'EOG': [('wait', 2.0), ('grip', 153, 10, 25)]}
    down = (1.0, 0.0, 0.0, 0.0)
    return [c for p in flow for c in special.get(p, [('move', p, down)])]


def ros1_place_after_detection(t, tool):
    flow = ['OG50', 'TPG', 'TG', 'CG', 'TPG', 'HD', f'RW{t}', f'RP{t}', f'R{t}', 'OG',
            f'RP{t}', f'RW{t}', 'HD', 'CG']
    x, y = tool.position[:2]
    special = {
        'OG': [('grip', 153, 10, 0)], 'OG50': [('grip', 127, 10, 0)],
        'CG': [('grip', 230, 10, 0)],
        'TPG': [('move', (x, y, 0.380), tool.orientation)],
        'TG': [('move', (x, y, Workcell().grasp_height(x, y)), tool.orientation)],
    }
    return [c for p in flow for c in special.get(p, [('move', p, (1.0, 0.0, 0.0, 0.0))])]


@pytest.mark.parametrize('tool', TOOLS)
def test_handover_matches_ros1(tool):
    assert run(HANDOVER, tool) == ros1_full_pick_and_place(tool)


def test_return_matches_ros1():
    assert run(RETURN, TOOL.tool_id, TOOL) == ros1_place_after_detection('C', TOOL)


def test_failed_step_stops_sequence():
    recorder = Recorder(fail_at=3)
    runner = TaskRunner(recorder, recorder, POSES, Workcell(), log=lambda _: None,
                        sleep=recorder.sleep)
    with pytest.raises(TaskError, match='step 3'):
        runner.run(HANDOVER, 'A')
    assert len(recorder.calls) == 3


def test_missing_pose_fails_before_moving():
    poses = {k: v for k, v in POSES.items() if k != 'RPD'}
    recorder = Recorder()
    runner = TaskRunner(recorder, recorder, poses, Workcell(), log=lambda _: None)
    with pytest.raises(TaskError, match='RPD'):
        runner.run(RETURN, 'D', TOOL)
    assert recorder.calls == []


def test_stop_request_ends_sequence_between_steps():
    recorder = Recorder()
    runner = TaskRunner(recorder, recorder, POSES, Workcell(), log=lambda _: None,
                        sleep=recorder.sleep, should_stop=lambda: len(recorder.calls) >= 2)
    with pytest.raises(TaskError, match='stopped before step 3'):
        runner.run(HANDOVER, 'A')
    assert len(recorder.calls) == 2


def test_return_needs_detection():
    with pytest.raises(TaskError, match='detected'):
        run(RETURN, 'C')


def test_joint_move_uses_recorded_joints():
    poses = {'H': NamedPose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0), joints=(1, 2, 3, 4, 5, 6))}
    recorder = Recorder()
    recorder.move_joints = lambda joints: recorder.calls.append(('joints', joints)) or True
    TaskRunner(recorder, recorder, poses, Workcell(), log=lambda _: None).run(
        (MoveToJoints('H'),), tool='')
    assert recorder.calls == [('joints', (1, 2, 3, 4, 5, 6))]


def test_joint_move_needs_recorded_joints():
    poses = {'H': NamedPose((0.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))}
    with pytest.raises(TaskError, match='joints'):
        TaskRunner(Recorder(), Recorder(), poses, Workcell(), log=lambda _: None).run(
            (MoveToJoints('H'),), tool='')
