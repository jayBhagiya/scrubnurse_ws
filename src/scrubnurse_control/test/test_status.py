from dataclasses import replace

import pytest
from scrubnurse_control.status import Status

SCALPEL = Status(cycle_id=3, tool_id='D', tool_name='scalpel', paused=False, step=4, step_count=14)


@pytest.mark.parametrize('kind, phase, text', [
    ('handover', 'picking', 'Picking the scalpel from the magazine'),
    ('handover', 'handing_over', 'Handing over the scalpel'),
    ('handover', 'returning_home', 'Going back to the rest position'),
    ('return', 'picking', 'Picking the scalpel up from the table'),
    ('return', 'placing', 'Placing the scalpel back in the magazine'),
    ('return', 'returning_home', 'Going back to the rest position'),
])
def test_cycle_phases_use_instrument_names_not_pose_codes(kind, phase, text):
    assert replace(SCALPEL, kind=kind, phase=phase).description == text


def test_outside_a_cycle():
    assert Status(paused=False).description == 'At the rest position, waiting for a request'
    assert Status().description == 'Paused: resume to continue'
    assert Status(phase='homing').description == 'Moving to the rest position'
    assert Status(error='move to PD failed').description == 'Stopped: move to PD failed'


def test_abort_wins_over_everything():
    status = replace(SCALPEL, kind='handover', phase='handing_over', aborted=True)
    assert status.description.startswith('Aborted')


def test_tool_id_when_name_unknown():
    assert Status(kind='handover', phase='picking', tool_id='G').description == \
        'Picking the G from the magazine'
