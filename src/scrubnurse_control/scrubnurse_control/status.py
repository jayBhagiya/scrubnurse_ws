"""Controller state as published in CycleStatus.msg, with its wording for people."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Status:
    cycle_id: int = 0
    kind: str = ''          # '', 'handover' or 'return'
    tool_id: str = ''
    tool_name: str = ''
    phase: str = 'idle'
    step: int = 0
    step_count: int = 0
    paused: bool = True
    aborted: bool = False
    error: str = ''
    queue: tuple = ()

    @property
    def description(self):
        return describe(self)


def describe(status):
    """One sentence for the operator, e.g. "Handing over the scalpel"."""
    name = status.tool_name or status.tool_id
    if status.aborted:
        return 'Aborted: make sure the arm is clear, then recover'
    cycle_text = {
        ('handover', 'picking'): f'Picking the {name} from the magazine',
        ('handover', 'handing_over'): f'Handing over the {name}',
        ('return', 'picking'): f'Picking the {name} up from the table',
        ('return', 'placing'): f'Placing the {name} back in the magazine',
    }
    if (status.kind, status.phase) in cycle_text:
        return cycle_text[status.kind, status.phase]
    if status.phase == 'returning_home':
        return 'Going back to the rest position'
    if status.phase == 'homing':
        return 'Moving to the rest position'
    if status.error:
        return f'Stopped: {status.error}'
    if status.paused:
        return 'Paused: resume to continue'
    return 'At the rest position, waiting for a request'
