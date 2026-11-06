from types import SimpleNamespace

from scrubnurse_dashboard.replay import CYCLE, ROSOUT, Run, VOICE


def status(cycle_id, description, kind='', error=''):
    return SimpleNamespace(cycle_id=cycle_id, kind=kind, tool_id='D', tool_name='scalpel',
                           phase='idle', description=description, step=0, step_count=0,
                           paused=False, aborted=False, error=error, queue=[])


def run():
    voice = SimpleNamespace(kind='heard', text='robot scalpel', wake_word=True, tool_id='D',
                            tool_name='scalpel')
    return Run([
        (102.0, CYCLE, status(0, 'At rest')),
        (103.0, CYCLE, status(0, 'At rest')),
        (104.0, VOICE, voice),
        (105.0, CYCLE, status(1, 'Picking the scalpel', 'handover')),
        (106.0, ROSOUT, SimpleNamespace(level=30, name='scrubnurse_controller', msg='slow plan')),
        (110.0, CYCLE, status(1, 'Handover failed', error='no plan')),
    ], start=100.0, end=120.0, tool_names={'D': 'scalpel'})


def test_state_at_a_point_of_the_run_holds_only_what_came_before():
    at = run().at(105.5)
    assert at['cycle']['description'] == 'Picking the scalpel'
    assert [v['text'] for v in at['voice']] == ['robot scalpel']
    assert [(e['source'], e['text']) for e in at['events']] == [
        ('controller', 'Picking the scalpel'), ('controller', 'At rest')]  # newest first, once
    assert run().at(101.0) == {'cycle': None, 'voice': [], 'events': []}


def test_logs_and_markers():
    replay = run()
    events = replay.at(120.0)['events']
    assert (events[0]['level'], events[0]['text']) == ('error', 'Handover failed')
    assert (events[1]['level'], events[1]['source']) == ('warn', 'scrubnurse_controller')
    assert replay.markers == [{'cycle': 1, 'kind': 'handover', 'tool': 'scalpel', 'start': 5.0,
                               'seconds': 5.0, 'result': 'failed: no plan'}]
