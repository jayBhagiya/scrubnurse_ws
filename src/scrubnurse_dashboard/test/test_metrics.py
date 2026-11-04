from types import SimpleNamespace

from scrubnurse_dashboard.metrics import cycles, summary


def status(cycle_id, kind='', tool='', error='', aborted=False):
    return SimpleNamespace(cycle_id=cycle_id, kind=kind, tool_name=tool, error=error,
                           aborted=aborted)


def test_cycles_measure_each_cycle_until_its_kind_clears():
    rows = cycles([
        (100.0, status(0)),
        (102.0, status(1, 'handover', 'scalpel')),
        (110.0, status(1, 'handover', 'scalpel')),
        (130.5, status(1)),
        (150.0, status(2, 'return', 'scalpel')),
        (180.0, status(2, error='return of tool D failed: no plan')),
        (200.0, status(3, 'handover', 'needle holder', aborted=True)),
        (201.0, status(3, error='handover of tool B failed: aborted', aborted=True)),
        (300.0, status(4, 'return', 'needle holder')),
    ])
    assert rows == [
        {'cycle': 1, 'kind': 'handover', 'tool': 'scalpel', 'start': 2.0, 'seconds': 28.5,
         'result': 'ok'},
        {'cycle': 2, 'kind': 'return', 'tool': 'scalpel', 'start': 50.0, 'seconds': 30.0,
         'result': 'failed: return of tool D failed: no plan'},
        {'cycle': 3, 'kind': 'handover', 'tool': 'needle holder', 'start': 100.0,
         'seconds': 1.0, 'result': 'aborted'},
        {'cycle': 4, 'kind': 'return', 'tool': 'needle holder', 'start': 200.0,
         'seconds': None, 'result': 'unfinished'},
    ]


def test_summary_counts_only_successful_cycles_in_the_times():
    rows = [{'kind': 'handover', 'seconds': 20.0, 'result': 'ok'},
            {'kind': 'handover', 'seconds': 30.0, 'result': 'ok'},
            {'kind': 'handover', 'seconds': 5.0, 'result': 'aborted'},
            {'kind': 'return', 'seconds': None, 'result': 'unfinished'}]
    assert summary(rows) == ['handover: 2 of 3 ok, mean 25.0 s, slowest 30.0 s',
                             'return: 0 of 1 ok']
