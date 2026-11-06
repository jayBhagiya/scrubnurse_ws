"""
Per-cycle metrics of a recorded run (launch with record:=true).

    ros2 run scrubnurse_dashboard metrics ~/scrubnurse_runs/<date_time>

Prints one row per handover or return with its duration and outcome, then a summary per
kind, and writes the rows to cycles.csv in the run folder.
"""

import csv
import os
import statistics
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from scrubnurse_interfaces.msg import CycleStatus

TOPIC = '/scrubnurse_controller/cycle'
COLUMNS = ['cycle', 'kind', 'tool', 'start', 'seconds', 'result']


def read_messages(run, types):
    """Yield (receive time in s, topic, message) of the topics in `types` (topic -> class)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=run, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        yield stamp / 1e9, topic, deserialize_message(data, types[topic])


def read_statuses(run):
    """Yield (receive time in s, CycleStatus) from the run's recording."""
    for t, _, msg in read_messages(run, {TOPIC: CycleStatus}):
        yield t, msg


def cycles(statuses):
    """
    Turn the controller's status messages into one row per cycle.

    A cycle starts when a status carries a new cycle_id with a kind, and ends with the first
    status of that cycle_id without a kind; `error` then says whether it failed. `start` is in
    seconds from the first status.
    """
    rows, t0 = {}, None
    for t, s in statuses:
        t0 = t if t0 is None else t0
        row = rows.get(s.cycle_id)
        if s.kind and row is None:
            row = rows[s.cycle_id] = {'cycle': s.cycle_id, 'kind': s.kind, 'tool': s.tool_name,
                                      'start': round(t - t0, 1), 'seconds': None,
                                      'result': 'unfinished', '_t': t}
        if row is None or row['seconds'] is not None:
            continue
        if s.aborted:
            row['result'] = 'aborted'
        if not s.kind:
            row['seconds'] = round(t - row.pop('_t'), 1)
            if row['result'] != 'aborted':
                row['result'] = f'failed: {s.error}' if s.error else 'ok'
    for row in rows.values():
        row.pop('_t', None)
    return list(rows.values())


def summary(rows):
    """Return one line per kind: count, successes, mean and slowest duration of the successes."""
    lines = []
    for kind in ('handover', 'return'):
        done = [r['seconds'] for r in rows if r['kind'] == kind and r['result'] == 'ok']
        total = sum(r['kind'] == kind for r in rows)
        if total:
            times = (f', mean {statistics.mean(done):.1f} s, slowest {max(done):.1f} s'
                     if done else '')
            lines.append(f'{kind}: {len(done)} of {total} ok{times}')
    return lines


def main(args=None):
    args = sys.argv[1:] if args is None else args
    if len(args) != 1:
        print(__doc__.strip().splitlines()[2].strip())
        return 1
    run = os.path.expanduser(args[0])
    rows = cycles(read_statuses(run))
    print(f'{"cycle":>5}  {"kind":<9} {"tool":<19} {"start s":>8} {"seconds":>8}  result')
    for r in rows:
        seconds = '' if r['seconds'] is None else r['seconds']
        print(f'{r["cycle"]:>5}  {r["kind"]:<9} {r["tool"]:<19} {r["start"]:>8} {seconds:>8}  '
              f'{r["result"]}')
    print(*(summary(rows) or ['no cycles recorded']), sep='\n')
    with open(os.path.join(run, 'cycles.csv'), 'w', newline='') as f:
        writer = csv.DictWriter(f, COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return 0


if __name__ == '__main__':
    sys.exit(main())
