#!/usr/bin/env python3
"""
End-to-end test of the simulated cell through the dashboard API.

Start the simulation first (docker compose up -d), then run:
    python3 scripts/sim_e2e_test.py [dashboard URL, default http://localhost:8080]

Resumes the controller and requests the scalpel (tool D). Passes when the scalpel was handed
over, laid back on the table by the simulated surgeon and returned to the magazine, with no
error or abort on the way.
"""

import json
import sys
import time
import urllib.error
import urllib.request

URL = sys.argv[1] if len(sys.argv) > 1 else 'http://localhost:8080'
TOOL_ID, TOOL_NAME = 'D', 'scalpel'


def call(path, body=None):
    """GET (or POST with a JSON body) a dashboard API path; None while it is not reachable."""
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(URL + path, data=data,
                                     headers={'content-type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as reply:
            return json.load(reply)
    except urllib.error.HTTPError as e:
        return json.load(e)
    except (urllib.error.URLError, ConnectionError, TimeoutError):
        return None


def wait_for(what, timeout, check):
    """Poll the state until check(cycle) is true; exit with the last state on timeout."""
    deadline = time.monotonic() + timeout
    cycle = None
    while time.monotonic() < deadline:
        state = call('/api/state')
        cycle = (state or {}).get('cycle')
        if cycle and (cycle['error'] or cycle['aborted']):
            sys.exit(f'FAIL while waiting for {what}: {cycle}')
        if cycle and check(cycle):
            print(f'ok: {what}')
            return cycle
        time.sleep(1)
    sys.exit(f'FAIL: no {what} within {timeout} s; last cycle: {cycle}')


def main():
    wait_for('controller', 300, lambda c: True)
    print(call('/api/resume', {}))
    wait_for('resumed', 30, lambda c: not c['paused'])
    wait_for('rest position', 120, lambda c: c['phase'] == 'idle')
    print(call('/api/request', {'instrument': TOOL_NAME}))
    handover = wait_for('handover', 60,
                        lambda c: c['kind'] == 'handover' and c['tool_id'] == TOOL_ID)
    wait_for('return', 180, lambda c: c['kind'] == 'return' and c['tool_id'] == TOOL_ID
             and c['cycle_id'] > handover['cycle_id'])
    wait_for('tool back in the magazine', 120, lambda c: c['phase'] == 'idle')
    print('PASS')


if __name__ == '__main__':
    main()
