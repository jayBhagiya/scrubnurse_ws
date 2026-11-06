"""
A recorded run as the dashboard shows it at a point of the recording (mode replay).

The bag player (replay.launch.py) plays the streams: camera, detections, arm, voice status.
The cycle status, voice events and logs are read here from the bag instead, so that after a
jump back the page shows the state of that moment, not the last message played.
"""

import bisect

from rcl_interfaces.msg import Log
import rosbag2_py
from scrubnurse_dashboard import metrics, state
from scrubnurse_interfaces.msg import CycleStatus, VoiceEvent

CYCLE, VOICE, ROSOUT = metrics.TOPIC, '/voice_command/events', '/rosout'
EVENTS_SHOWN = 200  # as the live event log
VOICE_SHOWN = 10


def log_level(level):
    """rcl_interfaces/Log level as an event log level."""
    return 'error' if level >= 40 else 'warn' if level >= 30 else 'info'


class Run:
    """
    The run's cycle statuses, voice events and log lines by receive time.

    `messages` are (receive time in s, topic, message) in time order, as from
    metrics.read_messages; `start` and `end` are the recording's (s).
    """

    def __init__(self, messages, start, end, tool_names):
        self.start, self.end = start, end
        self.cycles, self.voice, self.events, statuses = [], [], [], []
        for t, topic, msg in messages:
            if topic == CYCLE:
                cycle = state.cycle_dict(msg, tool_names)
                statuses.append((t, msg))
                if not self.cycles or self.cycles[-1][1]['description'] != msg.description:
                    self.events.append((t, {'time': t, 'level': state.cycle_level(cycle),
                                            'source': 'controller', 'text': msg.description}))
                self.cycles.append((t, cycle))
            elif topic == VOICE:
                self.voice.append((t, state.voice_dict(msg, t)))
            elif topic == ROSOUT:
                self.events.append((t, {'time': t, 'level': log_level(msg.level),
                                        'source': msg.name, 'text': msg.msg}))
        # Cycles start from the run's start, not from the first status as in metrics.
        offset = round(statuses[0][0] - start, 1) if statuses else 0.0
        self.markers = [dict(row, start=round(row['start'] + offset, 1))
                        for row in metrics.cycles(statuses)]

    @classmethod
    def read(cls, path, tool_names):
        info = rosbag2_py.Info().read_metadata(path, 'mcap')
        start = info.starting_time.nanoseconds / 1e9
        messages = metrics.read_messages(
            path, {CYCLE: CycleStatus, VOICE: VoiceEvent, ROSOUT: Log})
        return cls(messages, start, start + info.duration.nanoseconds / 1e9, tool_names)

    def at(self, t):
        """Cycle status, voice events and event log (newest first) up to time `t` (s)."""
        def until(items):
            return [item for _, item in items[:bisect.bisect_right(items, t, key=lambda i: i[0])]]
        cycles = until(self.cycles)
        return {'cycle': cycles[-1] if cycles else None,
                'voice': until(self.voice)[-VOICE_SHOWN:][::-1],
                'events': until(self.events)[-EVENTS_SHOWN:][::-1]}
