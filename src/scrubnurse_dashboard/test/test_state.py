import math

import numpy as np
import pytest
from scrubnurse_dashboard.state import bgr, draw_detections, freshness, refusal, TOOL_COLORS

IDLE = {'paused': False, 'aborted': False, 'error': ''}
PAUSED = dict(IDLE, paused=True)
ABORTED = dict(IDLE, paused=True, aborted=True)
FAILED = dict(IDLE, paused=True, error='Grip of tool B failed')


@pytest.mark.parametrize('action, cycle, allowed', [
    ('pause', IDLE, True),
    ('pause', PAUSED, False),
    ('resume', PAUSED, True),
    ('resume', IDLE, False),
    ('resume', ABORTED, False),
    ('resume', FAILED, True),
    ('abort', IDLE, True),
    ('abort', ABORTED, False),
    ('recover', ABORTED, True),
    ('recover', IDLE, False),
    ('request', IDLE, True),
    ('request', PAUSED, True),  # queued until resumed
    ('request', ABORTED, False),
    ('request', FAILED, False),
])
def test_refusal(action, cycle, allowed):
    assert (refusal(action, cycle, True) is None) == allowed


def test_everything_refused_without_controller():
    assert refusal('abort', IDLE, False) == 'the controller is not running'
    assert refusal('abort', None, True) == 'no status from the controller yet'


def test_freshness():
    assert freshness(None, 10.0, 2.0) == {'age': None, 'stale': True}
    assert freshness(9.0, 10.0, 2.0) == {'age': 1.0, 'stale': False}
    assert freshness(7.0, 10.0, 2.0)['stale']


def test_draw_detections():
    image = np.zeros((200, 300, 3), np.uint8)
    d = {'tool_id': 'D', 'tool_name': 'scalpel', 'score': 0.9, 'x': 150, 'y': 100,
         'width': 80, 'height': 40, 'theta': math.pi / 2}
    draw_detections(image, [d, dict(d, theta=None)])
    assert image[60, 150].any()  # top edge of the box


def test_tool_colors_are_distinct_and_drawn():
    assert len(set(TOOL_COLORS.values())) == 6
    assert bgr('#e6194b') == (0x4b, 0x19, 0xe6)
    image = np.full((200, 300, 3), 200, np.uint8)
    d = {'tool_id': 'C', 'tool_name': 'mosquito', 'color': TOOL_COLORS['C'], 'score': 0.9,
         'x': 150, 'y': 120, 'width': 100, 'height': 40, 'theta': None}
    draw_detections(image, [d])
    assert tuple(image[140, 150]) == bgr(TOOL_COLORS['C'])  # bottom edge of the box
