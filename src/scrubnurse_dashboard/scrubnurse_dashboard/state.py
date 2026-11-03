"""Dashboard logic without ROS: which controls are allowed, topic freshness, camera overlay."""

import math

import cv2

ACTIONS = ('pause', 'resume', 'abort', 'recover', 'request')

# One colour per tool, readable on the light table in the small stream (RGB hex).
TOOL_COLORS = {'A': '#e6194b', 'B': '#3cb44b', 'C': '#4363d8', 'D': '#f032e6', 'E': '#42d4f4',
               'F': '#ffffff'}
OTHER_COLOR = '#a0a0a0'


def bgr(color):
    """Convert a '#rrggbb' colour to the BGR tuple OpenCV draws with."""
    return tuple(int(color[i:i + 2], 16) for i in (5, 3, 1))


def refusal(action, cycle, controller_up):
    """Why `action` is not allowed now, or None; `cycle` is the last CycleStatus as a dict."""
    if not controller_up:
        return 'the controller is not running'
    if cycle is None:
        return 'no status from the controller yet'
    if action == 'recover':
        return None if cycle['aborted'] else 'only needed after an abort'
    if cycle['aborted']:
        return {'abort': 'already aborted'}.get(
            action, 'aborted: make sure the arm is clear, then recover')
    if action == 'pause' and cycle['paused']:
        return 'already paused'
    if action == 'resume' and not cycle['paused']:
        return 'not paused'
    if action == 'request' and cycle['error']:
        return f'stopped after an error ({cycle["error"]}): resume first'
    return None


def freshness(received, now, stale_after):
    """Age of a streaming topic and whether it is stale; `received` is None before any."""
    if received is None:
        return {'age': None, 'stale': True}
    age = now - received
    return {'age': round(age, 2), 'stale': age > stale_after}


def detection_dict(det, names):
    """Detection2D as {tool_id, tool_name, color, score, x, y, width, height, theta}."""
    hypothesis = det.results[0].hypothesis if det.results else None
    tool_id = hypothesis.class_id if hypothesis else '?'
    theta = det.bbox.center.theta
    return {'tool_id': tool_id, 'tool_name': names.get(tool_id, ''),
            'color': TOOL_COLORS.get(tool_id, OTHER_COLOR),
            'score': round(hypothesis.score, 3) if hypothesis else 0.0,
            'x': det.bbox.center.position.x, 'y': det.bbox.center.position.y,
            'width': det.bbox.size_x, 'height': det.bbox.size_y,
            'theta': None if math.isnan(theta) else theta}


def draw_detections(image, detections):
    """Draw boxes, labels and headings of detection dicts onto a BGR image in place."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    for d in detections:
        color = bgr(d.get('color', OTHER_COLOR))
        x1, y1 = int(d['x'] - d['width'] / 2), int(d['y'] - d['height'] / 2)
        x2, y2 = int(d['x'] + d['width'] / 2), int(d['y'] + d['height'] / 2)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 0), 7)  # dark outline: visible on white
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 4)
        if d['theta'] is not None:
            length = max(d['width'], d['height']) / 2
            end = (int(d['x'] + length * math.cos(d['theta'])),
                   int(d['y'] + length * math.sin(d['theta'])))
            cv2.arrowedLine(image, (int(d['x']), int(d['y'])), end, (0, 0, 0), 8, tipLength=0.25)
            cv2.arrowedLine(image, (int(d['x']), int(d['y'])), end, color, 4, tipLength=0.25)
        # Label on a filled tag in the tool colour, black text.
        label = f'{d["tool_id"]} {d["tool_name"]}'  # confidence: in the detection table
        (w, h), _ = cv2.getTextSize(label, font, 1.0, 2)
        top = max(y1 - h - 14, 0)
        cv2.rectangle(image, (x1, top), (x1 + w + 10, top + h + 12), color, -1)
        cv2.putText(image, label, (x1 + 5, top + h + 5), font, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
    return image
