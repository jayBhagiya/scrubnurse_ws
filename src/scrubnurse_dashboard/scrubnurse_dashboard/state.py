"""Dashboard logic without ROS: which controls are allowed, topic freshness, camera overlay."""

import math

import cv2

ACTIONS = ('pause', 'resume', 'abort', 'recover', 'request')


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
    """Detection2D as {tool_id, tool_name, score, x, y, width, height, theta} (pixels, radians)."""
    hypothesis = det.results[0].hypothesis if det.results else None
    tool_id = hypothesis.class_id if hypothesis else '?'
    theta = det.bbox.center.theta
    return {'tool_id': tool_id, 'tool_name': names.get(tool_id, ''),
            'score': round(hypothesis.score, 3) if hypothesis else 0.0,
            'x': det.bbox.center.position.x, 'y': det.bbox.center.position.y,
            'width': det.bbox.size_x, 'height': det.bbox.size_y,
            'theta': None if math.isnan(theta) else theta}


def draw_detections(image, detections):
    """Draw boxes, labels and headings of detection dicts onto a BGR image in place."""
    for d in detections:
        x1, y1 = int(d['x'] - d['width'] / 2), int(d['y'] - d['height'] / 2)
        x2, y2 = int(d['x'] + d['width'] / 2), int(d['y'] + d['height'] / 2)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 3)
        label = f'{d["tool_id"]} {d["tool_name"]} {d["score"]:.2f}'
        cv2.putText(image, label, (x1, max(y1 - 10, 25)), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 200, 0), 2, cv2.LINE_AA)
        if d['theta'] is not None:
            length = max(d['width'], d['height']) / 2
            end = (int(d['x'] + length * math.cos(d['theta'])),
                   int(d['y'] + length * math.sin(d['theta'])))
            cv2.arrowedLine(image, (int(d['x']), int(d['y'])), end, (0, 0, 255), 3)
    return image
