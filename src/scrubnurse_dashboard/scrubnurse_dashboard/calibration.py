"""
Camera-to-table calibration for the dashboard's calibration page.

Each pair is the pixel centre of a dark calibration block (40 x 40 x 10 mm, see
docs/calibration) and the tool0 x/y in the UR `base` frame once a pointer held in
the gripper touches the block's centre. Four or more pairs give the homography of
the tool_localizer node.
"""

import os
import threading
import time

import cv2
import numpy as np
from scrubnurse_perception import homography
from scrubnurse_perception.compute_homography import load_pairs, to_params_yaml


def find_block(image, side, roi=None):
    """
    Return the centre (u, v) and the 4 corners of the one dark square in a BGR image.

    `side` is the expected side length in pixels (within -30 % / +40 %), `roi` an
    optional polygon [[u, v], ...] to search in. Raise ValueError when there is no
    such square or more than one.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    area = np.full_like(gray, 255)
    if roi is not None and len(roi):
        area[:] = 0
        cv2.fillPoly(area, [np.asarray(roi, dtype=np.int32).reshape(-1, 2)], 255)
    # Otsu inside the search area splits the dark block from the light table.
    threshold, _ = cv2.threshold(gray[area > 0].reshape(-1, 1), 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    dark = ((gray <= threshold) & (area > 0)).astype(np.uint8) * 255  # Otsu: dark is <= t
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blocks = []
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        w, h = rect[1]
        right_size = 0.7 * side <= min(w, h) and max(w, h) <= 1.4 * side
        if right_size and max(w, h) < 1.25 * min(w, h) and cv2.contourArea(contour) > 0.8 * w * h:
            blocks.append((rect[0], cv2.boxPoints(rect)))
    if len(blocks) != 1:
        raise ValueError('no dark block of about {:.0f} px found'.format(side) if not blocks else
                         f'{len(blocks)} dark blocks found: leave only the calibration block')
    (u, v), corners = blocks[0]
    return (float(u), float(v)), corners.tolist()


class Calibration:
    """Recorded pairs, the pixel waiting for its robot position, and the fit."""

    def __init__(self, folder):
        self.folder = folder
        self._lock = threading.Lock()
        self._pairs = []      # [u, v, x, y]
        self._pending = None  # {'u', 'v', 'frames', 'spread'}

    def set_pixel(self, u, v, frames=1, spread=0.0):
        with self._lock:
            self._pending = {'u': u, 'v': v, 'frames': frames, 'spread': spread}

    def clear_pixel(self):
        with self._lock:
            self._pending = None

    def add_reference(self, x, y):
        """Pair the pending pixel with the robot position (m)."""
        with self._lock:
            if self._pending is None:
                raise ValueError('select the pixel position first')
            self._pairs.append([self._pending['u'], self._pending['v'], x, y])
            self._pending = None

    def remove(self, index):
        with self._lock:
            if not 0 <= index < len(self._pairs):
                raise ValueError(f'no pair {index + 1}')
            del self._pairs[index]

    def fit(self):
        """Homography and per-pair errors (m); None with fewer than four pairs."""
        with self._lock:
            pairs = np.array(self._pairs, dtype=float).reshape(-1, 4)
        if len(pairs) < 4:
            return None
        pixels, points = pairs[:, :2], pairs[:, 2:]
        h = homography.estimate(pixels, points)
        unseen = homography.leave_one_out_errors(pixels, points) if len(pairs) > 4 else None
        return h, homography.reprojection_errors(h, pixels, points), unseen

    def state(self):
        try:
            fit, fit_error = self.fit(), None
        except ValueError as e:  # e.g. all pairs at one robot position
            fit, fit_error = None, str(e)
        with self._lock:
            pairs = [dict(zip('uvxy', p)) for p in self._pairs]
            pending = self._pending
        result = None
        if fit is not None:
            h, errors, unseen = fit
            for pair, error in zip(pairs, errors):
                pair['error'] = float(error)
            if unseen is not None:
                for pair, error in zip(pairs, unseen):
                    pair['unseen_error'] = float(error)
            result = {'homography': h.ravel().tolist(), 'yaml': to_params_yaml(h),
                      'mean': float(errors.mean()), 'max': float(errors.max()),
                      'unseen_mean': None if unseen is None else float(unseen.mean()),
                      'unseen_max': None if unseen is None else float(unseen.max())}
        return {'pairs': pairs, 'pending': pending, 'fit': result, 'fit_error': fit_error,
                'files': self.files()}

    def save(self):
        """Write the pairs (CSV) and the tool_localizer parameters (YAML); return both paths."""
        fit = self.fit()
        if fit is None:
            raise ValueError('need at least 4 pairs')
        os.makedirs(self.folder, exist_ok=True)
        stamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        pairs_path = os.path.join(self.folder, f'pairs_{stamp}.csv')
        params_path = os.path.join(self.folder, f'homography_{stamp}.yaml')
        with self._lock:
            rows = [','.join(repr(float(v)) for v in p) for p in self._pairs]
        with open(pairs_path, 'w') as f:
            f.write('u,v,x,y\n' + '\n'.join(rows) + '\n')
        with open(params_path, 'w') as f:
            f.write(to_params_yaml(fit[0]))
        return pairs_path, params_path

    def files(self):
        if not os.path.isdir(self.folder):
            return []
        return sorted((f for f in os.listdir(self.folder) if f.endswith('.csv')), reverse=True)

    def load(self, name):
        """Replace the pairs with those of a saved CSV in the calibration folder."""
        if name not in self.files():  # only files listed in the folder, no paths
            raise ValueError(f'no saved pairs "{name}"')
        pixels, points = load_pairs(os.path.join(self.folder, name))
        with self._lock:
            self._pairs = np.hstack([pixels, points]).tolist()
            self._pending = None
