import math
import os

import numpy as np
from scrubnurse_perception import homography
from scrubnurse_perception.compute_homography import load_pairs, main, to_params_yaml
import yaml

DATA = os.path.join(os.path.dirname(__file__), 'data', 'calibration_pairs_2024-11-22.csv')

# Matrix the ROS 1 cell used in production (od_homography/config/tf-scrubnurse-22-11-2024.json).
REAL_CELL_H = [
    [1.0163595831671571e-5, -0.0006197696797893486, 0.3398135029914868],
    [-0.0006014182940706726, 4.1451986390270245e-5, -0.4024938848671156],
    [-1.577423913888336e-5, -5.1868361319957846e-5, 1.0],
]


def test_real_cell_matrix_reproduces_calibration_pairs():
    pixels, points = load_pairs(DATA)
    errors = homography.reprojection_errors(homography.as_matrix(REAL_CELL_H), pixels, points)
    assert errors.mean() < 0.002
    assert errors.max() < 0.0035


def test_estimate_is_as_good_as_the_real_cell_matrix():
    pixels, points = load_pairs(DATA)
    h = homography.estimate(pixels, points)
    fitted = homography.reprojection_errors(h, pixels, points)
    stored = homography.reprojection_errors(homography.as_matrix(REAL_CELL_H), pixels, points)
    assert fitted.mean() <= stored.mean() + 1e-4


def test_estimate_recovers_exact_homography():
    h_true = homography.as_matrix([[0.002, 0.0001, -1.0], [0.00005, -0.0021, 0.5],
                                   [1e-6, 2e-6, 1.0]])
    pixels = np.array([[u, v] for u in (100, 700, 1300, 1800) for v in (100, 500, 1000)], float)
    points = np.array([homography.pixel_to_plane(h_true, u, v) for u, v in pixels])
    np.testing.assert_allclose(homography.estimate(pixels, points), h_true, rtol=1e-6, atol=1e-9)


def test_estimate_rejects_too_few_points():
    try:
        homography.estimate([[0, 0], [1, 0], [0, 1]], [[0, 0], [1, 0], [0, 1]])
    except ValueError:
        return
    raise AssertionError('expected ValueError')


def test_quaternion_from_euler_matches_known_values():
    s = math.sqrt(0.5)
    np.testing.assert_allclose(homography.quaternion_from_euler(math.pi / 2, 0, 0), (s, 0, 0, s),
                               atol=1e-12)
    np.testing.assert_allclose(homography.quaternion_from_euler(0, 0, math.pi / 2), (0, 0, s, s),
                               atol=1e-12)


def _rotate(q, v):
    x, y, z, w = q
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return r @ np.asarray(v, float)


def test_grasp_orientation_points_tool_down_with_negated_yaw():
    for theta in np.linspace(-math.pi, math.pi, 13):
        q = homography.grasp_orientation(theta)
        np.testing.assert_allclose(_rotate(q, [0, 0, 1]), [0, 0, -1], atol=1e-9)
        x_axis = _rotate(q, [1, 0, 0])
        np.testing.assert_allclose(math.atan2(x_axis[1], x_axis[0]),
                                   math.atan2(math.sin(-theta), math.cos(-theta)), atol=1e-9)


def test_cli_writes_loadable_params(tmp_path):
    out = tmp_path / 'h.yaml'
    main([DATA, '-o', str(out)])
    params = yaml.safe_load(out.read_text())['tool_localizer']['ros__parameters']
    assert len(params['homography']) == 9
    assert to_params_yaml(np.eye(3)).count(',') == 8
