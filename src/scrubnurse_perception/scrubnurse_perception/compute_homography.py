"""
Compute the camera-to-table homography from recorded pixel/robot point pairs.

Input CSV columns: u, v, x, y -- pixel centre of the calibration target as seen
by the detector, and the matching tool0 x/y in the UR `base` frame (metres). A header row
is optional. Writes a parameter file for the tool_localizer node.

    ros2 run scrubnurse_perception compute_homography pairs.csv -o homography.yaml
"""

import argparse
import sys

import numpy as np
from scrubnurse_perception import homography


def load_pairs(path):
    """Return (pixels, points) arrays from a u,v,x,y CSV file."""
    with open(path) as f:
        first = f.readline()
    try:
        [float(v) for v in first.split(',')]
        skip = 0
    except ValueError:
        skip = 1
    data = np.loadtxt(path, delimiter=',', skiprows=skip, ndmin=2)
    if data.shape[1] != 4:
        raise ValueError(f'{path}: expected 4 columns (u, v, x, y), got {data.shape[1]}')
    return data[:, :2], data[:, 2:]


def to_params_yaml(h):
    values = ', '.join(repr(float(v)) for v in h.ravel())
    return f'tool_localizer:\n  ros__parameters:\n    homography: [{values}]\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pairs', help='CSV file with u,v,x,y rows')
    parser.add_argument('-o', '--output', help='write tool_localizer parameters to this file')
    args = parser.parse_args(argv)

    pixels, points = load_pairs(args.pairs)
    h = homography.estimate(pixels, points)
    errors = homography.reprojection_errors(h, pixels, points)
    print(f'{len(pixels)} pairs, reprojection error: mean {errors.mean() * 1000:.1f} mm, '
          f'max {errors.max() * 1000:.1f} mm', file=sys.stderr)
    params = to_params_yaml(h)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(params)
        print(f'wrote {args.output}', file=sys.stderr)
    else:
        print(params, end='')
