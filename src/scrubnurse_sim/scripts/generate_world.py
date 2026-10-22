#!/usr/bin/env python3
"""
Generate worlds/scrubnurse_cell.sdf from the recorded robot poses.

The cell layout is derived from scrubnurse_bringup/config/poses.yaml: for every
pose the grasp point is 0.146 m along the gripper's approach axis (tool0 z), and
the holders are placed so that a tool held there rests in them:

  magazine (A..F)   picked top-down: tools stand upright in slots, grasped just
                    below the handle
  handover (P)      gripper horizontal along +x: a cradle (the surgeon's hand)
                    under the horizontal tool
  return rack (R*)  gripper horizontal along +-y: a tool picked flat from the
                    table hangs upright and drops into a slot
  work table        tools lie flat; return-area grasp heights 0.246..0.249
The surgeon's hand at P is visual only: sim_surgeon takes the tool on release.

Run after re-teaching poses:
    cd src/scrubnurse_sim
    python3 scripts/generate_world.py ../scrubnurse_bringup/config/poses.yaml \
        worlds/scrubnurse_cell.sdf
"""

import math
import os
import sys

import yaml

REACH = 0.146          # tool0 -> grasp point along the approach axis
GRIP_ABOVE_CENTRE = 0.025  # upright tools are grasped this far above the shaft centre
SHAFT = (0.07, 0.012, 0.012)
HANDLE = (0.02, 0.03, 0.012)
HANDLE_X = SHAFT[0] / 2 + HANDLE[0] / 2   # handle centre along the tool's +x axis
TOOL_END = SHAFT[0] / 2                   # shaft end opposite the handle
TABLE_TOP = 0.095
COLORS = {'A': '0.85 0.2 0.2 1', 'B': '0.2 0.6 0.9 1', 'C': '0.95 0.75 0.1 1',
          'D': '0.3 0.75 0.3 1', 'E': '0.65 0.35 0.85 1', 'F': '0.95 0.5 0.15 1'}
GREY, WOOD, WHITE = '0.35 0.35 0.4 1', '0.55 0.45 0.35 1', '0.9 0.9 0.9 1'


def approach(q):
    x, y, z, w = q
    return (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))


def grasp_point(pose):
    a = approach(pose['orientation'])
    return [p + REACH * c for p, c in zip(pose['position'], a)]


def box(name, center, size, color, collide=True):
    pose = '<pose>{:.4f} {:.4f} {:.4f} 0 0 0</pose>'.format(*center)
    geometry = '<geometry><box><size>{:.4f} {:.4f} {:.4f}</size></box></geometry>'.format(*size)
    material = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    collision = (f'        <collision name="{name}_collision">{pose}{geometry}</collision>\n'
                 if collide else '')
    visual = f'        <visual name="{name}_visual">{pose}{geometry}{material}</visual>\n'
    return collision + visual


def cup(name, x, y, floor_z, inner=0.04, wall=0.005, height=0.03):
    """Square open slot whose floor is at floor_z."""
    out = ''
    half = inner / 2 + wall / 2
    for i, (dx, dy, sx, sy) in enumerate([(half, 0, wall, inner + 2 * wall),
                                          (-half, 0, wall, inner + 2 * wall),
                                          (0, half, inner, wall), (0, -half, inner, wall)]):
        out += box(f'{name}_wall{i}', (x + dx, y + dy, floor_z + height / 2),
                   (sx, sy, height), GREY)
    return out


def block(name, x_range, y_range, top, color):
    (x0, x1), (y0, y1) = x_range, y_range
    return box(name, ((x0 + x1) / 2, (y0 + y1) / 2, top / 2), (x1 - x0, y1 - y0, top), color)


def tool(name, tool_id, xyz, rpy):
    color = COLORS[tool_id]
    material = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    shaft = '<geometry><box><size>{} {} {}</size></box></geometry>'.format(*SHAFT)
    handle = '<geometry><box><size>{} {} {}</size></box></geometry>'.format(*HANDLE)
    handle_pose = f'<pose>{HANDLE_X} 0 0 0 0 0</pose>'
    pose = '{:.4f} {:.4f} {:.4f} '.format(*xyz) + '{:.6f} {:.6f} {:.6f}'.format(*rpy)
    inertia = ('<inertia><ixx>1e-6</ixx><iyy>2e-5</iyy><izz>2e-5</izz>'
               '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>')
    return f'''    <model name="{name}">
      <pose relative_to="ur_base">{pose}</pose>
      <link name="link">
        <inertial><mass>0.03</mass>{inertia}</inertial>
        <collision name="shaft">{shaft}</collision>
        <visual name="shaft_visual">{shaft}{material}</visual>
        <collision name="handle">{handle_pose}{handle}</collision>
        <visual name="handle_visual">{handle_pose}{handle}{material}</visual>
      </link>
    </model>
'''


def generate(poses):
    links = ''
    tools = ''

    # Work table under the camera's return area.
    links += block('work_table', (-0.30, 0.35), (-1.30, -0.62), TABLE_TOP, WOOD)

    # Magazine: upright tools, handle up, grasped near the top of the shaft.
    floors = []
    for t in 'ABCDEF':
        gx, gy, gz = grasp_point(poses[t])
        origin_z = gz - GRIP_ABOVE_CENTRE
        floor = origin_z - TOOL_END
        floors.append(floor)
        links += cup(f'magazine_{t}', gx, gy, floor)
        tools += tool(f'tool_{t}', t, (gx, gy, origin_z), (0.0, -math.pi / 2, 0.0))
    links += block('magazine', (-0.92, -0.67), (-0.20, -0.035), min(floors), GREY)

    # Return rack: tools hang upright, handle down, below the horizontal gripper.
    for group, ids in (('return_abc', 'ABC'), ('return_def', 'DEF')):
        rack_floors, xs, ys = [], [], []
        for t in ids:
            gx, gy, gz = grasp_point(poses[f'R{t}'])
            floor = gz - (HANDLE_X + HANDLE[0] / 2) - 0.002
            rack_floors.append(floor)
            xs.append(gx)
            ys.append(gy)
            links += cup(f'return_{t}', gx, gy, floor)
        links += block(group, (min(xs) - 0.04, max(xs) + 0.04), (min(ys) - 0.04, max(ys) + 0.04),
                       min(rack_floors), GREY)

    # Handover: the surgeon's hand under the horizontal tool (gripper points along +x at P).
    # Visual only - sim_surgeon takes the tool on release, so the retreat is not blocked.
    px, py, pz = grasp_point(poses['P'])
    links += box('surgeon_hand', (px + 0.03, py, pz - SHAFT[2] / 2 - 0.01), (0.12, 0.08, 0.02),
                 '0.95 0.8 0.7 1', collide=False)

    # A tool already lying in the return area, heading along base +x (handle towards +x).
    tools += tool('tool_C_table', 'C', (0.02, -0.95, TABLE_TOP + SHAFT[2] / 2), (0.0, 0.0, 0.0))
    return links, tools


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip().splitlines()[-1].strip())
        return 1
    with open(argv[1]) as f:
        poses = yaml.safe_load(f)['poses']
    links, tools = generate(poses)
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'scrubnurse_cell.sdf.in')) as f:
        template = f.read()
    with open(argv[2], 'w') as f:
        f.write(template.format(links=links, tools=tools))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
