# Camera calibration

The tool localizer turns a tool's pixel position into robot coordinates with a homography: a
3×3 matrix that maps camera pixels onto the table plane in the UR `base` frame. It is
calibrated from pairs of points: where the camera sees a target, and where the robot's tool0
is when a pointer in the gripper touches the same target. Recalibrate whenever the camera or the
table moves.

## What you need

- **Calibration block:** 40 × 40 × 10 mm, printed in **matte black**
  ([`calibration_block.stl`](calibration_block.stl), source
  [`calibration_block.scad`](calibration_block.scad)). The camera finds it as a dark square on the
  light table, so calibrate on the wood surface, not on dark mats. The shallow cone at the top
  centre guides the pointer tip; set `dimple_depth = 0` in the `.scad` for a plain block. Any
  dark, square object of that size works too, e.g. a black 40 mm PC fan.
- **Pointer:** anything the gripper holds firmly and centred, with a sharp tip pointing straight
  down, e.g. a 3D-printed cylinder with a screwdriver-like tip. The fingers centre it on the
  tool0 axis; check by rotating the wrist 180° above a mark: the tip should stay on the mark.

## Steps

1. Start the cell: `ros2 launch scrubnurse_bringup real.launch.py robot_ip:=<robot ip>`
   (the controller starts paused; nothing moves on its own). To jog with the pendant, stop the
   External Control program; the driver keeps publishing the arm position.
2. Open the dashboard (http://localhost:8080) and go to **Calibration**.
3. Put the block on the table and move the arm out of the camera view. When the camera view
   shows the block outlined in green, click **Select pixel position** (the block centre is
   averaged over one second).
4. Jog the pointer onto the block centre with the pendant: first in x/y above the block, then
   down until the tip almost touches it, then correct x/y again. Do not move the block. Click
   **Select reference position**.
5. Repeat for 20 or more positions spread over the whole return area, corners included. The
   recorded points are drawn on the camera view.
6. Check the errors. *Error* is each pair against the fit of all pairs; *Unseen* is each pair
   against a fit made without it, which is how well the matrix predicts new positions. Remove
   pairs with an unseen error well above the rest (usually a mis-pointed tip) and record them
   again.
7. Click **Save pairs and homography**. This writes `pairs_<time>.csv` and
   `homography_<time>.yaml` to `~/scrubnurse_calibration` (`calibration_dir`). Copy the
   `homography` line into the `tool_localizer` section of
   `src/scrubnurse_bringup/config/scrubnurse.yaml` and restart the cell.

Saved pairs can be loaded again from the list next to **Load**, e.g. to add more points later.
The matrix can also be recomputed from a pairs file on the command line:

```bash
ros2 run scrubnurse_perception compute_homography pairs.csv -o homography.yaml
```

## Settings

In the `scrubnurse_dashboard` section of `scrubnurse.yaml`:

| Parameter | Meaning |
|---|---|
| `calibration_block_px` | Block side in pixels (40 mm ≈ 44 px on the cell's camera); found within −30 % / +40 % |
| `calibration_roi` | Image polygon to search in (the table work area) |
| `calibration_dir` | Where pairs and matrices are saved |
