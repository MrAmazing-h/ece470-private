# Lab 5: Camera Sorting and Experiment Records

The main program follows the repository starter: UR3e callbacks -> blob_search() -> IMG2W() -> a frozen snapshot -> Lab 4 IK -> move_block_vision() -> two color stacks.

**Operator-run experiments are recorded.** Feedback logs show complete 3-green/1-yellow, 2-green/2-yellow, and 1-green/3-yellow runs, each with no missing blocks and confirmed valve closure. A later run logged one missing green source and continued to place the remaining three blocks. See [demo_results.csv](demo_results.csv) and [experiment_records.zip](experiment_records.zip), which contains the original lab5_runs directory with images, masks, plans, commands, and feedback. Logs do not independently verify stack stability or TA acceptance.

Current task settings: nominal block height=0.031 m; pickup Z=0.029 m for green and 0.026 m for yellow; green stack XY=(0.20,0.10) m; yellow stack XY=(0.30,0.10) m; clearance Z=0.15 m. Workspace bounds are X=0.10-0.50 m and Y=0.05-0.50 m. Native ROI=[230,140,500,340]; it covers only part of the configured workspace. Keep both stack sites empty, with source centers at least 5 cm away. Vertical travel uses at most three segments, with the existing velocity/acceleration limits. The runtime still checks IK/FK before motion; it does not plan around arbitrary obstacles. [config.json](config.json) is authoritative; task_setup.json is an earlier setup snapshot. The earlier 140-point workspace check predates the expanded bounds. Final tuning was not followed by another automated test run, at the user's request.

Measured calibration: [calibration_result.json](calibration_result.json). The user supplied red=(0.10,0.10), green=(0.20,0.10), yellow=(0.20,0.20) world meters. Centers were collected over 60 frames. beta=641.7244 px/m, theta=-0.0654575 rad, tx=-0.2316441 m, ty=-0.1431970 m; maximum fit residual=1.0665 mm. These are block-top observations, with nominal height 0.0315 m. They map same-height source blocks; measure stack-site world XY separately rather than applying this map to bare-table clicks. No robot or valve commands were sent.

Independent check: the user moved red to world (0.10,0.20) m. With the original fit unchanged, 60 frames produced median world (0.100921,0.200043) m: position error 0.9220 mm, maximum per-frame error 1.3549 mm. Green/yellow controls stayed in place. This passes the software's 5 mm check threshold, which is not an instructor grading specification. It validates this one location at the same block-top height, rather than every possible tabletop position. Records: [independent_validation.json](independent_validation.json), [CSV](independent_validation.csv).

Profile: [2026-11-12-v1.json](color_profiles/2026-11-12-v1.json). Green HSV: [45,100,80] to [75,255,255]; yellow: [15,140,150] to [32,255,255]. The original color check used ROI [100,160,570,355]; current sorting uses the narrower ROI above. Block area: 180-1200 pixels squared. Keep test objects inside the configured workspace; lighting or camera changes require rechecking.

## Source Check

On 2026-11-12, the pre-integration lab5_exec.py matched [Popilworck/ece470](https://github.com/Popilworck/ece470/blob/main/ece470lab/lab_files/src/ece470labs/ece470labs/lab5_exec.py) exactly. The local blob_search.py differed in HSV values, blob filters, keypoint display, and a commented beta=660 note. This identifies the changes, not their author. That GitHub repository is the supplied upstream, not an instructor-owned repository.

JSON settings, tests, the separate color viewer, and other helper tools were added during earlier assistant work. They are not all part of the upstream starter. The current main program calls the starter's blob_search(), not color_vision.py. Backups before this integration are in [code_backups](code_backups).

## Implemented in This Revision

- Appendix B image-to-world conversion, with explicit units and axis signs.
- Calibration fit from at least three non-collinear measured pixel/world pairs.
- One snapshot containing both colors after the settling interval.
- Lab 4 IK/FK verification for pickup, transfer, placement, retreat, and possible stack heights.
- Suction feedback with freshness checks, polarity, and debounce.
- Missing-source errors followed by attempts on remaining blocks.
- Stack height increases only after a successful placement.
- Abort on motion/feedback failure or suction loss.
- Timestamped snapshots, plans, encoder commands, and event records.
- Read-only default mode; command interfaces exist only in explicit motion modes.
- Concise English function descriptions.

The program validates positions and kinematics; it does not compute collision-free paths through arbitrary obstacles. Calibration and selected physical poses have recorded checks; the full expanded workspace has not been physically validated.

## Files Your Teammate Should Read

| File | Purpose |
| --- | --- |
| [blob_search.py](../ece470lab/lab_files/src/ece470labs/ece470labs/blob_search.py) | Original color/filter worksheet, IMG2W, calibration fit |
| [lab5_exec.py](../ece470lab/lab_files/src/ece470labs/ece470labs/lab5_exec.py) | Starter UR3e methods, callbacks, operating modes, logs |
| [lab5_func.py](../ece470lab/lab_files/src/ece470labs/ece470labs/lab5_func.py) | Settings, IK validation, pick/place sequence, stack counts |
| [lab5_calibrate.py](../ece470lab/lab_files/src/ece470labs/ece470labs/lab5_calibrate.py) | Fit and save calibration from measured CSV |
| [config.json](config.json) | Current measured calibration and operator-selected bench settings |
| [calibration_points.csv](calibration_points.csv) | Three actual pixel/world pairs, ordered red, green, yellow |
| [examples](examples) | Explicitly synthetic data for offline checks |

The native blob_search() reads vision.hsv, vision.roi, vision.area_px, and vision.morphology_kernel from config.json. Its shape filters remain in blob_search.py. The optional older lab5_color_view tool uses its own shape filters; validate sorting with the native main detector.

## Build

This creates an isolated Lab 5 overlay and does not launch a robot or camera:

~~~bash
cd /home/xliu173/ece470/ece470lab/lab_files
source /opt/ros/jazzy/setup.bash
colcon --log-base .lab5-log build --base-paths src \
  --build-base .lab5-build --install-base .lab5-install --symlink-install
~~~

In each later lab terminal:

~~~bash
source /home/xliu173/ece470/lab5/env.sh
~~~

The environment script uses the workstation's existing domain mapping. Do not start a second camera driver if one is already streaming.

## Test Without a Robot or Camera

~~~bash
cd /home/xliu173/ece470
source /opt/ros/jazzy/setup.bash
python3 -m pytest -q lab5/test lab4/test_lab4.py lab4/test_lab4_exec.py
~~~

Use explicit synthetic world detections to test planning while color work is deferred:

~~~bash
source /home/xliu173/ece470/lab5/env.sh
ros2 run ece470labs lab5_exec --dry-run --no-gui \
  --detections "$LAB5_REPO/lab5/examples/synthetic_detections.json" \
  --config "$LAB5_REPO/lab5/examples/synthetic_config.json"
~~~

Expected: four planned blocks, six named poses per block, and no ROS initialization or hardware commands. This isolates geometry and planning; it does not test real color detection or pickup.

Check the calibration fitter on synthetic data without saving anything:

~~~bash
ros2 run ece470labs lab5_calibrate \
  --points "$LAB5_REPO/lab5/examples/synthetic_calibration_points.csv" \
  --config "$LAB5_REPO/lab5/examples/synthetic_config.json" --check-only
~~~

Expected: beta=1000 px/m, theta=0 rad, tx=-0.1 m, ty=-0.3 m, residuals near zero. These are test values, not bench measurements. Synthetic configurations cannot be used with --execute or --park-only.

## Inspect Images and Record Calibration Pixels

With the existing camera driver publishing /image_raw:

~~~bash
ros2 run ece470labs lab5_exec --vision-only --config "$LAB5_CONFIG" \
  --calibration-points "$LAB5_REPO/lab5/calibration_points.csv"
~~~

This creates no trajectory publisher and no suction service client.
The GUI uses q/Esc to exit, f to freeze/unfreeze, and s to save images.
Left-click prints full-image (column,row). With --calibration-points, each click appends a CSV row with blank world coordinates. Fill those two fields with ruler-measured world X/Y in meters.

Keep the image resolution and flip unchanged during calibration. Click carefully on known target centers. Orange calibration-marker thresholds have not been validated in the current scene.

## Calibration Convention and Measurements

Appendix B defines camera x along image rows and camera y along columns:

~~~text
camera_x = (row - height/2) / beta
camera_y = (col - width/2) / beta

Rcw = [[cos(theta), -sin(theta)],
       [sin(theta),  cos(theta)]]

camera_xy = Rcw @ world_xy + [tx,ty]
world_xy  = Rcw.T @ (camera_xy - [tx,ty])
~~~

beta is pixels/meter; theta is radians; tx/ty are meters in the camera frame.
The fitter estimates this same mapping from corresponding pixels and measured world positions. It rejects reflected axes, nearly collinear points, and maximum fit error over 5 mm by default.

Use at least three non-collinear pairs; four or more well-spaced points are preferable.
The two orange centers on the calibration stick are 0.10 m apart. Move a target off that line for an additional known world position.
Use the Lab 4 world origin/axes, rather than copying the pendant's Base coordinates.

~~~bash
ros2 run ece470labs lab5_calibrate \
  --points "$LAB5_REPO/lab5/calibration_points.csv" \
  --config "$LAB5_CONFIG"
~~~

The command saves camera parameters and a per-point validation CSV. Verify at additional locations not used in the fit. Low fitting residuals alone do not confirm physical accuracy. Tabletop calibration can also differ at block-top height because of parallax.

The legacy two-argument IMG2W(col,row) can use module constants for the worksheet. The integrated runner explicitly passes calibration from config.json; changing only those module constants does not change the runner's calibration.

## Remaining Bench Configuration

Camera parameters come from the user's three measured block positions. Stack sites and workspace bounds are program choices based on the calibrated view, not additional ruler measurements. The user confirmed contact height Z=0.031 m.

| Field | Required value |
| --- | --- |
| camera.beta/theta/tx/ty | Fitted from actual measured pixel/world pairs |
| task.stack_xy_m.green/yellow | Selected [0.20,0.10] and [0.30,0.10], world meters |
| task.workspace_xy_m | Selected [0.10,0.50,0.05,0.50], world meters; camera ROI is narrower |
| task.stack_clearance_m | Keep source centers 0.05 m away from both empty stack sites |
| task.block_height_m | 0.031 m, latest user-confirmed block-top/contact height |
| task.table_z_m/contact_offset_m | Actual table height and cup contact adjustment |
| task.pickup_offset_m | Green=0.0 m, yellow=-0.003 m; applied only to pickup |
| task.safe_z_m | Clearance above the tallest stack and intervening objects |
| gripper.threshold/holding_above_threshold | Confirm using empty and held readings |

The digital output 0, analog input 0, and threshold 2.0 come from the existing Lab 2 code. Inspect live feedback without changing outputs:

~~~bash
ros2 run ece470labs lab5_exec --io-only --duration 10 --no-gui --config "$LAB5_CONFIG"
~~~

Only successful placements increase the stack height. Start each sorting run with empty stack sites. With the current 0.031 m nominal height and -0.002 m contact offset, placement heights are 0.029, 0.060, and 0.091 m for both colors. The extra yellow pickup offset does not change layer spacing; a smaller measured yellow block height has not been supplied.

## Physical Testing After Color and Calibration Are Verified

The following commands are for the operator; the assistant did not command the arm during final tuning. Recorded results are in demo_results.csv.

A live --dry-run observes the camera and validates its frozen plan without motion:

~~~bash
ros2 run ece470labs lab5_exec --dry-run --config "$LAB5_CONFIG"
~~~

Planning rejects detections closer than 1 cm rather than commanding duplicate pickups. This check does not replace proper color validation. Separate blocks for pickup, and remove calibration rulers/tools from the transfer path before execution.

After the live plan matches measured positions, the existing driver is ready, and paths are clear, keep all four blocks in view for a one-block trial:

~~~bash
ros2 run ece470labs lab5_exec --execute --first-only --config "$LAB5_CONFIG"
~~~

--first-only validates the complete four-block scene, then attempts only the first block in the sorted plan (green, lowest world X first). The remaining blocks stay untouched. It does not change the saved expected scene count. The single_block_test log records the selected source. The same option works with --dry-run.

--expected-blocks 1 means exactly one block must be visible; it is not a pickup limit. Use that override only after physically removing the other blocks from the camera view.

After the trial, redistribute all four blocks and clear both stack sites. Then test the required four-block task:

~~~bash
ros2 run ece470labs lab5_exec --execute --config "$LAB5_CONFIG"
~~~

Motion modes require operator Enter confirmation. The sequence parks the arm, waits five seconds, freezes both color lists, attempts the blocks, returns camera-clear, and exits. Missing pickups are logged individually and do not consume stack levels. Other hardware failures abort.

For the manual's missing-block demonstration, --pause-after-snapshot pauses while the arm is parked. Coordinate removal with the TA, clear the work area, then resume. Never reach in while the arm is moving.

Ctrl-C stops additional program steps; a trajectory already sent may continue. At the user's request, execute-mode cleanup sends valve OFF before destroying the node, including on an exception or Ctrl-C. It requires fresh digital output OFF and released-vacuum feedback before logging valve_closed. A failed closure is reported as an error, not success. Support any held block before interrupting a run, because closing suction can release it. This cleanup has unit tests; physical valve closure still needs verification with the driver running.

## Records and Submission

Run records are written under LAB5_LOG_DIR (normally lab5_runs), with run.json, camera image, overlay, and masks. A dry-run is a software record, not a physical experiment.

The manual requires two demonstrations with different initial block positions and suction-feedback verification. No formal report is required; submit the edited code document. The existing export tool can regenerate Submission_Code.html/PDF after final tuning:

~~~bash
cd "$LAB5_REPO"
python3 lab5/export_submission.py
~~~

Submission_Code.pdf and Submission_Code.html were regenerated from the final code on 2026-11-12. Personal details on the first page still need completion. Historical automated-test counts predate final hardware tuning and should not be presented as checks of the final settings.

Sources: [Lab 5 Manual](https://coecsl.ece.illinois.edu/ece470/Labs/Lab5_Manual.pdf), [Appendix B](https://coecsl.ece.illinois.edu/ece470/Labs/AppendixB.pdf), [supplied upstream repository](https://github.com/Popilworck/ece470).

