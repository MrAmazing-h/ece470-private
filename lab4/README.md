# Lab 4 Inverse Kinematics Experiment Record

Two UR3e experiments were performed using world-frame position targets and fixed-pitch, fixed-roll elbow-up inverse kinematics. Both supplied terminal logs report `Requested joint pose reached.` The measured scalar position errors were **6.164 mm** for P1 and **8.246 mm** for P2.

The records were supplied by the user on October 15, 2026, from workstation `eceb-3071-l07`. This workstation is configured for robot bench 7 and `ROS_DOMAIN_ID=57`.

## Targets and Measurements

Positions in this table use the world frame defined in Figure 4.3 of the Lab 4 manual. Measurements refer to the suction-cup contact center, using the ruler measurements supplied by the user.

| Point | Target XYZ in meters | Target yaw in degrees | Reported measured XYZ in centimeters | Position error in millimeters |
| --- | --- | --- | --- | --- |
| P1 | (0.270, 0.330, 0.230) | 90 | (26.5, 32.8, 22.7) | 6.164 |
| P2 | (0.180, -0.100, 0.060) | -45 | (18.2, -10.0, 6.8) | 8.246 |

P1's measurement note explicitly uses centimeters. P2's unit was not repeated and is interpreted as centimeters, consistently with P1. Both measurements are stored in meters in [measurements.csv](measurements.csv).

The original six unmeasured candidate points are preserved in [proposed_poses.csv](proposed_poses.csv). They are not reported as completed physical experiments.

## Position Errors

The residual is measured position minus target position. The scalar error is the Euclidean length of that residual.

| Point | X residual in millimeters | Y residual in millimeters | Z residual in millimeters | Scalar error in millimeters |
| --- | --- | --- | --- | --- |
| P1 | -5 | -2 | -3 | 6.164 |
| P2 | +2 | 0 | +8 | 8.246 |

For P1, the scalar error is `sqrt(5^2 + 2^2 + 3^2) = sqrt(38) = 6.164 mm`.
For P2, it is `sqrt(2^2 + 0^2 + 8^2) = sqrt(68) = 8.246 mm`.
The mean scalar error across these two trials is **7.205 mm**. P2's largest residual component is its vertical position, at +8 mm. These two samples do not isolate a dominant error source.

## Commanded Joint Angles

These are the target encoder angles printed by the executable, after the Lab 3 encoder offsets. They are not independently sampled actual joint angles. Units are degrees; ROS trajectory positions use radians.

| Point | Base | Shoulder | Elbow | Wrist 1 | Wrist 2 | Wrist 3 |
| --- | --- | --- | --- | --- | --- | --- |
| P1 | 179.38511109 | -60.34319423 | 58.46989844 | -88.12670421 | -90.00000000 | -0.61488891 |
| P2 | 122.74110329 | -61.35864394 | 112.19283558 | -140.83419165 | -90.00000000 | 77.74110329 |

## Terminal Results

| Point | Original logged IK/FK residual in meters | Terminal result |
| --- | --- | --- |
| P1 | 1.360e-16 | Requested joint pose reached |
| P2 | 4.013e-16 | Requested joint pose reached |

[Terminal logs](experiment_terminal_logs.txt) preserve the supplied matrices, joint-angle output, and completion messages. [Structured run records](experiment_runs.csv) retain the original logged numbers. [Numerical validation](numerical_validation.csv) is recomputed from the current code and can differ in the last digits because of floating-point rounding.

The small numerical residual checks consistency between IK and FK. It does not measure physical accuracy. Quantitative measured yaw and TA sign-off were not supplied; terminal completion messages do not establish either.

## Coordinate and Tool Conventions

Command syntax:

```bash
ros2 run ece470labs lab4_exec x_world y_world z_world yaw_degrees
```

XYZ are absolute world coordinates in meters. Yaw is in degrees. The model's base origin is `(-0.150, +0.150, +0.010)` meters in the world frame, with aligned axes. Therefore:

```text
x_base = x_world + 0.150
y_base = y_world - 0.150
z_base = z_world - 0.010
```

The pendant's Base/TCP coordinates use a different origin from the lab world frame. Its Rx/Ry/Rz values can use a rotation-vector representation, so they cannot be compared directly to the commanded yaw. Use the same TCP reference point when comparing poses.

## Build and Numerical Tests

From the repository root:

```bash
cd ece470lab/lab_files
source /opt/ros/jazzy/setup.bash
colcon --log-base .lab4-log build --base-paths src \
  --build-base .lab4-build --install-base .lab4-install --symlink-install
source .lab4-install/local_setup.bash
```

Run the regression tests from the repository root, with the ROS environment sourced:

```bash
python3 -m pytest -q lab4/test_lab4.py lab4/test_lab4_exec.py
```

The recorded software validation passed 24 tests, including 200 deterministic forward-generated poses. See [validation.txt](validation.txt) for the earlier software validation record.

## Recorded Experiment Commands

On bench 7, start the driver in terminal 1 from `ece470lab/lab_files`:

```bash
source ur3_setup.sh
source .lab4-install/local_setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=192.168.10.53 \
  headless_mode:=true launch_rviz:=true \
  kinematics_params_file:="/opt/ros/jazzy/share/ur_robot_driver/urdf/Robot7_calibration.yaml"
```

The calibration path and domain mapping are specific to the laboratory setup. On another workstation, use its actual robot calibration and domain configuration.

In terminal 2, from `ece470lab/lab_files`:

```bash
source ur3_setup.sh
source .lab4-install/local_setup.bash
```

P1 was run with:

```bash
ros2 run ece470labs lab4_exec 0.27 0.33 0.23 90
```

P2 was run separately with:

```bash
ros2 run ece470labs lab4_exec 0.18 -0.1 0.06 -45
```

To check either target without commanding motion, append `--dry-run`. For physical execution, the program asks for operator confirmation. Check the arm's path, move the monitor out of reach, and verify Remote/Normal state before confirming. Mathematical endpoint reachability does not check the intermediate path for collisions.

## Data Files and Report

| File | Purpose |
| --- | --- |
| [measurements.csv](measurements.csv) | Source table of targets and ruler measurements; XYZ in meters, yaw in degrees |
| [measurement_results.csv](measurement_results.csv) | Generated scalar position errors in meters |
| [experiment_runs.csv](experiment_runs.csv) | Original logged target encoder angles, numerical residuals, and run status |
| [experiment_terminal_logs.txt](experiment_terminal_logs.txt) | Supplied terminal output and measurement notes |
| [numerical_validation.csv](numerical_validation.csv) | Recomputed FK poses and encoder angles |
| [proposed_poses.csv](proposed_poses.csv) | Archived unmeasured candidate points |
| [Lab4_Report.pdf](Lab4_Report.pdf) | English report draft with derivation, figures, and measured results |
| [Lab4_Report.html](Lab4_Report.html) | Generated HTML report |
| [generate_report.py](generate_report.py) | Rebuilds figures, numerical validation, errors, and report |

From the repository root, regenerate the report with:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 lab4/generate_report.py
```

Dependencies include Python 3, NumPy, SciPy, and Matplotlib. PDF export uses Google Chrome or Chromium. Edit report wording in the template in `generate_report.py`, because regenerating overwrites `Lab4_Report.html` and the PDF.

Complete the report's name, partner, TA, section, submission date, and TA-review status before submission. Add any actual yaw observations and measurement-tool resolution if recorded. No experiment photographs were supplied for this record.

## Figures

The following original computer-generated figures support the derivation.

![World and base frame translation](figures/world_base.png)

![Top view of wrist and tool offsets](figures/top_view.png)

![Side view of the elbow-up construction](figures/side_view.png)

## Upload to GitHub

Review and stage the source, records, and report from the repository root:

```bash
git status --short
git add README.md .gitignore lab4 \
  ece470lab/lab_files/src/ece470labs/ece470labs/lab_func.py \
  ece470lab/lab_files/src/ece470labs/ece470labs/lab4_exec.py \
  ece470lab/lab_files/src/ece470labs/package.xml \
  ece470lab/lab_files/src/ece470labs/setup.py
git diff --cached --stat
git commit -m "Record Lab 4 IK experiments and measured position errors"
git push origin main
```

These commands are provided for the repository owner to run. This documentation update has not committed or pushed any changes.
