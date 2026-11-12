# ECE 470 Lab Work

ROS 2 Jazzy coursework for the UR3e robot. Lab 4 implements an analytical elbow-up inverse-kinematics solver and checks its results against the Lab 3 forward model.

Two physical Lab 4 experiments were recorded on October 15, 2026. Their measured position errors are **6.164 mm** and **8.246 mm**. Both supplied terminal logs report that the requested joint pose was reached.

| Resource | Contents |
| --- | --- |
| [Lab 4 experiment record](lab4/README.md) | Targets, measurements, joint angles, error analysis, and reproduction commands |
| [Measurements](lab4/measurements.csv) | World-frame targets and user-reported ruler measurements in meters |
| [Measurement results](lab4/measurement_results.csv) | Measurements and scalar position errors in meters |
| [Run records](lab4/experiment_runs.csv) | Original logged residuals, commanded encoder angles, and run status |
| [Terminal logs](lab4/experiment_terminal_logs.txt) | Supplied experiment output and measurement notes |
| [Numerical validation](lab4/numerical_validation.csv) | Recomputed FK results and commanded encoder angles |
| [Report PDF](lab4/Lab4_Report.pdf) | Illustrated derivation and experimental results; personal details and TA review remain to be completed |
| [Inverse kinematics source](ece470lab/lab_files/src/ece470labs/ece470labs/lab_func.py) | Reusable IK and FK functions |
| [Lab 4 executable](ece470lab/lab_files/src/ece470labs/ece470labs/lab4_exec.py) | Command-line interface, dry run, and trajectory execution |

The report records positional measurements. Quantitative measured yaw and TA sign-off have not been supplied. P2's measurement unit is interpreted as centimeters, consistently with P1.

The repository includes historical build artifacts from the original checkout. The Lab 4 instructions use separate `.lab4-build`, `.lab4-install`, and `.lab4-log` directories, which are ignored by Git.


## Lab 5 Camera Sorting

[Lab 5 guide](lab5/README.md) covers camera calibration, operating commands, and color sorting. Operator-run logs record complete 3-green/1-yellow, 2-green/2-yellow, and 1-green/3-yellow sorting runs, plus a missing-source run that continued with the remaining three blocks. See [demo results](lab5/demo_results.csv), [current code PDF](lab5/Submission_Code.pdf), and [experiment archive](lab5/experiment_records.zip). These are program feedback records; TA acceptance and independent inspection of the final stacks are not recorded.

The current [configuration](lab5/config.json) uses a 0.031 m nominal block height, green/yellow pickup Z of 0.029/0.026 m, clearance Z of 0.15 m, and workspace X=0.10–0.50 m, Y=0.05–0.50 m. The camera ROI remains narrower than this configured workspace. Vertical travel uses at most three segments. Later bench tuning was not followed by another automated test run, at the operator's request.
