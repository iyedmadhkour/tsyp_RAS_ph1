# Team 4 Final Gazebo Regression Recovery

Date: 2026-11-20
Branch: `phase2/reliability-optimization-20261120`
Tested source HEAD: `f1c05a792c9382c936c5a8d86166b07a9b076bac`
Verdict: **PASS for the bounded software/Gazebo regression suite; PARTIAL for system acceptance because Team 1 hardware acceptance remains outstanding.** All changes and evidence are local. No GitHub operations were performed.

## Watchdog scenario root cause and correction

The failed run at `docs/results/evidence/team4-ultimate-validation-20261120/stuck-regression-rerun/20261120-195818/` did not reach the watchdog test path. Its bag contains only 93 `/clock` samples, from simulation time 77.712 to 78.632 s, while `/writer/odom` starts at 0.238 s and advances to 161.704 s. It contains zero `/writer/cmd_vel` samples. The launch log also shows incomplete transform/costmap initialization and missing-map/out-of-bounds messages. That evidence is inconsistent simulation-time state and an unready Nav2 graph, not a Writer watchdog failure. The old harness ran sequential Gazebo cases in a shared ROS domain/Gazebo partition and had no explicit gate before enabling the third scenario. The evidence supports cross-case simulator-state contamination and a startup/readiness race; “map missing” was a symptom, not a sufficient root-cause diagnosis.

`scripts/validate_pr1_stuck_gazebo.sh` now gives each scenario its own `ROS_DOMAIN_ID` and `GZ_PARTITION`, terminates complete session-owned launch/bag process groups, subscribes before launch, and gates the watchdog run on a nonempty `/map`, Writer scan and odometry, active Writer controller/costmap initialization, and a successful Writer-enable service response. It records the setup artifacts and verifies the selected goal before timing watchdog behavior. Bag analysis rejects missing/stale/backward simulation clocks and checks actual command/odometry signals. `git status` parsing is NUL-safe for generated paths containing spaces. No production geometry or safety thresholds were changed.

## Three bounded STUCK scenarios

Run evidence: `docs/results/evidence/team4-ultimate-validation-20261120/final-gazebo-recovery-isolated/20261120-213332/`.

| Scenario | Result | Observed evidence |
| --- | --- | --- |
| Normal navigation and HOME | **PASS** | 1,162 translational, 180 rotational and 15 zero command samples; 3,940 odometry samples; 42.81 m path; 6.2552 rad yaw range; final state HOME, no false STUCK. `/clock`: 13,397 samples, 0.002–133.962 s, no backward jumps. |
| Forward immobilization | **PASS** | 453 forward commands; 1,589 odometry samples with 0.00 m path/displacement/yaw; STUCK emitted. `/clock`: 5,430 samples, 0.002–54.292 s, no backward jumps. |
| Watchdog cancel then independent STUCK | **PASS** | Readiness gates all passed. Goal at (0.99, 0.12) was watchdog-canceled after recovery limit (26 s), retry goal at (0.89, 1.62) was selected, then the independent Writer detector emitted STUCK after 45 s of commanded movement without progress. Bag: 872 rotational and 37 zero samples, no translation, 2,002 odometry samples with zero displacement/yaw. `/clock`: 6,852 samples, 0.002–68.512 s, no backward jumps. |

Per-scenario launch logs, state/STUCK topic captures, rosbag2 recordings, command/odometry analyses, readiness data, and timelines are in `cases/{normal-home,physical-immobility,watchdog-immobility}/`. `results.txt` records all three passes. Rotation commands in the watchdog scenario were present; this scenario did not demonstrate forward translation because the selected route commanded rotation.

## Cancellation, stale inputs, boundaries, HOME

The real local `rclpy` NavigateToPose fault injection passed at `final-checks/action-faults-final.json`: delayed acknowledgment took 3.609 s against a 3 s deadline while Writer remained busy and navigation-uncertain; duplicate cancellation produced one server request. A rejected HOME cancellation remained uncertain until late terminal success, which restored HOME. The server did not publish velocity, so this verifies ROS action-state handling only.

The bounded stale-input Gazebo run is at `final-checks/stale-evidence/stale-inputs-20261120-214701/`. Stale LiDAR canceled the active goal; no exploration goal was dispatched while stale; after inputs resumed, the requested return reached HOME. The independent `/model/writer/pose` audit is `ground-truth-audit.json`: 574 poses, 34 ms maximum sample spacing, zero position cross-check residual against `/writer/world_odom`, continuous padded-footprint containment with the 5 cm uncertainty assumption, and a conservative lower x bound of -0.5792 m against the -1.0 m entrance plane. During this run the robot moved 0.1602 m during the outage and 0.0169 m after terminal cancel while settling; no nonzero command was observed after the settling window. The canceled action did not mean instantaneous physical stopping. The observer's map-TF boundary result was marked INCONCLUSIVE during the data gap; the separate ground-truth swept-footprint audit is the basis for the simulated continuous containment result.

The stale-scan outage was re-executed. This run did not demonstrate a distinct stale-pose rejection (pose-unavailable counter remained zero). Pose freshness and reordered/invalid-message cases remain covered by the automated suite, and earlier independent evidence remains preserved. Battery return request and HOME recovery were exercised; battery telemetry/calibration and long-route reserve are not physically validated.

## Final software regression

- Full automated suite: **506 passed, 0 failed, 0 skipped, 0 deselected**, 8.89 s. Coverage run: 81% statement, 1,333 branches (150 missed). Evidence: `final-checks/pytest-full.log`, `final-checks/coverage.xml`.
- Focused Writer regressions: **165 passed**. Evidence: `pytest.log`.
- Isolated ROS 2 Jazzy build: **12/12 packages finished**, 20.0 s. Evidence: `build.log` and `package-resolution.txt`.
- Refreshed watchdog scenario preflight confirms Writer code and detector were resolved from the isolated build.
- Real ROS action fault injection: **PASS** for delayed and rejected response state handling.
- Bounded stale-input/HOME and independent ground-truth boundary audit: **PASS** for the stated simulation assumptions; stale pose not re-triggered as a separate fault in this rerun.
- Existing prior evidence is preserved. The earlier timeout remains an honest failed attempt and is not counted as a pass.

## Remaining acceptance limits

Simulation demonstrated normal HOME return, forward-drive immobilization detection, watchdog/physical-detector independence, action uncertainty handling, stale-LiDAR cancellation/recovery, and continuous boundary containment along the recorded stale-input path. It does not establish all-trajectory safety or physical braking.

Team 1 hardware is still required to confirm sensor cadence/latency and covariance bounds, STUCK thresholds, command mux/lease and stop acknowledgment, residual motor behavior, calibrated battery reserve and telemetry loss, actual footprint/dynamics, emergency stop, radios/beacons and actuator feedback, and real-hardware HOME/stale-data handling. These require measured interfaces and hardware-in-the-loop acceptance before deployment.

## Evidence and local state

The failed run, successful recovery run, action fault evidence and stale-input evidence are retained under `docs/results/evidence/team4-ultimate-validation-20261120/`. Generated build/install/log/runtime cache directories are locally available but excluded from version control; raw logs, bags, manifests, command analyses and reports are retained. This report does not authorize publication.
