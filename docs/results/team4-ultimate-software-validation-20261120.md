# LivingMap Team 4 — Ultimate Software Validation Update

Date: 2026-11-20
Branch: `phase2/reliability-optimization-20261120`
Starting checkpoint: `66b23b9fc2fe6b85c32614bbed24361e8eb93204`
Acceptance: **PARTIAL overall; newly closed stale-outage boundary evidence is
PASS within the recorded Gazebo model and motion assumptions.** All work is
local. No hardware was connected and no GitHub operation was performed.

## Results from this continuation

### Stale-input outage and continuous boundary evidence — PASS in this run

The bounded Gazebo scenario pauses the Writer scan/odometry adapter while a
frontier goal is active, requests battery return during the outage, then
restores valid inputs. The latest run against the final Writer code is
`docs/results/evidence/team4-ultimate-validation-20261120/stale-ground-truth/stale-inputs-20261120-194933/`;
the preceding independent run remains at `stale-inputs-20261120-192519/`.

- The latest bridge pause/resume was observed at simulation times 2.312 s and
  10.872 s. The frontier cancel request was observed at 2.602 s and its
  terminal cancellation at 2.672 s. No new goal was started while inputs were
  stale; after recovery the Writer reached HOME.
- Gazebo `/model/writer/pose` recorded 408 world-frame poses at 34 ms spacing
  over 13.838 s. They matched `/writer/world_odom` at all 408 simulation
  stamps (zero position residual; maximum yaw residual `7.34e-8` rad). The
  initial world spawn at `(-11, 0, 0)` was registered against the initial Nav2
  map pose `(0, 0)`; observed `map -> writer/odom` remained identity to
  floating-point precision.
- During the stale interval the center moved 0.0472 m and rotated 0.0991 rad;
  about 0.0335 m occurred before the cancel request. Ground truth recorded
  approximately 0.0005 m of residual movement after the terminal cancel/zero
  command while the base settled. No nonzero `cmd_vel` was observed after the
  cancel-settling window. The preceding run measured 0.025 m residual
  movement, so residual varies with motion state at cancellation. Neither run
  establishes a physical motor-stop guarantee.
- Between each pair of world-pose samples, the audit bounded any center motion
  by the SDF diff-drive maximum speed (1.2 m/s). It subtracted the full padded
  footprint circumradius (0.4884 m) and the configured 5 cm translational
  allowance. The resulting yaw-independent lower bound was `x = -0.5792 m`,
  0.4208 m inside the permitted entrance plane `x = -1.0 m`. This includes all
  intermediate headings, including orientation disturbances; the SDF speed
  cap and 5 cm uncertainty are simulation assumptions, not measured hardware
  limits.
- The older odometry observer still reports its own outage interval as
  `INCONCLUSIVE` in that run's `results.json`; it lacked map TF during the gap.
  The separate independent-pose audit closes that simulated evidence gap and
  does not overwrite the original observer result. Its method and result are
  in `ground-truth-audit.json`.

### Nav2 action cancellation fault injection — PASS for ROS action-state handling

`action-faults/final-results.json` records a real local `rclpy` client/server
on `/writer/navigate_to_pose`:

- A delayed cancellation acknowledgment arrived after 3.615 s, beyond the
  configured 3 s bound. While the test server continued the goal, Writer
  stayed busy and entered `NAVIGATION_UNCERTAIN`; duplicate cancel calls sent
  only one server request. The eventual terminal `CANCELED` result resolved
  the tracked action.
- A rejected cancellation response during a HOME goal latched uncertainty.
  A later terminal `SUCCEEDED` result resolved the state to HOME. Duplicate
  cancellation was again sent only once.

The harness has no `cmd_vel` publisher. It does not simulate a server restart,
an indefinitely missing terminal result, or prove physical stopping. The
existing policy remains conservative: unresolved action state blocks another
goal; a terminal action result resolves the ROS action state only. The
configured 3 s deadline is now checked by a steady-clock timer, so its
uncertainty latch continues if ROS simulation time pauses.

### ASGI worker handoff — defect isolated and regression fixed

The three previously hanging request tests blocked in HTTP request handling.
A minimal `anyio.to_thread.run_sync()` also blocked in this environment, and
system Python's AnyIO 4.2.0 behaved the same as the project environment's
AnyIO 4.16.0. A repeated standard-library worker-handoff probe also stalled.
The available evidence does not establish an AnyIO-version-only fault. No
compatible package was cached and package-index DNS failed, so dependencies
were not changed.

The command-post app now uses async route handlers, keeps short local callbacks
on the ASGI loop, and serves cached HTML and contained static assets without
`FileResponse` worker calls. Assertions were preserved and the dashboard test
now checks static JavaScript delivery and forbids the affected AnyIO worker
path. The three targeted request tests passed and the full suite passed
506/506. Detailed diagnosis is at
`docs/results/evidence/team4-ultimate-validation-20261120/asgi-worker-diagnosis.md`.

### Final STUCK/watchdog regression rerun — PARTIAL

After the steady-clock deadline change, the isolated STUCK harness was rerun at
`docs/results/evidence/team4-ultimate-validation-20261120/stuck-regression-rerun/20261120-195818/`.
The 12-package Jazzy build and all 165 focused detector, wiring, watchdog, and
navigation tests passed. Normal navigation returned HOME with translation,
rotation, zero-command intervals, and 42.75 m odometry path; the detector did
not report false STUCK. The static-base scenario logged 455 forward command
samples, 1,770 odometry samples with zero displacement, and emitted STUCK.

The third scenario timed out after its 210 s bound without emitting a
watchdog-cancel or STUCK event. Its Writer state and STUCK topic logs were
empty; Nav2 repeatedly reported no map on its static costmaps and the robot
outside the costmap. Thus this rerun did not exercise the watchdog path and is
not a pass. Previous independent watchdog-then-STUCK runs at
`docs/results/evidence/team4-phase2-validation-20261120/stuck/` and
`docs/results/evidence/team4-phase2-advanced-20261120/stuck-regression-20261120/`
remain valid recorded passes. The new timeout reduces confidence in repeatable
setup; it does not by itself establish a production regression. No production
code changed during this rerun.

## Final regression and build

- Full test suite: **506 passed, 0 failed, 0 skipped, 0 deselected** in 9.04 s
  under coverage measurement.
  Evidence: `docs/results/evidence/team4-ultimate-validation-20261120/pytest-final.log`.
- Coverage rerun with statement and branch measurement: **81% statement**
  (3,439 statements, 637 missed); **1,333 branches, 150 missed**. This is an
  audit metric, not a safety claim. The Writer frontier node, dashboard API,
  and launch/lifecycle wrappers remain the main lower-coverage areas.
  Evidence: `coverage-summary.txt` and the final `pytest-final.log`.
- Isolated ROS 2 Jazzy build: **12/12 packages finished** in
  `/tmp/team4-ultimate-stale-deadline/`; no errors. Existing unused
  `CATKIN_INSTALL_INTO_PREFIX_ROOT` CMake warnings and disabled Python bytecode
  notices remain. Evidence: `stale-ground-truth/stale-inputs-20261120-194933/build.log`.
- Relevant bounded Gazebo stale-input/HOME scenario: **PASS**, with the
  independent continuous-boundary result described above. The separate
  same-zone, STUCK/watchdog, SLAM and frontier-lifecycle results remain in the
  previous evidence directory and are unaffected by this command-post change.
- Incomplete earlier test and Gazebo attempts remain preserved in separate
  evidence files/directories and are not counted as passes.

## Phase 2 checklist status

| Domain | Status | Accepted evidence and remaining limit |
| --- | --- | --- |
| 1. Localization noise, drift and uncertainty | **PARTIAL** | Seeded software model covers 100 trials at ±5 cm and ±5°; stale-input Gazebo ground truth is now analyzed. No stochastic Gazebo SLAM-noise experiment or measured sensor statistics. |
| 2. Continuous padded-footprint safety | **COMPLETE for tested simulation paths** | Same-zone swept path and stale bridge outage are bounded with the padded footprint, 5 cm allowance, and simulator motion cap. This is not hardware acceptance or every possible trajectory. |
| 3. Navigation cancellation and recovery | **PARTIAL** | Delayed/rejected real ROS action responses, uncertain-state blocking and late HOME success pass. The 3 s response deadline now uses a steady-clock timer independent of `/clock`. Server restart and physical motor acknowledgment remain unvalidated. |
| 4. Sensor freshness and fault handling | **PARTIAL** | Stale combined scan/pose outage canceled the active goal, prevented new dispatch, and recovered to HOME. Unit cases cover invalid and reordered messages; independent pose-only and scan-only live Gazebo faults remain. |
| 5. STUCK detector and watchdog | **PARTIAL** | Prior bounded watchdog-then-STUCK Gazebo runs passed and the latest normal/forward-immobilization cases passed, but the latest attempted watchdog rerun timed out before a goal due missing costmap data. Reproduce the watchdog case reliably. Hardware thresholds remain uncalibrated. |
| 6. Frontier lifecycle | **COMPLETE for recorded policy scenarios** | Prior retry/cooldown, budget exhaustion, 120 s expiration/reactivation and same-zone cancellation-only re-selection evidence remains preserved. No frontier-selection algorithm change occurred. |
| 7. Battery/HOME | **PARTIAL** | Return requested during stale inputs and HOME completed after recovery. Physical battery calibration, telemetry-drop behavior and long-route reserve remain unverified. |
| 8. Radio, relays and beacon faults | **PARTIAL** | Existing mocks and software policy tests remain; physical RF loss, beacon acknowledgment and actuator behavior require Team 1. |
| 9. SLAM/mapping | **PARTIAL** | Isolated Karto runtime overlay and positive grid-edge fix remain validated in prior missions. Lower-passage wall occlusion and noisy physical mapping remain unresolved. |
| 10. Repeatable complete missions | **PARTIAL** | Bounded normal/HOME, STUCK, watchdog, stale-input and retry missions exist as separate runs; no single combined all-failure acceptance mission was run. |
| 11. Benchmarking/optimization | **COMPLETE for previously compared maps** | Preserved paired benchmark shows identical goals/diagnostics and selector speed improvements of 43.1–54.3% on captured maps and 59.2–71.9% on seeded maps. Selector implementation was not changed here. |
| 12. Team 1 integration handoff | **PARTIAL / HARDWARE-BLOCKED** | ROS interfaces, mock boundaries and Team 1 questions are documented in `team4-team1-integration-readiness.md`; actual driver/mux/lease, stop acknowledgment, calibrated battery/radio and dispenser feedback require Team 1. |

## Remaining acceptance boundaries

- The simulation uses assumed 0.35 s scan freshness, 0.5 s pose and Nav2
  costmap timeouts, 10 Hz scan cadence, 1.2 m/s SDF linear-speed cap, 5 cm
  translation allowance, and seeded ±5° yaw disturbance. Team 1 must provide
  measured rates, timestamps, covariance/error bounds and approved limits.
- A Nav2 cancel acknowledgment and a zero `cmd_vel` message do not prove
  physical stopping. A hardware command lease, safety mux, stop acknowledgment,
  and safe HIL protocol are prerequisites for motor-stop acceptance.
- Physical LiDAR, fused localization, battery, radio, beacon dispenser and
  emergency-stop behavior have not been accepted or deployed.
- There is no demo video or formal sign-off evidence.
- No optimization beyond previously validated frontier-selection work was
  started.

## Local checkpoint

The intended checkpoint is on
`phase2/reliability-optimization-20261120`, authored and committed as
`Iyed Madhkour <iyedmadhkour4@gmail.com>`. Earlier reports and evidence were
preserved. The final commit hash is reported by the local Git commit; nothing
was pushed, published, merged, or written to GitHub.
