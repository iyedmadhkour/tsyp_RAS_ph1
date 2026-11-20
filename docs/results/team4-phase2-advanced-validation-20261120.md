# Team 4 Phase 2, Round 2 — Advanced Safety Validation

Date: 2026-11-20  
Branch: `phase2/reliability-optimization-20261120`  
Starting commit: `7e5ac45a00457ff5c34ddfe7baed34ebdfbac9a6`

## Verdict

**Software-side fault handling and bounded simulation regressions pass. Some
boundary coverage is inconclusive during an injected odometry outage. Hardware
acceptance remains blocked on Team 1 interfaces and measurements.** The prior
frontier selector optimization remains behavior-equivalent on all six paired
benchmark maps. No physical deployment or GitHub operation occurred.

## Requirement checklist

| Requirement | Status | Result and limit |
| --- | --- | --- |
| Pose freshness, missing/stale/future/out-of-order updates | **PASS in unit tests; PASS for stale bridge fault in Gazebo** | Freshness checks gate frontier and HOME goals. Active navigation requests cancellation on stale localization. Out-of-order and non-finite odometry/scan timestamps are rejected. The Gazebo bridge pause made both pose and scan stale; a pose-only and scan-only fault were not separately injected live. |
| Frozen pose with advancing timestamps | **PASS in seeded software experiment** | Watchdog canceled all 100 stationary cases at median 26 s; physical STUCK asserted in all 100 at median 45 s. This uses the software detector model, not a noisy Gazebo localization stack. |
| Stale LiDAR and Nav2 costmap | **PASS in unit tests and combined Gazebo bridge fault** | Scan age is checked before a new goal and during an active goal. Nav2 costmap observation timeout was configured to the same explicit 0.5 s Gazebo assumption; the run logged the expected 0.50 s timeout warning. Team 1 must calibrate deployed rates. |
| Continuous padded-footprint containment | **PASS on observed same-zone Gazebo route; INCONCLUSIVE during stale-data outage** | Analytic swept support was checked between odometry samples, using the real padded footprint and 5 cm translation allowance. Same-zone route minimum was x = -0.410 m including allowance against entrance x = -1.0 m, with 1,025 samples, zero odometry gaps and zero unaligned-motion samples. During the bridge fault, the observer recorded an 8.874 s odometry gap and one unaligned-motion sample, so that interval is not claimed safe by measurement. |
| Seeded localization disturbance | **PASS as a bounded software model** | Seed `20261120`, 100 trials, x/y disturbance ±0.05 m and yaw ±5°. All frontier selections remained safe and selected the same candidate on the test map; no moving watchdog false cancels or moving false STUCK; all stationary watchdog/STUCK outcomes triggered. This is not measured physical localization performance. |
| Delayed, missing, conflicting Nav2 cancellation/action results | **PASS in callback/unit tests; hardware stop BLOCKED** | Unresolved cancel responses retain `busy`; unknown/nonterminal results and lost acceptance/results latch `NAVIGATION_UNCERTAIN`, preventing another goal. A terminal CANCELED result resolves an unconfirmed cancel; a terminal SUCCEEDED result records success. No Gazebo/Nav2 action server was modified to drop responses. Physical motor stopping is not guaranteed by these tests. |
| Battery return/HOME after stale input and cancellation | **PASS in Gazebo** | Return was requested while the bridge was paused. No new goal was selected during the outage; after bridge recovery the Writer reached HOME. Same-zone external cancellation also reached HOME. |
| STUCK detector/watchdog regression | **PASS in Gazebo** | Normal route completed HOME with movement, turns and zero-command samples without false STUCK. Static forward immobility asserted STUCK after 453 forward commands and zero odometry displacement. Watchdog canceled after 26 s; independent physical STUCK then asserted after continued immobilization. |
| Frontier retry pacing and failure-budget separation | **PASS in Gazebo** | External cancellation used a 15.0 s cancellation-only cooldown; failure-zone and failure-attempt counts stayed zero; the same frontier zone was selected after expiry; HOME followed. |
| Prior frontier optimization | **PASS, behavior equivalent** | 15 trials after 2 warmups, paired old/new implementations, identical goals and diagnostics on all maps. Captured maps improved 43.1%, 54.3%, 54.1%; seeded synthetic rooms/corridors/dense maps improved 59.2%, 61.9%, 71.9% in median selector time. Synthetic maps use seed `20261120`; they are not Gazebo maps. |
| Full regression/build | **PASS** | Full repository suite: 500 passed. Isolated ROS 2 Jazzy colcon builds: 12/12 packages. STUCK harness focused suite: 157 passed. |

## Confirmed software fixes

- Reject out-of-order and non-finite odometry samples instead of refreshing the
  fallback with invalid data.
- Reject out-of-order LiDAR timestamps and gate both new goals and active-goal
  watchdog operation on fresh pose and scan data.
- Transform fresh odometry into the map frame through a fresh map-to-odom TF;
  do not treat raw odometry coordinates as map coordinates.
- Request cancellation when localization or LiDAR becomes stale during a goal.
  Honor a stale-input cancellation latched while goal acceptance is pending.
- Keep the active action busy until a terminal result arrives. Lost acceptance,
  lost result and nonterminal/unknown result states latch
  `NAVIGATION_UNCERTAIN` and block subsequent goals. The state does not assert
  that the physical base stopped.
- Preserve an in-flight HOME goal when a battery reserve request arrives.
- Add exact swept-footprint checks for the interpolated linear-translation,
  shortest-yaw path between odometry samples.

The 0.5 s pose/scan age and 10 Hz Gazebo sensor cadence are labeled simulation
assumptions, not production timing requirements. The 5 cm translational
uncertainty is also an assumption pending Team 1 measurements. The noise
experiment includes ±5° orientation disturbances, but deployed yaw/covariance
limits are still unknown.

## Evidence

All new evidence is under
`docs/results/evidence/team4-phase2-advanced-20261120/`:

- `pytest-full-20261120.log`: final 500-test run.
- `localization-noise-seed-20261120.json`: seed, bounds, configuration/source
  hashes and outcomes.
- `frontier-comparison-seed-20261120.json`: raw paired timing results for six
  maps, input hashes and selector diagnostics.
- `same-zone-swept-retry-20261120/`: Gazebo logs, observations, patched SLAM
  runtime library check, cancellation evidence and result summary.
- `stale-inputs-20261120-181717/`: bridge fault injection, ROS build and Gazebo
  logs, topic observations, HOME result and explicitly inconclusive boundary
  gap. Earlier failed attempts remain in separate timestamped directories with
  attempt notes.
- `stuck-regression-20261120/`: ROS build, 157 focused test output, three Gazebo
  logs and command/odometry bags for normal, immobilized and watchdog scenarios.

## Hardware-dependent blockers

Team 1 must confirm sensor frame IDs, update cadence, timestamp behavior,
localization covariance/error bounds, motor command lease and stop
acknowledgment, Nav2/action-to-motor cancellation contract, battery telemetry
and reserve calibration, emergency-stop integration, radio acknowledgments,
and beacon actuator feedback. Hardware-in-loop acceptance remains outstanding.
No claim is made that canceling a Nav2 action guarantees physical motor stop.
