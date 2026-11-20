# Team 4 frozen-pose safety checkpoint — 2026-11-20

## State

- Branch: `phase2/reliability-optimization-20261120`
- Parent before this checkpoint: `c702e1af0a9ddd8c8a07af095d0ffab179227ae9`
- Working identity verified before checkpoint: `Iyed Madhkour <iyedmadhkour4@gmail.com>`
- All changes and evidence are local. Nothing was pushed or published.
- Session-owned Gazebo, ROS, launch, bag, and validation processes were checked; none remained active after interruption.

## Implemented locally

- Added `PoseIntegrityMonitor`, which compares Writer odometry motion with a distinct, independently sourced, frame-axis-aligned reference. It detects translation and yaw disagreement while allowing different origins and interpolating asynchronous samples to a common timestamp.
- Writer autonomy now requires the configured independent reference. Missing or stale pairing, pose disagreement, or an unconfirmed recovery blocks new goals, latches `NAVIGATION_UNCERTAIN`, and requests Nav2 cancellation. Cancellation acknowledgment is not treated as proof the physical base stopped.
- Recovery requires two agreeing paired samples, fresh map-frame pose and LiDAR, no active goal, and explicit `/writer/ack_pose_integrity_recovery` acknowledgment. An external freshness fault now enters the same recovery gate as measured disagreement.
- Added actionable pairing diagnostics: sample timestamps and ages, skew, common-time pairing availability, and freshness failure reason.
- Added unit regressions for stationary samples, origin offsets, frozen translation and rotation, skew interpolation, missing/unpaired input, and explicit recovery.
- Updated the Gazebo harness to source ROS before checking tools, use the documented normal speed preset, compare goal timestamps to actual fault activation, and accept either stale-input cancellation or integrity lockout as the safety response. Its rotation phase now requires both a commanded turn and measured independent yaw motion before injecting the freeze.
- Updated the evidence auditor for integrity lockouts, the recorded lockout time, interpolation assumptions, independent rotation evidence, and continuous swept-footprint containment.

The simulation settings reuse the existing 5 cm translation allowance, 0.02 rad yaw allowance, 0.5 s pose age, and 0.35 s scan age. The 50 ms timestamp pairing allowance is a simulation assumption. Team 1 must confirm deployed independence, frame alignment, timestamps, and all limits.

## Validation evidence and limits

- Focused pose-integrity and navigation tests: **11 passed, 1 skipped** after the latest production-code changes.
- Full suite: **519 passed in 10.88 s** before the final diagnostics-only production change. It included the monitor/recovery fixes and interpolation; rerun it after resuming to validate the checkpoint as a whole.
- Isolated ROS 2 Jazzy build: **12 packages finished** in the final interrupted attempt (`stale-pose-20261120-234837/build.log`), after the latest code changes.
- In `stale-pose-20261120-233805`, the stale/delayed/missing phase and the two frozen-pose phases each recorded a phase summary of `PASS`, including HOME, pose lockout/cancel, and no goal after lockout. The continuous footprint analysis found no entrance crossing and conservative swept lower bounds above the `-1.0 m` entrance plane. The combined audit was **INCONCLUSIVE**: its then-current criteria did not recognize the pose-integrity cancellation path and sampled the rotation lockout timestamp too early. That directory also recorded a 0.0388 rad independent Gazebo yaw change during the frozen rotation phase, but the required pre-fault command/motion pairing had not yet been captured.
- Harness and auditor criteria were corrected after that run. The final rerun at `stale-pose-20261120-234837` built 12 packages but was interrupted during startup before any fault scenario began. Its partial evidence is preserved; it is not a simulation result.
- Earlier failed runs, including the original boundary-crossing evidence and the pairing/recovery failures, remain preserved under `docs/results/evidence/team4-phase2-final-20261120/` and were not rewritten.

**Overall status: PARTIAL.** The original frozen-pose condition now triggers fail-closed lockout in bounded Gazebo evidence and the recorded runs remained within the conservative swept entrance boundary. Final end-to-end acceptance is incomplete because the latest all-phase run was interrupted and the updated combined evidence auditor has not passed on a complete run. Simulation does not establish a motor-stop guarantee or physical sensor performance.

## Resume

1. Confirm this branch and the checkpoint commit; preserve all existing evidence.
2. From `/home/iyed/LivingMap-Team4`, run `scripts/validate_phase2_stale_pose_gazebo.sh` in Ubuntu 24.04 WSL with ROS 2 Jazzy and the validated SLAM overlay available. The script performs an isolated build and bounded stale/delayed/missing, frozen-translation, and frozen-rotation scenarios, then runs the ground-truth audit.
3. Require all three phase summaries and `pose-fault-audit.json` to report `PASS`. If rotation evidence is inconclusive, retain the evidence and diagnose the harness; do not relax thresholds or boundaries.
4. Rerun the complete suite with ROS and the generated interface overlay sourced:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /tmp/<latest-team4-pose-run>/install/setup.bash
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -p anyio -q tests
   ```

5. Confirm all session-owned processes stop, review results, and create any subsequent local commits only with the verified Iyed identity. Do not publish.
