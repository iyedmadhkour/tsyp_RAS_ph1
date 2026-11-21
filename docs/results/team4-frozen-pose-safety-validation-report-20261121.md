# Team 4 frozen-pose safety validation — 2026-11-21

## Checkpoint and scope

- Branch: `phase2/reliability-optimization-20261120`
- Starting HEAD: `ccc873956339fb1b774cf55648035c3569ac7308`
- Work stayed local. No remote, GitHub account, credential, PR, or published branch was accessed or changed.
- All earlier evidence directories were preserved.

## Implemented corrections

- Pose freshness now allows a sample to be ahead of the current ROS clock only within the already configured symmetric pairing allowance. Samples farther ahead remain invalid. Added tests for both sides of that bound.
- `scan_frames` uses idempotent ROS shutdown. The validation harness deliberately terminates this process to replace it with a fault relay; the former `rclpy.shutdown()` raised a second-shutdown error during normal test teardown.
- The harness waits for active SLAM, a transient-local map, active Writer costmaps, subscribed bag topics, and a measurable stationary interval before injecting faults. It captures startup diagnostics on readiness failures.
- Pose injection now distinguishes frozen translation from frozen rotation and requires independently observed motion before injecting either fault.
- The combined evidence audit measures swept-footprint containment, stationary behavior, fresh timestamps, detector disagreement at lockout, operator recovery, and HOME. It does not require nonzero Nav2 commands after a cancellation has correctly stopped the controller.
- Phase cleanup signals the ROS launch manager first. Signaling the whole launch process group killed Gazebo directly while `ros2 launch` was still active; the launch's `respawn=True` policy then started Gazebo during teardown.
- Fault-mode service calls are bounded. A delayed parameter response is accepted only after the pose relay logs that it actually applied the requested mode.

## Test results

- Full automated suite: **521 passed in 33.72 s**. Command used ROS 2 Jazzy and the isolated overlay from `stale-pose-20261121-095355`.
- Focused pose-integrity, navigation, STUCK, and watchdog tests: **180 passed in 1.88 s**.
- Isolated ROS 2 Jazzy build: **12 packages finished in 3 min 34 s**, recorded in `docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261121-095355/build.log`.
- Syntax checks: `bash -n` on the validation harness; `py_compile` on the adapter, analyzer, and pose-integrity module; `git diff --check` all passed.

## Gazebo results and limitations

The latest complete three-phase Gazebo run is `docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261121-093243/`.

| Scenario | Phase result | Evidence and limits |
| --- | --- | --- |
| Stale, delayed, and missing pose | PASS | Three pose-integrity lockouts/cancel requests; no goals while each fault was latched; explicit recovery and HOME observed. `stale-recovery/summary.json`. |
| Legitimately stationary robot | PASS | All three recorded stationary windows passed; paired streams advanced for at least 0.5 simulated seconds with zero nonzero commands. `pose-fault-audit.json`. |
| Fresh-stamped frozen translation | Phase summary PASS; combined audit INCONCLUSIVE | Lockout reported 0.052 m motion disagreement; explicit recovery and HOME observed. The audit did not yet establish the required independent post-injection forward-motion interval. `fresh-stamped-frozen-translation/summary.json` and `pose-fault-audit.json`. |
| Fresh-stamped frozen rotation | Phase summary PASS; combined audit INCONCLUSIVE | Lockout reported 0.0208 rad rotation disagreement; independent yaw range was 0.1188 rad; explicit recovery and HOME observed. The captured fresh-stamped interval was 0.884 simulated seconds, below the audit's 1 s requirement. `fresh-stamped-frozen-rotation/summary.json` and `pose-fault-audit.json`. |
| Swept entrance-boundary containment | PASS for recorded simulation intervals | No center crossing; maximum sampling gap 0.034 s; conservative swept lower bound was about `-0.5792 m` against the `-1.0 m` entrance plane in each phase. Uses the configured footprint, padding, 0.05 m uncertainty, and 1.2 m/s speed bound. This is not a hardware guarantee. |

The three phase summaries passing does **not** make the combined audit pass. The combined result in that directory is **INCONCLUSIVE**. Subsequent runs exposed harness issues and were retained separately:

- `stale-pose-20261121-094501/`: simulator setup passed, but a ROS service call did not return; this run is incomplete.
- `stale-pose-20261121-095355/`: 12-package build passed and the stale fault latched, but recovery's parameter client timed out before the relay's late `pass` transition was confirmed by the earlier harness logic. The adapter log records that transition.
- `stale-pose-20261121-100102/` and `stale-pose-20261121-100538/`: interrupted before Gazebo scenarios started; their partial build logs are preserved.

Earlier `scan_frames` shutdown exceptions were caused by the harness's intentional process replacement and the node's non-idempotent shutdown path, not by SLAM or map publication. Earlier 0.102 s pose/reference skew occurred during the deliberate missing-pose fault, exceeded the configured 0.05 s simulation pairing bound, and correctly caused a fail-closed lockout. It was not treated as a false positive.

**Final Gazebo verdict: PARTIAL.** The complete final combined audit has not passed. Do not claim the frozen-pose Gazebo suite fully validated.

## Remaining requirements and hardware acceptance

- Rerun the updated bounded harness and require its final `pose-fault-audit.json` to report `PASS`; verify fresh-stamped frozen translation and rotation independently, with their no-new-goals, acknowledgment, HOME, stationary, and swept-footprint checks.
- Team 1 must confirm the deployed independent odometry source, frames, timestamp synchronization, and the limits currently represented by simulation assumptions (0.05 s pairing, 0.5 s pose age, 0.35 s scan age, 0.05 m translation allowance, 0.02 rad yaw allowance).
- Physical acceptance still requires motor-command lease and safe-stop acknowledgment contracts, measured residual motion after cancellation, and hardware-in-loop testing. A Nav2 cancel acknowledgment is not proof that the base has stopped.

## Resume

From Ubuntu 24.04 WSL in `/home/iyed/LivingMap-Team4`, on `phase2/reliability-optimization-20261120`, with ROS 2 Jazzy and the validated Karto overlay available, run:

```bash
scripts/validate_phase2_stale_pose_gazebo.sh
```

The script builds into a fresh `/tmp/team4-pose-*` directory and preserves each run under `docs/results/evidence/team4-phase2-final-20261120/`. ROS DDS requires local UDP socket access in this environment. Do not publish; keep any follow-up commit local and authored by `Iyed Madhkour <iyedmadhkour4@gmail.com>`.
