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

### Follow-up bounded rerun from `588e973`

The run `docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261121-141326/`
completed its isolated ROS 2 build (12 packages), then failed during the first
scenario's startup readiness check: `/writer/world_odom` did not produce a
message within the harness's existing 20-second bound. The harness exited
before fault injection. Its `launch.log`, `world-odom-ready.yaml`, and
`startup-*` diagnostics are retained. This is a Gazebo setup/readiness failure,
not evidence for or against the pose-integrity scenarios. The combined result
therefore remains **INCONCLUSIVE**.

The run did not establish a cause for the missing world-odometry message. No
threshold or assertion was relaxed, and no additional rerun was attempted after
the execution environment stopped accepting shell processes. The earlier
complete `stale-pose-20261121-093243/` evidence remains intact and inconclusive
for the reasons recorded below.

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

**Checkpoint verdict at the time of this report: PARTIAL.** The complete final combined audit had not passed at that point. The follow-up result below supersedes that checkpoint verdict.

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

## Follow-up validation completion — 2026-11-21

### Root causes and corrections

- The `scan_frames` tracebacks in earlier runs followed the harness's deliberate `SIGTERM` replacement of that node. It was a test setup transition, not a spontaneous SLAM or Gazebo crash. The relay now replaces `scan_frames` from launch startup (`scan_frames:=false`), so there is one Writer pose publisher throughout each test. The normal demo launch still starts `scan_frames` by default.
- The 0.102–0.136 s estimate/reference skews occurred during the harness's publisher handoff. The safety gate correctly latched when the configured 0.05 s pairing limit was exceeded. The final run uses the single relay from startup; the full-run map/odom alignment stayed within 0.010 m translation and `2.42e-7` rad yaw (pre-fault maximum yaw was `5.1e-10` rad).
- In the failed GUI startup, the Gazebo topic list showed bridge-created topics but no model odometry publisher, and the GUI waited for world initialization. A bounded server-only probe loaded the local world and exposed Writer world odometry. The launch now has a headless option, used only for WSL validation; the GUI remains enabled by default.
- Startup snapshots also showed orphaned ROS processes from prior temporary validation runs and a duplicate command-post bind on port 8081. Cleanup now drains the launch's private process group even if `ros2 launch` exits first. The final run started without stale validation processes or that port conflict.
- A production safety gap was confirmed: the fast monitor returned while no action was active, so stale pose/reference data between goals did not enter the sticky uncertainty state. It now latches after at least one synchronized pair has been established. Before the first pair, goal readiness remains closed without turning normal startup into a sticky fault. Regression tests cover both cases.
- The first combined audit of the complete final run was marked INCONCLUSIVE by an auditor scoping bug: it applied the rotation criteria to the translation phase. The phase-specific checks are now isolated in `_frozen_phase_pass`, with a regression test. The original inconclusive file is preserved; the corrected audit was rerun against the same recorded Gazebo evidence and passed.

### Final bounded Gazebo results

Run: `docs/results/evidence/team4-phase2-final-20261120/stale-pose-20261121-211121/`.

| Scenario | Result and evidence |
| --- | --- |
| Legitimately stationary startup | PASS in all three phases; synchronized estimate/reference streams advanced for 0.714–0.780 simulated seconds with zero nonzero commands. See each `stationary-window.txt` and `pose-fault-audit-corrected.json`. |
| Stale, delayed, and missing pose | PASS: 3 lockouts, 3 cancellation requests, no new goals during lockout, valid-stream recovery, explicit acknowledgement, and HOME. See `stale-recovery/summary.json` and `mode-timeline.txt`. |
| Frozen translation with fresh stamps | PASS: independent forward motion 0.0397 m before injection; 0.051 m disagreement latched within 0.03 simulated seconds; no new goal while locked; acknowledged recovery and HOME. |
| Frozen rotation with fresh stamps | PASS: injection began during an observed in-place angular command (`angular_z=-0.899`); independent pre-fault yaw change was 0.1656 rad; 0.0376 rad disagreement latched within 0.02 simulated seconds; acknowledged recovery and HOME. Independent reference yaw range during the recorded interval was 0.1034 rad. |
| Continuous padded-footprint entrance boundary | PASS for all recorded intervals: 978 rotation, 879 translation, and 1,607 stale/recovery ground-truth samples; largest gap 0.034 s; no center crossing. The conservative swept lower bound was `-0.5792 m` against the `-1.0 m` entrance plane, leaving about 0.421 m simulated margin. This used the configured footprint, padding, 0.05 m uncertainty allowance, and 1.2 m/s bound. |
| Combined corrected pose-fault audit | **PASS**. The source audit initially wrote `pose-fault-audit.json` as INCONCLUSIVE because of the phase-selection bug; that failed result is preserved. After the fix, the same bag and observations produced PASS in `pose-fault-audit-final.json` and `.log`. |

The bounded run also completed an isolated ROS 2 Jazzy build: **12 packages finished** (`build.log`). The focused pose-integrity/navigation/STUCK/watchdog/audit tests passed **121 tests** (`pytest-focused-final.log`); the full suite passed **524 tests** (`pytest-full-final.log`).

### Safety limits and verdict

The original frozen-pose boundary crossing was **not reproduced** in the final bounded scenarios. Independent Gazebo ground truth showed lockout before any center crossing and conservative swept-footprint containment in all three phases. This demonstrates the configured fail-closed response under these simulation conditions; it does not establish physical stopping distance or a hardware motor-stop guarantee.

Pose age (0.5 s), scan age (0.35 s), injected delay (1.5 s), 0.05 s pair skew, 0.05 m translation agreement, 0.02 rad yaw agreement, and the 1.2 m/s speed bound remain simulation assumptions. Team 1 must confirm actual independent odometry, frames, timestamp behavior, limits, motor command lease/stop acknowledgements, battery behavior, and hardware-in-loop acceptance. No physical robot test was performed.

**Latest verdict: PASS for the bounded frozen-pose Gazebo acceptance and the applicable software regression suite.** The work is ready for local review and controlled Team 1 integration planning, but not physical robot acceptance. No GitHub publication was performed.
