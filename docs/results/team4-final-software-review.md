# Team 4 software completion review — 2026-11-20

Branch: `review/pr1-writer-stuck-detection`  
Scope: requirements closure and correctness only; no optimization work.

## Work completed in this continuation

- An unexplained external cancellation of a frontier goal now receives the
  configured base cooldown before the same spatial goal can be submitted again.
  It remains a cancellation, not evidence of an unreachable frontier: the
  failure-attempt count and exhaustion budget are unchanged. Explicit pause,
  battery, emergency, and watchdog cancellation paths retain their prior
  classifications. Regression coverage checks the cooldown and separation from
  failure memory.
- `scripts/run_demo.sh` accepts an explicit `LIVING_MAP_SLAM_OVERLAY` install
  prefix, sources it after the normal project setup, and verifies that ROS
  resolves `slam_toolbox` from that exact prefix. `--check-slam-overlay` tests
  resolution and exits before starting Gazebo. A new bounded Gazebo scenario
  also launched with the overlay active and verified the live
  `async_slam_toolbox_node`, `libasync_slam_toolbox`, `libtoolbox_common`, and
  `libkartoSlamToolbox` mappings under that isolated install. No global setup,
  `/opt/ros`, or default package selection changes.
- The three API tests initially hung inside the workspace sandbox. Their
  `asyncio.run`/ASGI lifecycle is valid: AnyIO's worker executed its function
  and queued its completion callback, but the sandboxed event loop remained
  blocked in its selector. The same AnyIO 4.16.0 passed the first test outside
  the sandbox; all three then passed together, and the full suite passed with
  no deselections. Ubuntu system AnyIO 4.2.0 showed the same sandbox symptom,
  so AnyIO 4.16.0 was not the cause. No dependency pin or application
  workaround is required. All three tests and the full suite now pass outside
  the sandbox with the normal project environment.
- The lower-passage wall gap remains narrower than the padded Writer footprint.
  Earlier preserved scan analysis supports physical viewpoint/occlusion as the
  cause of missing faces there. Reaching those surfaces would require unsafe
  geometry changes or a hardware/sensor design change; no software-only change
  can safely recover those observations. The pinned grid-extent fix addresses
  positive-edge endpoint loss and must be explicitly enabled through the
  isolated overlay.

## Verification

- Focused navigation, policy, watchdog, and detector integration regressions:
  **83 passed**.
- Full offline suite outside the workspace sandbox, with no deselections:
  **466 passed**. The three formerly excluded ASGI tests passed together:
  **3 passed**.
- External-cancel Gazebo run: **PASS**. The Nav2 cancel service reported one
  active Writer goal canceled at simulation time 3.592 s. The explorer published
  a cancellation-only cooldown until 18.592 s (exactly 15 s), with zero failed
  frontiers and zero failure zones. No same-zone goal was selected before the
  deadline. STUCK remained false, no watchdog cancellation occurred, and a
  deliberate HOME request after the deadline completed. The padded footprint's
  minimum x was -0.3600 m against the -1.0 m entrance boundary; 1,259 odometry
  samples covered 14.71 m of path.
- Isolated ROS 2 Jazzy build: **12 packages finished** in
  `/tmp/livingmap-final-20261120/`.
- SLAM overlay resolver and actual runtime checks: **passed** for
  `/home/iyed/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install`.
- Prior preserved evidence remains the basis for successful Task 5B patched
  missions, Task 6 expiry/reactivation, STUCK/watchdog integration, HOME return,
  battery reserve, and entrance-boundary behavior. No long mission was repeated
  for changes that do not alter those paths. The cancellation pacing change has
  unit/regression coverage but has not had a new Gazebo action-cancellation
  experiment.
- No physical robot/HIL test was performed. No Gazebo validation is claimed
  on physical hardware.

## Current milestone state

1. Radio, beacon persistence and placement — **PARTIAL**: software protocols,
   durable retry logic and mocks are covered; RF calibration, deployment
   acknowledgment, and dispenser behavior need Team 1 hardware contracts/HIL.
2. Battery reserve and HOME return — **PARTIAL**: simulated reserve/return and
   bounded HOME retry behavior pass; measured battery behavior and calibration
   need physical hardware.
3. Emergency/destruction handling — **PARTIAL**: software and simulated
   failure paths pass; motor-stop timing, radio loss, and restart behavior need
   HIL.
4. Navigation and entrance safety — **PARTIAL**: watchdog, bounded unreachable
   retries, cancellation pacing, HOME, footprint boundary, and rotation have
   regression/simulation evidence; noisy localization and continuous swept
   volume need additional acceptance evidence. External-cancel Gazebo evidence
   is in `external-cancel-20261120-153342` under the evidence root below.
5. SLAM — **PARTIAL**: isolated Karto fix, native 8/8 checks and three completed
   patched missions are preserved; the explicit demo overlay is now selectable.
   Lower-passage visibility, comprehensive map completeness, and physical
   localization remain open.
6. Frontier lifecycle/watchdog — **COMPLETE for validated software scenarios**:
   bounded retries, cooldown, exhaustion, time expiry, reactivation and watchdog
   interaction are covered by tests and preserved Gazebo evidence.
7. End-to-end failure scenarios — **COMPLETE in recorded simulation**: prior
   preserved missions cover immobilization, destruction, relay failure and
   HOME. This is not physical acceptance.
8. Documentation/submission — **PARTIAL**: technical reports, instructions,
   and checklists exist; final presentation/video and formal submission sign-off
   are outside software verification.

## Remaining blockers and publication

- No ASGI test blocker remains; all three and the complete suite passed outside
  the sandbox with the unchanged development environment.
- Obtain Team 1's signed hardware interfaces and a safe HIL procedure before
  physical sensor freshness, command lease, RF, battery, beacon, motor-stop, or
  dispenser acceptance.
- The Gazebo run reached the cooldown deadline and then the test requested HOME;
  it did not observe the canceled frontier being selected again after the
  deadline. The unit regression verifies that the spatial blacklist clears at
  the deadline, making the goal eligible again. Existing successful Gazebo
  evidence was preserved and not overwritten. Two harness startup/monitor
  failures (wrong action namespace, then an outdated HOME-log matcher) are
  retained in adjacent timestamped run directories; the corrected final run
  completed successfully.
- The successful external-cancel capture is
  `/home/iyed/livingmap-pr1-evidence-20261120/external-cancel-20261120-153342/`.
  Earlier diagnostic attempts remain alongside it and were not overwritten.
- All work remains local. No GitHub account, remote, PR, or branch was accessed
  or changed.
