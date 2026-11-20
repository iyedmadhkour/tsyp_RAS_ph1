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
  resolution and exits before starting Gazebo. No global setup, `/opt/ros`, or
  default package selection changes. The already validated pinned overlay was
  selected successfully.
- The three API tests were reproduced under bounded timeouts. They hang in the
  current virtual environment before completing their ASGI requests. The
  environment contains AnyIO 4.16.0; its `WorkerThread.run()` processes one
  queued call, exits, and `_report_result()` returns that thread to the idle
  queue. A later Starlette thread-pool call then waits indefinitely. This is a
  resolved dependency/runtime cause, not an application assertion failure.
  `requirements-dev.txt` now caps AnyIO at `<4.16.0`. The current environment
  still has 4.16.0. A compatible wheel fetch failed because DNS/network access
  is unavailable, so the three tests remain unverified with a compatible
  resolution; they were not removed or weakened.
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
- Full offline suite, excluding only the three reproduced ASGI hangs:
  **463 passed, 3 deselected**.
- Isolated ROS 2 Jazzy build: **12 packages finished** in
  `/tmp/livingmap-final-20261120/`.
- SLAM overlay resolver check: **passed** for
  `/home/iyed/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install`; no
  Gazebo process was started by this check.
- Prior preserved evidence remains the basis for successful Task 5B patched
  missions, Task 6 expiry/reactivation, STUCK/watchdog integration, HOME return,
  battery reserve, and entrance-boundary behavior. No long mission was repeated
  for changes that do not alter those paths. The cancellation pacing change has
  unit/regression coverage but has not had a new Gazebo action-cancellation
  experiment.
- No physical robot/HIL test was performed. No Gazebo validation is claimed
  from this continuation.

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
   volume need additional acceptance evidence.
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

- Re-resolve development dependencies with AnyIO `<4.16.0`, then run the three
  ASGI tests and the complete suite without deselection. Do not run the tests
  against the current AnyIO 4.16.0 worker implementation without a hard timeout.
- Obtain Team 1's signed hardware interfaces and a safe HIL procedure before
  physical sensor freshness, command lease, RF, battery, beacon, motor-stop, or
  dispenser acceptance.
- The only Gazebo gap introduced by this continuation is a new end-to-end
  external-cancel pacing scenario; existing successful Gazebo evidence remains
  preserved and was not overwritten.
- All work remains local. No GitHub account, remote, PR, or branch was accessed
  or changed.
