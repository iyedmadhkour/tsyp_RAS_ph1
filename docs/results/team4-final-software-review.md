# Team 4 software completion review — 2026-11-20

Branch: `review/pr1-writer-stuck-detection`  
Scope: requirements closure and correctness only; no optimization work.

## Work completed in this continuation

- Added an explicit 0.05 m bounded entrance-position uncertainty parameter.
  Frontier goal centers include that allowance in addition to the padded
  footprint's all-heading radius and Nav2 goal tolerance. The Writer-only
  lethal entrance mask is shifted inward by the same amount. Shared SLAM data
  and robot footprint dimensions are unchanged. Tests cover the combined
  worst-case position error, goal tolerance, all robot headings, and mask
  rasterization. The 5 cm bound is a software acceptance assumption; Team 1
  must confirm it against measured localization behavior before hardware use.
- Added a preconnected validation controller so the first frontier goal can be
  canceled before the Writer moves far enough to map the candidate away. The
  bounded Gazebo scenario holds the Explorer and map snapshot through the
  configured cancellation-only cooldown, then verifies the same spatial zone
  is selected after expiry. The diagnostic variants where cancellation came
  later are preserved and not counted as passes.

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
  **146 passed** after adding the uncertainty envelope and entrance-mask tests.
- Full offline suite with the modified Writer source explicitly first on
  `PYTHONPATH`: **468 passed**, no deselections. The three previously hanging
  ASGI tests pass outside the sandbox with the unchanged AnyIO environment.
- Controlled external-cancel Gazebo run: **PASS**. The first frontier goal was
  canceled at simulation time 2.232 s; its cancellation-only retry deadline
  was 17.232 s (15.000 s later), with zero failure zones and zero failure
  attempts. After expiry, the same zone was selected again. STUCK remained
  false, the watchdog did not cancel, and the battery-reserve path reached
  HOME. The minimum padded footprint x was -0.3600 m; applying the configured
  0.05 m worst-case x-pose error gives -0.4100 m, still 0.59 m inside the
  x=-1.0 m boundary. The result includes 929 odometry samples. Evidence:
  `docs/results/evidence/team4-same-zone-retry-20261120/`.
- Isolated ROS 2 Jazzy build during the final Gazebo run: **12 packages
  finished** in `/tmp/livingmap-pr1-evidence-20261120/external-cancel-20261120-163548/`.
- SLAM overlay resolver and actual runtime checks: **passed** for
  `/home/iyed/livingmap-setup/writer-slam-5b-9qc5m2a6/overlay/install`.
- Prior preserved evidence remains the basis for successful Task 5B patched
  missions, Task 6 expiry/reactivation, forward-drive STUCK/watchdog integration,
  battery reserve, and HOME behavior. The current Gazebo run also covers
  cancellation pacing, same-zone recovery and the bounded entrance-pose-error
  envelope; it does not simulate stochastic SLAM localization error.
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
4. Navigation and entrance safety — **COMPLETE for software scenarios;
   hardware acceptance PARTIAL**: watchdog, retry budget, cancellation pacing,
   same-zone post-cooldown selection, battery-triggered HOME, rotated footprint
   boundary and a 5 cm pose-error envelope have test/simulation evidence.
   Stochastic localization and continuous swept-volume acceptance remain open.
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
- The same-zone Gazebo requirement now passes under a frozen map/pose snapshot;
  this controlled setup isolates cooldown eligibility and is not a claim that
  an evolving frontier map must retain the same candidate during ordinary
  exploration. Failed setup attempts and earlier successful runs remain
  preserved in their original locations.
- The 5 cm position-error bound has not yet been confirmed against Team 1's
  physical localization accuracy. Stale sensor/pose freshness thresholds,
  command leases, RF and measured-battery interfaces likewise await the signed
  hardware data rates and safety contracts. A lost Nav2 cancel result also has
  no validated recovery policy: safely deciding whether to retry, halt or
  return depends on action-server and motor-command lease semantics. No evidence
  of stochastic SLAM localization or continuous swept-volume acceptance is
  claimed.
- The final controlled Gazebo run is under
  `/tmp/livingmap-pr1-evidence-20261120/external-cancel-20261120-163548/`; key
  logs and results are preserved in the repository evidence directory.
- All work remains local. No GitHub account, remote, PR, or branch was accessed
  or changed.
