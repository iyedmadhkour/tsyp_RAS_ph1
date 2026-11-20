# LivingMap Team 4 — Final Software Validation

Date: 2026-11-20
Branch: `phase2/reliability-optimization-20261120`
Validated code base: `af4c420297974c907a84b315f075fd51618c77b7`
Validation status: **PARTIAL — regression suite/build pass; frozen-pose Gazebo safety check fails**

## Verdict

The repository's full automated suite passed (506/506), and an isolated ROS 2
Jazzy build finished all 12 packages. The previously completed normal/HOME,
forward-immobilization, and watchdog-then-STUCK Gazebo scenarios remain valid
and are preserved. The new dedicated stale/delayed/missing-pose scenario
reached HOME after the localization gate canceled three active goals. The
fresh-stamped frozen-pose phase did reach STUCK, but the independent Gazebo
ground-truth audit found the physical model crossed the configured entrance
plane before STUCK. This is a failed safety scenario, not a full simulation
acceptance. No geometry or limits were relaxed to make it pass.

No physical robot or Team 1 hardware was available. No publication, remote,
GitHub, or deployment operation was performed.

## Latest validation results

| Check | Result | Evidence |
| --- | --- | --- |
| Full repository tests | **PASS: 506 passed, 0 failed, 0 skipped, 0 deselected; 5.73 s** | `evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/final-checks/pytest-full.log` |
| Isolated ROS 2 Jazzy build | **PASS: 12 packages finished; 26.9 s** | `evidence/team4-phase2-final-20261120/stale-pose-20261120-222259/build.log` |
| Validation scripts | **PASS: Python compilation and shell syntax** | Final-check command output in session; source scripts are in `scripts/` |
| Stale / delayed / missing pose, recovery and HOME | **PASS within the stated simulation assumptions**: three active-goal localization cancellations; goal dispatch resumed after each recovery; HOME observed after missing-pose recovery; no false STUCK in this phase | `.../stale-pose-20261120-222259/stale-recovery/summary.json`, `mode-timeline.txt`, `launch.log`, bag and independent ground-truth stream |
| Fresh-stamped frozen pose | **PARTIAL / safety FAIL**: frozen odometry remained exactly fixed for 62.152 sim seconds while 840 nonzero commands were observed; watchdog canceled/retried goals; STUCK asserted; physical ground truth crossed x=-1.0 m entrance plane | `.../fresh-stamped-freeze/summary.json`, `fault-injection.txt`, `launch.log`, `bag/`, `ground-truth-pose.txt`, `pose-fault-audit.json` |
| Continuous swept footprint over stale/recovery route | **PASS within the simulator assumptions**: 1,042 poses, max 34 ms gap, conservative minimum x=-0.5792 m against x=-1.0 m | `pose-fault-audit.json`, phase `stale-recovery` |
| Continuous swept footprint over frozen-pose route | **FAIL**: 1,957 poses, max 34 ms gap; center minimum x=-1.3169 m; conservative padded/uncertainty lower bound x=-1.8960 m against x=-1.0 m | `pose-fault-audit.json`, phase `fresh-stamped-freeze` |
| Earlier normal/HOME, forward immobilization, watchdog then independent STUCK | **PASS in bounded Gazebo runs** | `evidence/team4-ultimate-validation-20261120/final-gazebo-recovery-isolated/20261120-213332/` |
| Frontier same-zone cancellation cooldown and HOME | **PASS in bounded Gazebo** | `evidence/team4-phase2-advanced-20261120/same-zone-swept-retry-20261120/` |

The final build uses the validated local Karto overlay recorded in each run
manifest. Existing performance comparisons and all earlier failure evidence
remain unchanged.

## Frozen-pose finding

The live test relay held the Writer's reported odometry pose fixed while
refreshing its timestamp. During the 62.152-second injected interval it
observed 1,829 fresh-stamped odometry samples, zero reported position range,
915 command samples (840 nonzero), and 1,957 independently captured Gazebo
poses. Navigation watchdog cancellations occurred and goals were retried;
the independent STUCK detector eventually asserted. However, the model's
ground-truth center reached x=-1.3169 m relative to the spawn registration,
past the configured x=-1.0 m entrance plane. The robot was already displaced
0.7612 m from the spawn registration when the freeze began and ended at
x=-1.1284 m. The analyzer's full padded-footprint and 5 cm uncertainty bound
was still more negative. HOME was not observed in this phase because the robot
entered STUCK.

This test establishes that fresh timestamps alone do not identify a frozen
pose estimate. It also shows the cost of waiting for existing watchdog and
STUCK policy outcomes when the reported pose can be decoupled from physical
motion. It does **not** establish a physical robot outcome: the Gazebo relay
artificially disconnected the Writer's localization stream from ground truth.
The current system must not be accepted for autonomous motion under a
fresh-stamped-but-frozen localization source until a sensor-motion consistency
contract and conservative response are implemented and validated. Choosing a
new production timeout without Team 1's sensor and motor behavior would invent
a hardware timing requirement; therefore no uncalibrated production threshold
was added.

The old attempt `stale-pose-20261120-221733` is preserved and remains
INCONCLUSIVE because its frozen interval marker included pre-fault motion. The
corrected run is `stale-pose-20261120-222259`; it now aligns the start marker
with the adapter's `POSE_FAULT_MODE=frozen` acknowledgment. Its failure is
retained as evidence. The test adapter shutdown path was made idempotent after
this run; the change affects teardown only and was syntax-checked, not
re-simulated.

## Completion boundary

- **Verified software requirements:** the 506-test checkout, 12-package ROS
  build, goal blocking/cancellation on stale, delayed and missing pose,
  recovery to navigation and HOME after fresh pose returns, previously passing
  STUCK/watchdog/HOME scenarios, and swept containment on the stale/recovery
  route.
- **Implemented but incompletely validated:** live sensor-fault handling as a
  whole. The frozen-pose run exposes an unsafe behavior in the combined
  navigation/entrance-safety scenario and is not accepted.
- **Software blocker:** fresh-stamped frozen localization is not detected by
  the pose-age gate; during retrying navigation the simulated model crossed
  the entrance plane before STUCK. Resolve only with an agreed motion-feedback
  contract, safe command-stop behavior, and regression acceptance; do not hide
  this run or reduce the real footprint/boundary.
- **Team 1 / hardware blockers:** encoder and fused-localization contracts,
  physical localization integrity/freshness, command lease and independent
  motor stop acknowledgment, measured stopping distance, battery telemetry,
  RF/beacon acknowledgment, emergency-stop and physical STUCK/DESTROYED
  behavior, followed by HIL acceptance.
- **Integration readiness:** Team 4's software interfaces and regression
  artifacts are ready for a controlled interface review, but autonomous
  physical integration is not safety-accepted while the frozen-pose scenario
  remains open.

See `team4-final-phase1-phase2-checklist-20261120.md`,
`team4-team1-final-integration-handoff-20261120.md`,
`team4-ros-hardware-interface-checklist-20261120.md`,
`team4-gazebo-demo-instructions-20261120.md`, and
`team4-final-evidence-index-20261120.md`.
