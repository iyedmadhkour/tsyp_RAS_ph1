# LivingMap Team 4 Final Evidence Index

Branch: `phase2/reliability-optimization-20261120`; code checkpoint at review
start: `af4c420297974c907a84b315f075fd51618c77b7`. Evidence below is local and
preserved. Earlier runs are not overwritten by this final audit.

## Current final audit

Root: `docs/results/evidence/team4-phase2-final-20261120/`

| Evidence | Result / use |
| --- | --- |
| `stale-pose-20261120-222259/build.log` | Isolated ROS 2 Jazzy build: 12 packages finished. |
| `stale-pose-20261120-222259/final-checks/pytest-full.log` | Full unexcluded suite: 506 passed, 0 skipped/deselected. |
| `stale-pose-20261120-222259/stale-recovery/summary.json` | Stale, delayed and missing odometry; three localization cancellations; resumed goals; HOME after fresh data returned; no STUCK. |
| `stale-pose-20261120-222259/stale-recovery/launch.log`, `mode-timeline.txt`, `bag/`, `ground-truth-pose.txt` | Detailed control/event trace, fault intervals, action/odom/cmd bag, independent Gazebo trajectory. |
| `stale-pose-20261120-222259/fresh-stamped-freeze/summary.json` | Frozen reported pose reached STUCK; no HOME. |
| `stale-pose-20261120-222259/fresh-stamped-freeze/launch.log`, `fault-injection.txt`, `bag/`, `ground-truth-pose.txt` | Watchdog retries and STUCK timestamps; 62.152 s frozen stream; actual command and world-motion evidence. |
| `stale-pose-20261120-222259/pose-fault-audit.json` | Machine-readable swept-footprint results: stale/recovery path contained; frozen-pose path crossed boundary; overall INCONCLUSIVE. |
| `stale-pose-20261120-221733/` | Earlier frozen-pose attempt preserved but invalid for acceptance because fault interval marker included pre-injection motion. |
| `stale-pose-20261120-221154/`, `stale-pose-20261120-221246/` | Preserved harness setup failures (ROS shell/domain setup); not counted as tests. |

## Previously accepted Gazebo and Phase 2 evidence

| Path | Result / use |
| --- | --- |
| `docs/results/evidence/team4-ultimate-validation-20261120/final-gazebo-recovery-isolated/20261120-213332/` | Three STUCK scenarios passed: normal/HOME; forward immobilization; watchdog cancellation then independent STUCK. Includes per-case bags/logs and final checks. |
| `docs/results/evidence/team4-phase2-advanced-20261120/same-zone-swept-retry-20261120/` | Cancellation-only cooldown; same spatial frontier eligible and selected again; zero failure budget; HOME; swept-boundary observer. |
| `docs/results/evidence/team4-phase2-advanced-20261120/stale-inputs-20261120-181717/` | Combined pose/scan outage, cancellation, recovery and HOME. Boundary during stale observer gap remained explicitly inconclusive. Earlier attempts are kept separately. |
| `docs/results/evidence/team4-phase2-advanced-20261120/stuck-regression-20261120/` | Normal, forward immobility and watchdog/STUCK controlled scenarios plus focused tests/build. |
| `docs/results/evidence/team4-phase2-advanced-20261120/localization-noise-seed-20261120.json` | Seeded software-only ±0.05 m / ±5° uncertainty experiment; not physical sensor characterization. |
| `docs/results/evidence/team4-phase2-advanced-20261120/frontier-comparison-seed-20261120.json` | Paired selector benchmark for captured/synthetic maps; goal/diagnostic equivalence and performance results. |
| `docs/results/evidence/team4-master-qa-20261120/` | Prior full suite/build/coverage record at the 500-test checkpoint. |

## Earlier Team 4 evidence retained

- `docs/results/team4-task6-final-validation.md` and its referenced local
  `r4-expiry-reactivation-pass/` run: retry, cooldown, exhaustion, 120 s
  expiration and reactivation evidence.
- `docs/results/team4-phase2-baseline.md`: frontier benchmark baseline and
  prior dynamic frontier evidence.
- `docs/results/team4-slam-grid-fix.md` and `docs/results/team4-slam-investigation.md`:
  isolated Karto overlay runtime, grid correction, mission/map measurements and
  remaining lower-passage occlusion.
- `docs/results/team4-final-gazebo-recovery-20261120.md`: prior stale-input
  recovery and continuous-ground-truth audit, residual motion and action
  cancellation limits.
- `docs/results/team4-ultimate-software-validation-20261120.md`: steady-clock
  action deadlines, stale-input and action-response analysis, plus final STUCK
  regression recovery.
- Phase 1 reports and mock tests listed in `docs/results/team4-software-completion-checklist.md`;
  software mocks do not actuate hardware.

## Read this with the evidence

- `team4-final-software-validation-20261120.md` — final verdict and measured
  frozen-pose failure.
- `team4-final-phase1-phase2-checklist-20261120.md` — requirement-by-requirement
  status.
- `team4-team1-final-integration-handoff-20261120.md` — architecture, typed
  interfaces, hardware questions and HIL order.
- `team4-ros-hardware-interface-checklist-20261120.md` — Team 1 sign-off list.
- `team4-gazebo-demo-instructions-20261120.md` — reproduction commands and
  expected current failure.

All simulation findings are bounded to recorded configurations. Neither
Gazebo ground truth nor software mocks establish physical localization
accuracy, motor stopping, battery reserve, RF behavior or dispenser actuation.
