# Team 4 Phase 2 reliability and optimization report

Date: 2026-11-20
Branch: `phase2/reliability-optimization-20261120`
Baseline: `8714209b2045f45963fde0715815d58d76f789d4`

## Verdict

**PARTIAL for advanced validation; feasible software regression scope passes.**
The project is ready to begin a separate testing/optimization phase only if the
team accepts the documented simulation limits. The frontier selector has one
measured, behavior-equivalent optimization. No physical hardware acceptance is
claimed. Everything remains local.

## Results

- Full repository tests: **480 passed** with AnyIO 4.16.0 in 6.37 s.
- ROS 2 Jazzy isolated build: **12 packages finished**. A second isolated build
  from the STUCK harness also finished all 12 packages; its focused Writer
  suite passed **144/144**.
- Normal navigation Gazebo case: passed. It showed translation, rotation,
  zero-command intervals, route completion and HOME, with no STUCK assertion.
- Forward physical immobilization: passed. Gazebo recorded 453 forward command
  samples and 1,757 odometry samples with 0.00 m path/deviation; physical STUCK
  asserted.
- Watchdog interaction: passed. Nav2's recovery-limit watchdog canceled after
  26 simulated seconds; exploration retried; continued forward command with no
  physical displacement then independently asserted STUCK.
- External cancellation: passed. Cancellation at 2.232 s created a 15 s
  cancellation-only cooldown; failure attempts/zones remained unchanged; the
  same frontier zone was selected after 17.232 s; STUCK remained false and
  HOME was reached. The conservative 5 cm uncertainty envelope stayed inside
  the entrance boundary. Patched SLAM libraries were confirmed in the running
  process.
- Selector benchmark: on three captured Task 6 maps (15 trials, 2 warmups),
  median runtime fell 47.42%, 54.37% and 54.05%. Goal coordinates and all
  selection diagnostics were identical on each input.
- AnyIO investigation: the ASGI test hang was sandbox-specific, not confirmed as
  an AnyIO 4.16 defect. AnyIO worker-thread calls hung inside the restricted
  sandbox with versions 4.16.0, 4.15.1 and 4.9.0. A 4.9.0 direct worker probe
  succeeded outside the sandbox, and the full suite passed outside it using
  the unchanged project AnyIO 4.16.0. No app workaround or dependency change
  was retained.

## Requirement checklist

| Area | Status | Evidence or remaining condition |
| --- | --- | --- |
| Frontier selector correctness and measured efficiency | COMPLETE | Exact seeded reference checks; paired real-map benchmark and output equivalence in `evidence/team4-phase2-baseline-20261120/`. |
| External Nav2 cancellation pacing and same-zone re-selection | COMPLETE | Bounded Gazebo result and event stream in `evidence/team4-phase2-validation-20261120/external-cancellation/`. |
| Normal navigation, rotation, intentional stops and HOME | COMPLETE in simulation | STUCK harness normal-home case; preserved command/odom bag and logs. Prior HOME reserve scenarios remain preserved. |
| Forward-drive physical STUCK and watchdog independence | COMPLETE in simulation | Immobilized and watchdog-then-STUCK cases, raw bags and logs in `evidence/team4-phase2-validation-20261120/stuck/`. |
| Retry/cooldown/exhaustion/expiry/reactivation | COMPLETE in simulation | Prior Task 6 r4 evidence remains preserved under the checkpoint evidence directory; not repeated because this branch changes selector internals and pose freshness only. |
| Battery reserve and safe HOME decision policy | COMPLETE in software tests; SIMULATED previously | Full tests and existing navigation evidence; battery telemetry accuracy/return guarantee require hardware. |
| Stale/future/nonfinite pose rejection before new frontier goal | COMPLETE in unit tests; PARTIAL in Gazebo | Freshness helper and explorer regressions pass. Timestamp loss was not injected into a live Gazebo mission. |
| Boundary safety with bounded pose uncertainty | COMPLETE for tested bound | Same-zone Gazebo observer evaluated the configured 5 cm envelope and stayed inside the boundary. This is not randomized localization-noise validation. |
| Random localization noise, delayed/inconsistent localization, continuous swept footprint | PARTIAL | No seeded localization-noise model exists in the world; sampled path containment is not a proof for continuous swept volume. Requires a trustworthy sensor/localization model. |
| Unreachable goals and external cancellation | COMPLETE in software/simulation | Existing failure-budget, cooldown, and cancellation regressions plus Gazebo cancellation run. |
| Lost/ambiguous Nav2 cancel result and guaranteed stop | BLOCKED | Requires Team 1's action acknowledgment and motor command lease/stop contract; Gazebo cannot establish the physical guarantee. |
| Radio/beacon/emergency software policies and mocks | COMPLETE in software/mock tests | Existing full-suite tests and preserved Phase 1 simulation evidence; physical RF, actuation and emergency-stop acceptance remain open. |
| Sensor freshness/covariance and actual hardware interfaces | BLOCKED pending Team 1 | Need actual frame IDs, rates, timestamps, covariance, motor stop/lease, battery, radio and dispenser contracts. |
| Hardware-in-loop acceptance | BLOCKED | Requires Team 1 hardware and integrated test access. |

## Remaining blockers and evidence paths

No new software defect was established by the completed tests. The stale-pose
fallback limit (0.5 s) is a conservative software default, not a hardware
calibration. The older pose fallback is only useful in its existing frame
contract; Team 1 must confirm the transform and timestamps against the deployed
localization stack. Stale LiDAR age handling and safe stop after an ambiguous
cancel remain unresolved until sensor and motor interfaces are specified.

All current run evidence is indexed in
`evidence/team4-phase2-validation-20261120/`; earlier Task 6, battery, radio,
beacon, emergency, SLAM and navigation evidence remains untouched. Simulation
results are not physical acceptance. No remote was accessed or changed.
