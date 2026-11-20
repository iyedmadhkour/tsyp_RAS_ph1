# Team 4 Phase 1 and Phase 2 Completion Checklist

Date: 2026-11-20
Branch/checkpoint: `phase2/reliability-optimization-20261120` / `af4c420`
Scope: software and simulation review; no physical robot acceptance.

## Phase 1 — eight implementation-plan milestones

| # | Milestone | Status | Evidence and outstanding acceptance |
| ---: | --- | --- | --- |
| 1 | Radio, beacon placement and persistence | **PARTIAL** | Protocol, placement, persistence repair, retries, and adapter mocks have software tests. Physical RF calibration, link freshness, beacon write/release acknowledgment, and jam behavior need Team 1/HIL. |
| 2 | Battery reserve and return HOME | **PARTIAL** | Reserve policy and simulated HOME paths are covered by tests and prior missions; current stale/missing-pose phase returned HOME after recovery. Real pack telemetry, sudden voltage drop, calibrated energy budget, and physical return margin need hardware. |
| 3 | Emergency and Writer destruction | **PARTIAL** | Software state transitions, dying-gasp, late deployment gate, and simulated failure scenarios have regression evidence. Physical E-stop, motor stop, radio loss, restart and destroyed-state signal need HIL. |
| 4 | Navigation and entrance safety | **PARTIAL — frozen-pose failure open** | Existing geometry, padded footprint, entrance mask, uncertainty allowance, cancellation cooldown, and prior normal routes pass. The latest independent ground-truth test found a frozen-pose route center crossing x=-1.0 m before STUCK. Preserve the geometry and resolve the pose integrity gap before acceptance. |
| 5 | SLAM and mapping | **PARTIAL** | Isolated Karto 2.8.5 positive-edge grid correction, native checks and patched runtime missions remain validated. Lower-passage wall occlusion/map completeness and physical localization noise remain. |
| 6 | Writer exploration/watchdog lifecycle | **COMPLETE for recorded policy cases; safety acceptance PARTIAL** | Tests and preserved Gazebo r4 evidence cover retries, cooldowns, exhaustion, 120 s memory expiry, and same-zone reactivation. Watchdog and independent STUCK scenarios passed. Frozen-pose watchdog retry and boundary response now require resolution. |
| 7 | End-to-end failures and mission continuity | **PARTIAL** | Prior simulated destruction, immobilization, beacon relay, HOME and STUCK scenarios remain. Frozen-pose run did not reach HOME and crossed the boundary; physical acceptance requires motor/sensor HIL. |
| 8 | Documentation and submission | **PARTIAL** | This validation report, refreshed checklist, Team 1 interface/hardware checklist, demo recipe and evidence index are prepared. Demo video, Team 1 sign-off, hardware acceptance and formal submission sign-off remain. |

## Phase 2 — advanced safety validation domains

| # | Domain | Status | Evidence / limitation |
| ---: | --- | --- | --- |
| 1 | Missing, stale and delayed pose | **PASS in bounded simulation** | Three active-goal localization cancellations; no new goal while invalid; navigation resumed after every recovery; HOME after missing-pose recovery. Age 0.5 s and injected delay 1.5 s are test assumptions only. |
| 2 | Fresh-stamped frozen localization | **FAIL / software blocker** | 62.152 s fixed reported pose; 840 nonzero command samples; watchdog retries then STUCK; independent ground-truth center crossed x=-1.0 m. Details in final validation report and raw bag. |
| 3 | Stale LiDAR and joint pose/LiDAR freshness | **PARTIAL** | Existing tests and an earlier combined bridge outage cover stale scan and pose together. This latest test was a pose injection; no separate scan-only fault was run here. Hardware cadence is unknown. |
| 4 | Continuous footprint and boundary containment | **PARTIAL** | Stale/recovery path passed under the existing padded footprint, 5 cm allowance and 1.2 m/s simulator cap. Frozen route failed; therefore global boundary acceptance is not complete. Sample interpolation is bounded only under stated simulator assumptions. |
| 5 | Localization uncertainty/noise | **PARTIAL** | Previously preserved seeded software model: 100 cases with ±0.05 m and ±5° disturbances. No measured sensor accuracy and no stochastic Gazebo localization model. Frozen-pose fault fails as above. |
| 6 | Nav2 lost/ambiguous cancellation result | **PARTIAL** | Callback/unit fault injection verifies `NAVIGATION_UNCERTAIN` and blocks replacement goals. ROS actions and Gazebo cannot prove base motor stopping; no physical lease/ack exists. |
| 7 | Watchdog and STUCK independence | **PASS in prior isolated Gazebo regressions; PARTIAL under frozen localization** | Normal, genuine forward immobilization and watchdog-then-STUCK passed in the preserved isolated run. Frozen pose later failed containment before STUCK. |
| 8 | Battery return/HOME | **PARTIAL** | HOME reached after missing-pose recovery in latest phase and in earlier cancellation scenarios. Frozen route ended STUCK, not HOME. Real battery measurement/calibration remains hardware-dependent. |
| 9 | Frontier cooldown and failure memory | **PASS for recorded software/simulation cases** | Same-zone cancellation-only cooldown and Task 6 retries/exhaustion/120 s expiry/reactivation remain preserved. Not rerun because this validation did not change frontier policy. |
| 10 | Frontier optimization equivalence | **PASS for the recorded benchmark set** | Prior paired maps show equivalent selected goals/diagnostics and 43–54% medians on captured maps, 59–72% on seeded synthetic maps. No new optimization was performed. |
| 11 | SLAM runtime and lower-passage visibility | **PARTIAL** | Isolated Karto overlay was runtime-verified in earlier evidence. Lower-passage occlusion is unresolved and needs physical sensor/visibility evidence; production geometry was not changed. |
| 12 | Full automated regression and ROS build | **PASS** | Current full suite 506/506, no exclusions; current isolated Jazzy build 12/12. See final validation report and evidence index. |

## Acceptance decision

Team 4's software artifacts are suitable for a controlled Team 1 interface
review. **Autonomous integration and Phase 2 safety acceptance are not complete**
until fresh-stamped frozen localization is handled conservatively and the
independent boundary check passes, then physical command stop/lease and sensor
contracts are validated on HIL. No hardware acceptance is claimed.
