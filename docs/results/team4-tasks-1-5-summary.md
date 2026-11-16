# LivingMap Team 4 — Tasks 1–5 engineering summary

Validated on Ubuntu 24.04.5 WSL2 with ROS 2 Jazzy, Nav2 1.3.13 and SLAM Toolbox 2.8.5. Final checks: 2026-11-16. Work stays on `feature/team4-writer-dependencies-and-footprint-boundary`; original `main` remains `aeb012714c3ebf16262b949bf58c09c45857612b`.

| Task | Status | Implemented change / verified result | Remaining limitation |
| --- | --- | --- | --- |
| 1 — Radio and beacons | PARTIAL | Persist validated candidate records before changing inventory/sequence; preserve retry eligibility after rejected/failed relay writes; keep a durable new drop if the predecessor update fails. | Physical RF, dispenser acknowledgement/jams and automatic predecessor-link repair remain unvalidated or unimplemented. |
| 2 — Battery | PASS in simulation | Measure motor travel from odometry, so a SLAM map correction does not spend motor energy; preserve the documented HOME reserve and latched return request. | Synthetic battery model, fixed route factor and ideal odometry require hardware calibration. |
| 3 — Emergency | PASS in tested simulation | Repeated DESTROYED requests preserve the first 3-second dying-gasp deadline; block late new/rewritten Writer deployment after failure while existing beacons continue. | Restart assumes the old Writer has stopped; physical failure, radio and dispenser behavior are untested. |
| 4 — Navigation | PARTIAL | A* remains primary; only NO_VALID_PATH (208) permits Dijkstra fallback. Default recoveries, collision checks and entrance protection remain. | Some unreachable/planner failures remain; varied mission routes do not establish general reliability improvement. |
| 5 — SLAM | PARTIAL | Identified observed-but-unmapped positive-edge walls and reproduced the installed Karto extent defect; separated lower-gap visibility limitations. No production mapping change. | Needs an approved dependency fix and repeated validation; no claimed accuracy improvement. |

## Evidence by task

**Radio:** seven new failure-handling regressions. An isolated ROS service returned failure without spending stock/sequence on an injected persistence error, then succeeded on retry. Seeded synthetic RF replay delivered 198/200 good, 86/200 intermittent, 0/200 disconnected and 197/200 recovered messages. The decision threshold remains raw first-hop probability below 0.85. This is software/channel simulation, not physical RF calibration.

**Battery:** seven new regressions. A stationary 0.5 m map correction previously charged 0.18% over one second; the corrected case charges only 0.03% powered-time energy. Two 25%-start Gazebo missions reached HOME: 66.08 s before and 86.16 s after, with zero sampled entrance violations and no aborted goals. Different routes prevent an efficiency claim. An isolated live ROS probe observed exactly one latched return request across charge changes.

**Emergency:** eleven new regressions. Repeated destruction previously extended status output to 4.776 s after the first fault; the corrected actual ROS timer produced six FAILED samples, last at 2.780 s, then silence through 6.5 s. A short Gazebo test verified STUCK, successful live event recording, explicit recovery with 2.369 m movement, destruction, rejection of a late deployment, and continued existing beacon RF frames. Two synthetic Nav2 action goals were cancelled correctly. The controlled destruction scenario intentionally did not end HOME.

**Navigation:** five new pytest cases include eleven native BehaviorTree.CPP control-flow cases inside one test; four separate external native planner evidence tests also passed. On identical captured raw grid bytes, Dijkstra extracted paths in 20 of 37 replayed failure pairs where A* did not; seven were robust to every grid-index interpretation of rounded log coordinates. Published costmaps are not proven identical to request-locked grids, and the probe did not reproduce the planner's goal-tolerance search.

| Autonomous mission | HOME time | Successful / aborted outer goals | NavFn extraction failures | Sampled entrance violations |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 194.272 s | 7 / 2 | 37 | 0 |
| After 1 | 174.264 s | 8 / 2 | 75 (45 A*, 30 Dijkstra) | 0 |
| After 2 | 160.298 s | 7 / 0 | 8 (A*) | 0 |
| Final combined validation | 190.333 s | 7 / 0 | 16 (A*) | 0 |

After 1, after 2 and final captures contain 15, 8 and 16 closely paired primary-208 failures followed by successful plans. Successful branch identity is inferred from the gated tree and timing because action GetResult omits planner_id. Inner FollowPath status-6/error-0 messages during normal path replacement are not counted as failed outer goals. All four missions had no controller-patience or halt-result errors. After 2 had two transient collision-ahead warnings; final had none. Writer startup no-map and idle rescue-robot costmap warnings remain in the logs.

**SLAM:** the unchanged SDF-grounded method uses 2,030 wall samples and 59,091 footprint-safe reachable center samples, with exact-stamp TF and fixed-anchor measurements, without fitted alignment.

| Mission | Wall-point RMSE | Reference walls within 10 cm | Observed safe-center coverage |
| --- | ---: | ---: | ---: |
| Baseline | 3.622 cm | 68.621% | 99.5126% |
| After 1 | 2.411 cm | 96.995% | 99.4940% |
| After 2 | 3.337 cm | 67.192% | 99.4991% |
| Final | 2.988 cm | 97.241% | 99.5058% |

These are repeated unchanged-SLAM results, not a successful mapping-fix comparison. Most missing top/right wall samples have many raw returns within 5 cm. Native Karto reproduction confirms that positive-maximum endpoints can be indexed outside an undersized grid. Lower passage faces are a different occlusion problem; its 0.65 m gap is narrower than the padded robot's minimum 0.66 m width. Final free-classified coverage is 99.3062%, distinct from observed coverage. See [the SLAM report](team4-slam-investigation.md) for method, sensitivity, exact findings and limitations.

## Final checks and review

- **281 project tests passed, 0 failed, 0 skipped**; baseline before Tasks 1–5 was 251. Existing battery, emergency, radio/deployment, frontier, HOME and footprint regressions remain active.
- **3 packages built successfully:** `living_map_beacons`, `living_map_writer`, `living_map_bringup`. The first build command stopped on an install-layout mismatch; retrying with the existing `--merge-install` layout succeeded without cleanup. A restricted test run stalled and was terminated; the full retry with normal local access passed. Both attempts are retained.
- Final Gazebo used NVIDIA D3D12, documented `full_demo` / return controls, an isolated ROS domain and isolated runtime storage. HOME, 7 successful goals, no aborts, 575 published-footprint samples, 3,542 pose samples and 1,410 LiDAR messages were recorded. All 216 inspected raw entrance-mask points were lethal; no sampled footprint crossed the boundary. This is not proof of continuous swept-volume safety.
- Reviewed all accumulated source/test/config changes and commit contents. No known new safety regression was identified. Entrance geometry/masks, live collision checks, battery reserve and recovery actions were not weakened. No ROS message/service schema or other-team source changed. External ComputePathToPose clients must now provide a planner_id; the normal Writer tree supplies it.
- Secret-pattern scan of changed file contents in every new commit and manual diff review found no credentials. Only source, configuration, tests and concise documentation are selected; raw logs, captures, runtime stores, build/install outputs and virtual environments remain outside the commits. Pattern scanning cannot prove absence of every possible secret.
- All owned Gazebo/ROS and test processes stopped after final validation. No package installation, system/Windows/driver change, physical robot operation, destructive cleanup or history rewrite occurred. **Tasks 6 and 7 were not started.**

## Saved commits and files

Existing task commits are preserved:

- `7ab9bf0` — relay persistence and retry handling.
- `b0f05a0` — odometry-based battery travel.
- `9b031c0` — destruction timing and late-deployment gate.
- `579eaac` — planner fallback, originally saved as WIP; subsequent mission validation is recorded in `e4a3dfd` and the final reports. History was not rewritten.
- `e4a3dfd` — initial planner-fallback validation report.

Task 5 adds documentation only. The complete accumulated changed-file set relative to `c4ff6ef` is:

- `config/nav2/writer.yaml`
- `docs/results/team4-navigation-validation.md`
- `docs/results/team4-slam-investigation.md`
- `docs/results/team4-tasks-1-5-summary.md`
- `src/living_map_beacons/living_map_beacons/node.py`
- `src/living_map_writer/behavior_trees/navigate_with_planner_fallback.xml`
- `src/living_map_writer/living_map_writer/frontier_node.py`
- `src/living_map_writer/living_map_writer/node.py`
- `src/living_map_writer/package.xml`
- `src/living_map_writer/setup.py`
- `tests/native/verify_planner_fallback.cpp`
- `tests/test_writer_battery_integration.py`
- `tests/test_writer_beacon_deployment.py`
- `tests/test_writer_beacon_emergency.py`
- `tests/test_writer_beacon_failures.py`
- `tests/test_writer_emergency_reliability.py`
- `tests/test_writer_planner_fallback.py`

## Evidence and next priorities

Full local evidence: `/home/iyed/livingmap-setup/writer-five-xtouqie6/`, including reports for every task, raw maps/scans/costmaps, patches, diagnostic sources and final build/test/mission results in `final-2026-11-16/`. Git publication verification is saved there separately after the push, so this summary does not claim a push before it happens.

Personal branch: https://github.com/iyedmadhkour/tsyp_RAS_ph1/tree/feature/team4-writer-dependencies-and-footprint-boundary . Only a normal push of this branch is authorized; organizer upstream and both main branches are untouched by publication.

Priorities: obtain approval for an isolated upstream Karto extent fix and repeat wall-completeness measurements; continue investigating remaining unreachable-goal cases; validate real odometry, battery, RF and dispenser acknowledgements before hardware use. No physical safety or complete-mine-mapping claim follows from these simulations.
