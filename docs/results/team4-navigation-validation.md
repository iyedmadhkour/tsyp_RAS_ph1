# Task 4 — navigation validation (2026-11-15)

Status: PARTIAL. The bounded planner fallback passes its functional and safety checks; a general mission-reliability improvement is not established by three variable-route runs.

The original A* planner can fail to extract a path despite legal potential. Native installed NavFn replay on 37 logged failures and actual published raw costmaps found 20 cases where Dijkstra extracted a path on the same bytes; seven remained valid across all log-coordinate rounding possibilities. Published snapshots are not proven identical to the request-locked costmap, and replay did not simulate goal-tolerance search.

Commit 579eaac is now reviewed and validated as a bounded mitigation. Writer frontier and HOME goals use a packaged copy of the installed Nav2 recovery tree. Only the planning leaf changes: after NO_VALID_PATH (208), attempt Dijkstra once before the unchanged recoveries. All entrance masks, padded footprints, controller collision checks, unknown-space and tolerance policies remain intact. No controller tuning was justified.

| Metric | Baseline | After 1 (recovered) | After 2 (new repeat) |
|---|---:|---:|---:|
| HOME elapsed seconds |194.272|174.264|160.298|
| Outer navigation successes |7|8|7|
| Outer navigation aborts |2|2|0|
| NavFn extraction failure logs |37|75|8|
| Controller-patience / halt-result warnings |0/0|0/0|0/0|
| Sampled footprint violations |0|0|0|

After1 includes45 primary and30 fallback failure logs;15 closely paired primary208→success results support successful fallback recovery. GetResult has no planner_id, so successful-branch attribution is a timing/tree inference. Inner FollowPath status6/error0 during path replacement is not counted as an outer failed mission. The extra fallback can increase warning counts; it cannot make unreachable goals reachable. Different routes preclude a causal speed claim. Baseline observed safe-center coverage99.5126%, after1 99.4940%; full fixed-denominator comparison is in external resumed-metrics.

The already completed Writer+bringup build passed;281 automated tests passed, including11 native BehaviorTree.CPP scripted control-flow cases inside one pytest test. Four separate native raw-costmap evidence tests passed. These completed checks were preserved rather than rerun before the unchanged repeat. Both after missions reached HOME with NVIDIA D3D12 and zero sampled padded/published footprint violations. After2 runner exited0 and all owned ROS/Gazebo processes stopped. Sampling does not prove continuous swept-volume containment.

Changed files in579eaac: config/nav2/writer.yaml; src/living_map_writer/living_map_writer/frontier_node.py; src/living_map_writer/behavior_trees/navigate_with_planner_fallback.xml; src/living_map_writer/package.xml; src/living_map_writer/setup.py; tests/test_writer_planner_fallback.py; tests/native/verify_planner_fallback.cpp. No further source fix was needed in this review.

Limitations: external ComputePathToPose clients must explicitly specify planner_id now that two plugins exist. Selecting Dijkstra itself as primary can redundantly retry it; documented default is GridBased A*. Neither NavFn variant reasons about orientation. Unreachable goals and unseen surfaces remain separate investigation targets. Startup no-map and idle rescue-robot warnings remain recorded; no unsafe suppression was introduced.

Evidence: /home/iyed/livingmap-setup/writer-five-xtouqie6/task4, including baseline/, after-1/, after-2/, native-baseline-final/, resumed-review.md and resumed-metrics/. Raw logs/maps stay outside Git.
