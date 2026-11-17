# Task 6 — Writer exploration robustness (2026-11-17)

Status: **PARTIAL**. Bounded failure memory, truthful termination codes and a
navigation-progress watchdog are implemented, tested and exercised in three
Gazebo missions without a safety regression. The watchdog and the
cooldown-wait branch were not triggered by those missions; they are covered
only by deterministic tests. Three runs per arm cannot establish a general
reliability improvement.

Validated on Ubuntu 24.04.5 WSL2 with ROS 2 Jazzy, Nav2 1.3.13 and stock SLAM
Toolbox 2.8.5. Work stays on
`feature/team4-writer-dependencies-and-footprint-boundary`. The three baseline
missions ran at `5d10a99`; the implementation is commit `0dc7fef`. No system
package, SLAM fix or STUCK-detector change was made.

## Question and method

The question was whether the Writer can keep exploring after a transient
frontier failure without repeatedly spending the mission on the same place,
and whether it can say why exploration ended. The comparison holds everything
except the exploration code fixed:

- Six fresh missions in the same SDF world (SHA-256
  `b6b0038604cd6f9fa392a9ff40a73c2578ae0aa88b338173c669b0a75f25e74a`):
  three stock-SLAM baseline runs at `5d10a99`, three modified runs with the
  Task 6 tree.
- Isolated ROS domains and runtime directories, one mission at a time, the
  existing `full_demo` harness and unchanged mission start/return controls.
- Coverage and wall measurements from the frozen
  `coverage_reference.py` / exact-stamp method already used for Tasks 1–5;
  logs and captures are retained per run.
- Two modified launches stopped at the readiness gate and were excluded. Both
  were harness false negatives: the Firebot lifecycle manager reported all
  managed nodes active, while the observer recorded its planner as `pending`
  after its `GetState` response was lost
  (`failed to send response ... (timeout)`). Only the observer's readiness
  polling was changed to reissue a request pending for more than 3 s; the
  mission, planner and measurement paths were not changed. Both excluded
  directories are preserved as `modified-1-readiness-failure/` and
  `modified-2-readiness-failure/`.

## Reproduced baseline weaknesses

1. **"Mine mapped" was reported when it was not.** All three baseline
   missions ended with `Exploration complete: no reachable frontier left -
   mine mapped`, yet the archived final maps still contain 307, 301 and 289
   raw frontier cells respectively (recomputed with the unchanged
   `frontier_cells` function). No candidate survived the cluster-size,
   reachability, goal-clearance or visited filters. The message conflated
   "no safe reachable goal" with a fully mapped mine.
2. **A failed goal could be retried almost immediately.** In `baseline-3`,
   the goal at `(7.33, 3.86)` failed; a later goal at `(7.78, 4.26)` was
   0.60 m away, started 24.85 s later and followed one intervening goal. This
   was the only nearby retry in the six measured missions. There was no
   durable failure memory, bounded attempt count or cooldown.
3. **No mission-level progress watchdog.** Baseline relied on Nav2's own
   controller patience and recovery tree. NavFn "failed to create a plan"
   messages occurred 23, 17 and 72 times in the three baseline runs; nothing
   in the Writer measured whether the current goal was still making progress
   or bounded a silent stall.
4. **No machine-readable termination.** A single human sentence was emitted
   for every empty search, so a downstream supervisor could not distinguish a
   mapped mine, an unreachable frontier, exhausted retries, budget return or
   battery return.

## Changes by component

| Component | Change |
| --- | --- |
| `frontier_policy.py` (new) | Failure zones: 1.5 m radius, 3 attempts, 15 s then 30 s cooldown (60 s is only the cap, not reached with 3 attempts), then 120 s exhaustion. Expiry or at least 2.0 m² of known-area growth re-arms a zone, at most twice. Reached goals: 1.0 m radius, 30 s minimum age, 120 s/growth expiry, permanent after two visits. Cancellations are recorded separately and are not failures. |
| `goal_watchdog.py` (new) | Per-goal progress watchdog using Nav2 `distance_remaining` and `number_of_recoveries`, with a map-pose fallback: 0.25 m within 20 s, 10 s rotation grace, 45 s stall, 150 s goal timeout, 90 s/300 s HOME limits, 3 recoveries, 1.0 m jump reset. Never publishes STUCK or DESTROYED. |
| `frontier.py` | Configurable failure-zone radius (was hard-coded 0.6 m) and optional search statistics (`raw_frontier_cells`, cluster counts and rejection reasons). |
| `frontier_node.py` | Integrates the policy and watchdog; classifies failure/cancel/success outcomes; emits `EXPLORED`, `NO_REACHABLE_FRONTIER`, `FRONTIERS_EXHAUSTED`, `BUDGET`, `BATTERY_RESERVE` and `RETURN_FAILED`; publishes `/living_map/writer_exploration_summary`; bounds HOME retries. Also passes `frontier_visited_radius_m` through to the selector — it was declared but previously ignored. |
| `sortie.py` | Keeps the coarse `phase` and adds `reason_code`; a cooling frontier resets the empty-check counter instead of counting toward completion; repeated HOME failure sets `RETURN_FAILED`. |
| `config/nav2/writer.yaml` | Publishes the Task 6 thresholds under `/writer/frontier_explorer`; defaults match the values above. |
| tests | 32 new cases in three new files (13 policy, 15 watchdog, 4 reason-code) and 9 new cases in existing selector/sortie/navigation tests. |

The LiDAR is 360° (720 samples, ±π), so frontier goals retain yaw 0. No
heading change was warranted. The STUCK detector in `progress.py`, the SLAM
fix and all system ROS packages are untouched.

## Deterministic validation and build

- **322 tests passed, 0 failed, 0 skipped in 9.78 s** — 41 more collected
  cases than the 281 recorded at the end of Tasks 1–5.
- `scripts/build.sh` completed a clean merged build: **12 packages finished in
  35.7 s**, with CMake warnings only.
- After the build, the installed `living_map_writer` copies of
  `frontier_node.py`, `frontier_policy.py`, `goal_watchdog.py`, `frontier.py`
  and `sortie.py` hash-match the source, and installed
  `living_map_bringup/config/nav2/writer.yaml` is byte-identical to the
  repository file and contains the new parameters.
- The new reason-code integration test found the ignored visited-radius
  parameter. The fix is behavior-neutral at the default 1.0 m used by these
  missions, so no mission was rerun for it.

## Gazebo comparison

Per-run values; wall time is shown but is **not** comparable across arms
because Gazebo's real-time factor changed (see below).

| Run | Wall s | Sim span s | RTF | Frontier goals | Failed | Nearby retry | Battery at return | Path m | Known area m² | Coverage | Wall ≤10 cm | Wall RMSE | Termination |
| --- | ---: | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| baseline-1 | 212.3 | 193.13 | 0.910 | 7 | 0 | no | 78.87% | 74.42 | 214.08 | 99.5109% | 96.847% | 3.924 cm | `LEGACY_UNCODED` |
| baseline-2 | 202.3 | 182.85 | 0.904 | 6 | 0 | no | 78.20% | 78.43 | 213.45 | 99.5160% | 97.291% | 2.663 cm | `LEGACY_UNCODED` |
| baseline-3 | 182.3 | 167.22 | 0.917 | 7 | 1 | **yes** | 81.17% | 57.75 | 212.72 | 99.5092% | 90.887% | 2.522 cm | `LEGACY_UNCODED` |
| modified-1 | 242.4 | 178.01 | 0.734 | 6 | 0 | no | 78.56% | 76.99 | 213.13 | 99.4957% | 97.291% | 2.604 cm | `NO_REACHABLE_FRONTIER` |
| modified-2 | 246.4 | 198.28 | 0.805 | 7 | 1 | no | 79.63% | 72.08 | 212.70 | 99.5194% | 88.571% | 2.472 cm | `NO_REACHABLE_FRONTIER` |
| modified-3 | 192.4 | 146.62 | 0.762 | 6 | 0 | no | 85.32% | 57.19 | 212.42 | 99.5177% | 87.488% | 2.924 cm | `NO_REACHABLE_FRONTIER` |

| Metric (mean of 3) | Baseline | Modified |
| --- | ---: | ---: |
| Missions reaching HOME | 3/3 | 3/3 |
| Wall time | 198.97 s | 227.05 s |
| Simulation-clock span | 181.07 s | 174.30 s |
| Gazebo real-time factor | 0.910 | 0.767 |
| Frontier goals | 6.67 | 6.33 |
| Frontier failures | 0.33 | 0.33 |
| Nearby retries (<2 m) | 0.33 | 0 |
| Watchdog cancellations | 0 | 0 |
| Cooldown waits | 0 | 0 |
| Battery at return | 79.41% | 81.17% |
| Known area | 213.41 m² | 212.75 m² |
| Path length | 70.20 m | 68.76 m |
| Observed coverage | 99.512% | 99.511% |
| Free-classified coverage | 99.306% | 99.317% |
| Wall RMSE | 3.04 cm | 2.67 cm |
| Wall surface within 10 cm | 95.01% | 91.12% |
| NavFn no-path messages | 37.3 (median 23) | 5.7 (median 3) |
| Controller-patience errors | 0 | 4.7 |
| Published footprint crossings | 0 | 0 |
| Termination codes | `LEGACY_UNCODED` ×3 | `NO_REACHABLE_FRONTIER` ×3 |

All six runs reached HOME. Modified-2 is the only modified failure: the goal
at `(18.87, -0.31)` accumulated controller-patience errors and one
collision-ahead warning. The policy recorded attempt 1/3 with a 15 s cooldown.
No alternative candidate existed, so the explorer correctly did not wait for
that cooldown; after four empty searches it reported
`NO_REACHABLE_FRONTIER` and returned. There was no nearby retry. The three
modified completions logged raw frontier counts of 320, 336 and 260 with
no eligible goal, which is now reported truthfully instead of as a mapped
mine.

### Duration and real-time factor

The modified wall-time mean is 14.1% higher, but Gazebo ran slower: mean RTF
fell from 0.910 to 0.767. Simulation-time mean fell from 181.07 s to 174.30 s
(−3.7%), and path length per simulation second was slightly higher
(0.386 → 0.396 m/s). The wall-time increase is therefore a change in
simulation speed under load, not evidence that the new code slows the robot.
The cause of the lower RTF was not isolated; runs were sequential and no
other simulation was intentionally left running, but host load was not
controlled as an experimental variable.

## Regressions and limitations

- **Watchdog not exercised in a mission.** No modified run stalled enough to
  cancel a goal; the watchdog is covered by 15 deterministic timer/progress
  cases. The missions show that it does not false-trip on normal frontier or
  HOME driving, not that it rescues a real stall.
- **Cooldown wait not exercised in a mission.** Modified-2's failure had no
  alternative candidate, so waiting was inapplicable. The waiting branch is
  covered by deterministic policy, sortie and node tests only.
- **Small sample.** Three missions per arm, one world, one route family and
  stock SLAM. Route and candidate order vary between runs; the two arms also
  visited different frontier sequences.
- **Coverage is effectively unchanged, wall metrics are mixed.** Observed
  coverage differs by 0.001 percentage points, while wall surface within
  10 cm fell from 95.01% to 91.12% even though RMSE improved from 3.04 cm to
  2.67 cm. No SLAM code changed; the per-run spread is large (87.5–97.3% in
  the modified arm, 90.9–97.3% in baseline). This is not an accuracy
  improvement claim.
- **NavFn errors are down but not eliminated.** The median fell from 23 to 3;
  modified-2 still had 3 planner failures, 14 controller-patience errors and
  one collision-ahead warning. The bounded retry policy prevents an immediate
  repeat, but it does not make an unreachable goal reachable.
- **Reason-code classification is partly unit-tested.** `EXPLORED`,
  `FRONTIERS_EXHAUSTED` and the visited-only `NO_REACHABLE_FRONTIER` cases did
  not occur in these missions; they are covered by the new deterministic
  tests.
- **Two readiness false negatives** required rerunning modified-1 and
  modified-2; modified-3 first ran in the second suite. They are preserved,
  not hidden. The monitor patch changes only readiness polling; mission
  execution and measurement were unchanged.
- **No physical claim.** Battery, radio, dispenser and kinematics remain
  simulated. No hardware validation follows from this work.

## Evidence

- `/home/iyed/livingmap-setup/writer-task6-k3m9/` — six measured runs,
  preserved readiness failures, `task6-comparison.json`, per-run summaries,
  maps, captures and logs.
- `task6-comparison.json` records the shared world hash and the frozen
  coverage/wall results. The comparison script and analysis helpers remain on
  the host-side tooling directory, not in the repository.
- All owned Gazebo/ROS processes were stopped after the runs.

## Relationship to the STUCK detector and next priorities

This work does **not** implement, replace or claim Ahmed Chabbeh's updated
STUCK detector / PR #1; at `5d10a99` that change is not present. The new goal
watchdog is deliberately narrower: it cancels and records a navigation goal
that is not progressing, while `progress.py` remains the physical-immobilisation
detector that is allowed to declare STUCK.

Team 4 next steps:

1. Exercise the watchdog and cooldown-wait branches in Gazebo with a
   deterministic obstacle or stalled-controller scenario, not just unit tests.
2. Repeat the comparison with more missions and a controlled RTF (or compare
   only simulation-clock metrics) before claiming reliability improvement.
3. Keep the truthful `NO_REACHABLE_FRONTIER` statistics and use them to
   investigate the remaining small/unreachable clusters, especially the
   modified-2 failure region.
4. Integrate and review the teammate STUCK detector separately when it is
   available; do not conflate the two detectors.
