# Task 6 — Writer exploration robustness (2026-11-17)

Initial status: **PARTIAL**. Bounded failure memory, truthful termination codes and a
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


## r3 watchdog and cooldown validation — 2026-11-17

**Overall Task 6 status: PARTIAL.** The watchdog now has a controlled Gazebo
stall result. Cooldown suppression also appeared during that mission, but the
wait-and-retry branch, attempt exhaustion, and expiry/re-arm were not triggered
in Gazebo. These remaining paths are covered by deterministic tests only.

### Controlled r3 result

Evidence is preserved at
/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-watchdog-cooldown-r3/.
The run used the isolated final ROS overlay and D3D12 settings. Independent
checks of runner-result.json, fault-injector-result.json, summary.json,
writer-status.json, published-footprints.json, git-before.json, git-after.json,
and launch.log show:

- The first frontier was (0.94, 0.13). The simulated zero-velocity fault began
  at simulation time 6.002 s. Maximum measured displacement during injection
  was 0.01734 m.
- At simulation time 30.002 s, the watchdog cancelled the goal, 24.0 simulated
  seconds after injection. The reason was recovery_limit after 26.0 s since
  progress, with Nav2 feedback reporting three recoveries and 0.366 m remaining.
  The cancellation happened before the independent 45 s physical STUCK window.
- At 30.022 s the frontier was recorded as failure attempt 1/3 with a 15 s
  cooldown. Five later frontier goals were selected outside that failure zone;
  the first was selected at 32.072 s while the cooldown was still active.
  No second attempt or cooldown-wait log occurred. After the cooldown expired,
  the failed region was no longer an eligible frontier. At 164.842 s the
  explorer reported NO_REACHABLE_FRONTIER and returned to the entrance.
- The Writer reached HOME with 70.5% battery. writer-status.json contains no
  STUCK or FAILED state. There were zero entrance-boundary violations across
  669 published footprint samples; the minimum sampled footprint x was
  -0.360003 m against the -1.0 m boundary.
- Git before/after snapshots match at HEAD 601b657 with the same four local
  modifications. The Gazebo launch and monitor exited 0. The injector exited 1
  only during cleanup: SIGINT raised KeyboardInterrupt after rclpy had already
  shut down, then an unconditional second shutdown raised RCLError.

### Injector cleanup and follow-up scenario

The original r3 files and logs are unchanged. A separate r4 helper is prepared
at
/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-cooldown-retry-r4/.
Its injector catches KeyboardInterrupt and calls rclpy.shutdown only while
rclpy.ok() is true. The runner applies min_cluster_cells=1000 as a temporary
runtime parameter, based on the archived r3 initial map where one frontier
cluster exceeded 1000 cells. It then repeats the velocity fault only for goals
within the same 1.5 m failure zone. The run is bounded and stops if another
zone is selected or if the watchdog fails to cancel by 42 simulated seconds.
The helper scripts compile successfully but were not run here because this
Codex environment lacks WSLg/GPU access and Gazebo Transport interface access.

Run from a normal Ubuntu-24.04 WSL terminal after sourcing the existing overlay:

    cd /home/iyed/LivingMap-Team4
    source /opt/ros/jazzy/setup.bash
    source scripts/_env.sh
    source /home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/install-final/local_setup.bash
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    python3 /home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-cooldown-retry-r4/run_once.py

The temporary threshold is valid only for this isolated diagnostic. Confirm
runtime-parameter-result.json says applied, each selected goal stays within the
same failure zone, failure attempts are 1/3, 2/3, and 3/3, cooldown waits report
about 15 s and 30 s, and the terminal reason is FRONTIERS_EXHAUSTED before
accepting the scenario. Confirm HOME, footprint samples, and clean injector
exit in the saved results.

### Remaining lifecycle coverage

- **Cooldown wait/retry in Gazebo:** not yet validated. r3 proves the first
  failure is suppressed while other reachable goals continue, but it did not
  wait or retry this zone.
- **Bounded attempts in Gazebo:** not yet validated. r4 targets the three
  attempts; its outcome remains NOT TESTED until run in the normal WSL session.
- **Cooldown values:** deterministic tests verify 15 s after attempt 1 and 30 s
  after attempt 2. Attempt 3 exhausts the zone, so the configured 60 s maximum
  is not reached under the three-attempt policy.
- **Exhaustion expiry/re-arm in Gazebo:** not tested. Existing policy tests
  verify the 120 s expiry, two re-arms, and re-arm after 2.0 m² known-map growth.
  After exhaustion with no alternative candidate, the current mission reports
  FRONTIERS_EXHAUSTED and returns HOME, so observing expiry in a mission requires
  another reachable frontier to keep exploration active for more than 120 s.
- The broad suite passes 321 tests with three known async ASGI tests deselected;
  the focused Writer navigation/safety suite passes 170 tests. The isolated
  source build previously finished all 12 packages. Full-suite hang details and
  logs are recorded in the checkpoint summary.

The only source/config/test edits remain the four watchdog-related files listed
in the checkpoint. The r4 helper is external evidence tooling, not production
code. No production change was made for the cooldown policy. This documentation
records partial validation; do not treat Task 6 as complete until the r4
cooldown/retry scenario is run and reviewed.

## Extended cooldown validation checkpoint — 2026-11-17

The r4 harness was reviewed but **not run** in the Codex execution sandbox.
The runner uses ROS domain 184, D3D12 environment variables, a separate runtime
and log directory, a 560 s wall limit, and only signals the process group it
starts. The injector applies zero velocity only while the Writer is navigating
within the initial 1.5 m failure zone. It aborts on 0.25 m movement, no watchdog
cancel within 42 simulated seconds, a 75 s wall-time clock stall, an unexpected
fourth goal, or selection outside that zone. The temporary
`min_cluster_cells=1000` parameter is runtime-only. This gives a guarded way to
exercise repeated failures, but the map-dependent filter does not guarantee an
initial candidate; the harness stops if it cannot isolate one. The runner also
refuses to start if prior output exists in r4 or if a service is already
listening on the dashboard's fixed `localhost:8081` port, preventing evidence
overwrite and accidental control of an unrelated dashboard.

The verified r3 result remains unchanged: one injected failure, attempt 1/3,
15 s cooldown, safe selection of five other frontiers, and HOME return at 70.5%
battery. It did **not** wait for the cooldown or retry the failed region. The
Writer had no STUCK/FAILED state and recorded zero entrance-footprint crossings
in 669 published samples (minimum footprint x -0.360003 m against boundary
-1.0 m). The helper cleanup defect was corrected in the separate r4 injector;
that correction has only passed Python syntax compilation, not runtime
shutdown validation.

Cooldown 15 s/30 s timing, three-attempt exhaustion, terminal reason, and
120 s/time or 2.0 m²/map-growth re-arm remain deterministic-test evidence only.
The broad regression run recorded 321 passed and three known hanging async API
tests deselected. `colcon test` discovered zero tests in eight ament_python
packages (exit code 5 per package); it is not evidence of package test success.
An earlier isolated build finished all 12 packages. Full r4 and expiry behavior
in Gazebo remain outstanding; overall Task 6 stays **PARTIAL**.

## Validation continuation — 2026-11-20

The original r4 harness ran safely but did not reach attempt 2: its initial
frontier was `(0.94, 0.13)` and the next candidate was more than the configured
1.5 m away. Its fail-closed guard stopped the run rather than counting a
different region as a retry. The original r3 directory was not modified.

Separate diagnostic runs used a temporary 10 m `frontier_zone_radius_m`, a
1000-cell initial `min_cluster_cells` filter, and a 1000 m² map-growth
threshold. These overrides live only in the external harness/evidence; the
production configuration remains 1.5 m / 2.0 m². The successful partial run at
`/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-20/r4-zone-10m-exhaustion/`
recorded watchdog cancellations with less than 0.04 m displacement, failures
1/3, 2/3, and 3/3 in one diagnostic zone, cooldown waits reporting 13 s and 28
s (the configured 15 s and 30 s), then `FRONTIERS_EXHAUSTED` with re-arms 0/2.

After exhaustion, the harness tried to lower the runtime cluster threshold to
5 so the Writer could explore outside the exhausted zone while the 120 s timer
ran. ROS parameter discovery returned `Node not found`; a retry variant later
timed out, and another run stopped when the injected fault exceeded its 0.25 m
movement guard. Each runner stopped its owned processes. These runs do **not**
verify the 120 s expiry or reactivation in Gazebo.

All diagnostic outcomes and temporary configuration snapshots are preserved
at `/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-20/`.
The policy's deterministic tests still cover time expiry, two bounded re-arms,
and map-growth reactivation. The full offline suite now reports **460 passed,
3 known hanging async API tests deselected**; the isolated ROS build completed
12 packages. Task 6 remains **PARTIAL** until a controlled Gazebo scenario
observes expiry/re-activation or a safe test harness for that path is prepared.

## Expiry and reactivation Gazebo follow-up — 2026-11-20

The 120 s behavior was reachable in unit tests but not in the explorer
lifecycle: after the last eligible frontier was exhausted,
`_no_candidate()` completed the sortie after four empty checks and sent the
Writer HOME before the exhausted memory could expire. The node now waits while
the candidate is covered by an exhausted zone with a re-arm still available.
Battery return, the configured sortie budget, and terminal behavior after the
two-rearm limit are unchanged. Regression tests cover waiting through expiry,
the post-rearm cooldown, candidate re-selection, and final exhaustion.

A bounded Gazebo rerun used the isolated Jazzy overlay built on 2026-11-20 and
temporary diagnostic values only: 10 m failure-zone radius, 1,000 m² map-growth
threshold, production `min_cluster_cells=5`, three attempts, 15/30 s cooldowns,
120 s expiry and two re-arms. Production defaults remain 1.5 m / 2.0 m².

- Watchdog cancellations caused failures 1/3, 2/3 and 3/3 at simulation times
  50.072, 92.032 and 152.012 s. The first two waits reported 13 s and 28 s
  remaining from the configured 15 s and 30 s cooldowns.
- The exhausted zone remained pending instead of ending the sortie. A frontier
  in that zone was selected after expiry and re-arm cooldown, 139.99 simulated
  seconds after exhaustion; a fourth fault was recorded as a fresh attempt 1/3
  at simulation time 316.112 s.
- Four watchdog cancellations were observed. Maximum injected-fault
  displacement was 0.0433 m, below the 0.25 m safety bound. The runner stopped
  after the fourth failure to avoid another retry. This run did not claim HOME;
  prior HOME and battery-return evidence remains unchanged.
- Launch and injector exited 0; the monitor was intentionally interrupted by
  the safety stop; before/after Git state matched.

The first post-fix diagnostic attempt used `min_cluster_cells=1000`; it
filtered the remaining candidates and ended with `NO_REACHABLE_FRONTIER` before
expiry. That run is preserved separately and is not counted as a pass. The
successful run used the production cluster threshold.

Evidence, separate from r3 and prior r4 runs:

- `/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-20/r4-expiry-fix-attempt-1/`
- `/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-20/r4-expiry-reactivation-pass/`

**Task 6 retry/cooldown/exhaustion/time-expiry/reactivation software scenario:
COMPLETE in deterministic tests and bounded Gazebo.** Hardware command leases
and sensor freshness remain outside this simulation result.
