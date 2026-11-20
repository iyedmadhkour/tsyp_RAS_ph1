# Team 4 Phase 2 reliability and optimization baseline

Date: 2026-11-20
Baseline commit: `8714209b2045f45963fde0715815d58d76f789d4`
Experimental branch: `phase2/reliability-optimization-20261120`

This baseline reuses successful Phase 1 runs and adds a repeatable selector
benchmark on captured occupancy grids. Existing evidence is read-only and
unchanged. No physical hardware test is represented here.

## Environment and reproducibility

- Ubuntu 24.04.5 LTS in WSL2; kernel `6.18.58.1-microsoft-standard-WSL2`.
- ROS 2 Jazzy, `rclpy 7.1.12`; Python 3.12.3; pytest 8.4.2; AnyIO 4.16.0.
- Gazebo Sim 8.15.0; Nav2 bringup/planner 1.3.13; ROS-Gazebo Sim 1.0.24;
  system SLAM Toolbox 2.8.5. The validated SLAM grid fix uses the separate
  pinned 2.8.5 overlay referenced by `team4-slam-grid-fix.md`.
- Machine CPU: Intel Core i7-13650HX, 20 logical CPUs. Previous renderer
  verification recorded NVIDIA D3D12; NVML was unavailable to this read-only
  baseline command.
- Gazebo mine world SHA-256:
  `150cb9c548d70af28cb42ba8da83f8a1e41f3b528b87f8ab4799d80a6bccdb29`.
- Writer Nav2 config SHA-256:
  `69461f53bceea11a231ce8a6e25b17e35186622bb3fa5336e6e820de35d769da`.
- SLAM config SHA-256:
  `7d6f8934793b9c2a2746ddf20308993918190f66761ddea95ca2c4e2e414d10e`.
- The Gazebo world/config does not specify a randomized sensor or physics seed;
  the baseline simulator uses its fixed world and ideal physics odometry. Thus
  no random seed applies to these missions. The RF unit test has an explicit
  seed of 42. Pose-error assumptions are modeled separately and are not
  stochastic SLAM localization.

To repeat the algorithm baseline from the repository root:

```bash
source scripts/_env.sh
export PYTHONPATH="$PWD/src/living_map_writer:$PYTHONPATH"
python3 scripts/benchmark_frontier.py --trials 15 --warmup 2 \
  --output /tmp/frontier-repeat.json
```

The benchmark inputs are three captured maps from Task 6 r4 at simulation
times 31.832, 159.832 and 306.022 s. Each `.npz` file is preserved beside
`maps.json`, which records its hash, shape, resolution, exact map-to-odom
alignment and interpolated robot pose. The selector uses the production
footprint/boundary parameters and an empty frontier memory so every timing
uses an equivalent input.

## Existing navigation, mapping and mission measurements

- Navigation comparison in `team4-navigation-validation.md`: HOME times were
  194.272, 174.264 and 160.298 s for the original baseline and two fallback
  repeats. The final combined run took 190.333 s, completed seven outer goals,
  had no outer aborts and no sampled entrance-footprint crossings. Routes
  differed; the timings do not establish a causal speed improvement.
- Patched SLAM comparison in `team4-slam-grid-fix.md`: three completed patched
  missions retained all 608 positive-edge samples within 10 cm; mean wall
  completeness was 97.044% versus 87.143% for stock. Mean observed floor
  coverage was 99.5064% versus 99.4996%. Mean HOME time increased from
  174.965 s to 228.401 s. The simulator uses ideal odometry; localization
  improvement is not claimed.
- Task 6 r4 failure-memory run: watchdog canceled four injected immobilization
  episodes; maximum displacement during a fault was 0.0433 m. Attempts
  advanced 1/3, 2/3, 3/3, then reactivated after 139.99 simulated seconds and
  attempted again. The harness intentionally stopped after capturing that
  post-expiry failure; HOME was not claimed for that run.
- Same-zone cancellation baseline: the first goal was externally canceled at
  2.232 simulated seconds. The 15 s cancellation-only deadline was 17.232 s;
  failure-zone count remained zero. The same spatial zone became selectable
  after expiry, STUCK remained false and the battery reserve path reached HOME.
  Its 929 odometry samples traversed 1.195 m. Evidence is in
  `evidence/team4-same-zone-retry-20261120/`.
- Full automated baseline at the prior checkpoint: 468 tests passed, no
  deselections; all 12 ROS packages built. Existing STUCK, watchdog, battery,
  emergency, beacon mock and Task 6 evidence is catalogued in the completion
  checklist and failure matrix.

## Frontier-selector baseline

`frontier-baseline.json` records all trial timings, selected goals, map hashes,
configuration hash, source commit and selector counters. With two warmups and
15 trials, measured medians were:

| Captured grid | Cells | Occupied cells | Median | p95 | Selected goal (map frame) |
| --- | ---: | ---: | ---: | ---: | --- |
| 284 × 194 at 31.832 s | 55,096 | 441 | 43.07 ms | 50.04 ms | (2.407, -2.074) |
| 371 × 195 at 159.832 s | 72,345 | 1,115 | 83.81 ms | 88.54 ms | (10.620, -3.199) |
| 477 × 195 at 306.022 s | 93,015 | 2,185 | 158.99 ms | 170.70 ms | (7.320, 3.801) |

These are single-process CPU selector timings, not end-to-end navigation
latencies. On the largest real map, a 159 ms median consumes about 8% of the
explorer's 2 s tick. The paired `cProfile` run attributes most cumulative time
to obstacle-footprint dilation (`set.update`) and frontier-cell scanning. This
is the only optimization target selected so far; any implementation change
must keep all three selected goals and selector diagnostics unchanged, then
show repeatable latency reduction and retain the full safety/regression suite.

## Known baseline gaps

- No stale LiDAR/odometry age gate or measured pose covariance is defined; Team
  1 update rates and timestamps are unavailable. The 5 cm entrance allowance
  is an explicit test assumption pending measured hardware error.
- A lost Nav2 cancellation result has no validated recovery policy. A safe
  response depends on the action-server and motor-command lease contract.
- Current localization evidence is bounded-error analysis, not injected
  stochastic noise. Existing entrance checks sample poses; continuous
  swept-volume containment has not been proven.
- Physical radio, beacon dispenser, battery telemetry, emergency motor stop
  and hardware-in-loop acceptance are not available in Gazebo.
- The lower-passage wall occlusion is constrained by the 0.65 m passage and
  padded footprint; no geometry weakening is allowed to improve coverage.

## Phase 2 validation and measured change

Date: 2026-11-20. Source is on the separate local branch
`phase2/reliability-optimization-20261120`; no GitHub operation was performed.

- The frontier selector now stores circular obstacle dilation in per-row bit
  masks and evaluates its four frontier neighbors directly. Seeded reference
  tests cover map-edge clipping, occupancy values and several clearance/raster
  ratios. On the same three captured maps, 15 trials after 2 warmups kept each
  selected goal and all selector diagnostics identical. Median latency fell
  from 43.07 to 22.65 ms (47.42%), 83.81 to 38.24 ms (54.37%), and 158.99 to
  73.05 ms (54.05%). Input map hashes and selector-relevant config hash are
  identical across paired runs. See `evidence/team4-phase2-baseline-20261120/`.
- Pose selection now rejects missing, stale, future-dated and non-finite map
  transform/odometry timestamps; the configured odometry fallback age is 0.5 s.
  If an active goal loses fresh pose observations, its watchdog retains the last
  observed position rather than treating missing data as progress. Seven
  timestamp cases, stale-odometry fallback behavior, no-goal-on-stale-pose and
  active-watchdog behavior are regression-tested. This age limit has not been
  calibrated against Team 1 sensor timing and covariance.
- Full repository pytest: **480 passed in 6.37 s** with the project environment's
  AnyIO 4.16.0. Tests had hung when launched inside this restricted sandbox.
  A direct AnyIO worker diagnostic reproduced that sandbox-only behavior with
  4.16.0, 4.15.1 and 4.9.0; AnyIO 4.9.0's worker completed outside the sandbox,
  and the full suite completed outside it under 4.16.0. No package pin or app
  workaround was made.
- Isolated ROS 2 Jazzy build: **12 packages finished**. The STUCK harness also
  built all 12 packages and ran 144 focused tests (all passed).
- Bounded Gazebo STUCK regression: normal navigation completed its route and
  HOME with translation, turns, zero-command intervals and no STUCK; a static
  Writer produced 453 forward command samples with zero odometry displacement
  and correctly reached physical STUCK; in the watchdog case, Nav2 canceled
  after the recovery limit at 26 s, exploration retried, and physical STUCK
  independently asserted after continued forward command without displacement.
  All three scripted cases passed.
- Bounded same-zone external cancellation: canceled at simulation 2.232 s,
  cooldown deadline 17.232 s (15 s), no failure attempt or failure zone was
  consumed, and the exact same frontier zone was selected after cooldown.
  STUCK remained false; the robot reached HOME. The sampled padded footprint
  minimum x was -0.36 m and its conservative 5 cm uncertainty envelope was
  -0.41 m, both on the safe side of the -1.0 m entrance boundary. 1,097
  odometry samples were collected. Patched SLAM libraries were verified in the
  running process. See `evidence/team4-phase2-validation-20261120/`.

## Remaining advanced validation limits

- The Gazebo world has no seeded stochastic localization/odometry-noise model.
  The 5 cm entrance allowance was evaluated as a bounded uncertainty envelope,
  not as randomized localization error. Delayed and missing-pose behavior has
  unit-level coverage, but timestamp loss was not injected into a running
  Gazebo mission.
- Entrance containment evidence is sampled odometry at the observer rate;
  continuous swept-footprint containment during arbitrary turns has not been
  proven. Do not interpret sampled checks as a formal geometric guarantee.
- Lost or ambiguous Nav2 cancellation results and safe motor stopping still
  depend on Team 1's action-server and motor-command lease/acknowledgment
  contract. There is no simulator evidence for that physical interface.
- Sensor frame/rate/covariance, battery telemetry, radio acknowledgments,
  beacon actuation, emergency motor stop and HIL acceptance require Team 1's
  hardware. Simulation passes do not constitute hardware acceptance.
- The lower-passage wall occlusion and the documented HOME-time variation
  remain mapping/navigation trade-offs, not corrected defects. No footprint,
  boundary or reserve parameters were relaxed.
