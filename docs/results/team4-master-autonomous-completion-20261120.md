# LivingMap Team 4 — Master Phase 2 Completion and Final QA

Date: 2026-11-20  
Branch: `phase2/reliability-optimization-20261120`  
Validated source checkpoint: `d6ae84bda9f9f598e47aafb250c2c20b46362d20`  
Starting checkpoint: `7e5ac45a00457ff5c34ddfe7baed34ebdfbac9a6`

## Executive verdict

**PARTIAL overall.** The software and bounded simulation scope described below
passes its applicable regression suite and isolated ROS build. Recorded Gazebo
scenarios demonstrate selected navigation, stale-input, retry, HOME, watchdog,
STUCK and footprint properties. They do not establish complete physical
safety, acceptance of real sensor performance, or a guarantee that a Nav2
cancellation stops the motors. Several Phase 2 domains remain
hardware-blocked or incompletely exercised in live Gazebo. No physical robot
was deployed and nothing was published.

## Completion dashboard

| Area | Status | Evidence-backed conclusion |
| --- | --- | --- |
| Phase 1 software workflows | **COMPLETE within recorded software/simulation scope** | Existing requirements, protocol, mock and mission evidence remain indexed in `docs/results/` and earlier evidence directories. Physical interfaces are not accepted. |
| Phase 2 software implementation | **COMPLETE for the validated behaviors; PARTIAL overall** | Freshness, conservative action-state handling, retry policy, swept-footprint math, battery/HOME policy and existing radio/beacon/emergency software are implemented and regression-tested. Unvalidated live combinations and hardware contracts remain. |
| Final QA | **PASS for the executed checkout** | 500/500 pytest tests passed with no skips or deselections; 12/12 ROS 2 Jazzy packages built in an isolated `/tmp` overlay; statement coverage measured at 82%. |
| Gazebo acceptance | **PARTIAL** | Bounded scenarios listed below passed. Swept-boundary measurement during the stale bridge outage is INCONCLUSIVE. Gazebo evidence is simulation-only. |
| Physical/HIL acceptance | **BLOCKED** | Requires Team 1 hardware, authoritative interface specifications and a safe HIL procedure. |
| Publication/deployment | **NOT PERFORMED** | Local-only work; no GitHub or robot operation. |

## Twelve-domain completion checklist

The following 12 domains group the master mission's Phase 2 requirements; they
are not a claim that an earlier canonical 12-row checklist existed.

| # | Domain | Status | Results and limits |
| ---: | --- | --- | --- |
| 1 | Pose and LiDAR freshness | **PARTIAL** | Unit regressions cover missing, stale, future, non-finite and out-of-order data. A bounded Gazebo bridge pause made pose and scan stale together; the active goal requested cancellation, no new goal was sent, and HOME completed after input recovery. Pose-only and scan-only live faults were not separately run. The simulation's 0.5 s age and 10 Hz sensor assumptions need Team 1 calibration. |
| 2 | Continuous footprint and entrance boundary | **COMPLETE for the tested route/model** | Exact swept support is computed between each pair of observed planar poses, retaining the padded footprint and 5 cm translation allowance. Same-zone Gazebo path: 1,025 odometry samples, no gaps or unaligned-motion samples; minimum uncertain footprint x = −0.410 m versus the permitted plane at −1.0 m. During the stale bridge outage, an 8.874 s odometry gap and one unaligned-motion sample make that interval INCONCLUSIVE. Motion between samples follows the test's linear-translation/shortest-yaw interpolation assumption. |
| 3 | Localization uncertainty | **PARTIAL** | Seed `20261120`, 100 software-model trials, x/y bounds ±0.05 m and yaw ±5°. Same frontier candidate remained safe; 0 moving false watchdog cancels and false STUCK; 100/100 stationary watchdog and STUCK detections. This was not a live stochastic SLAM/Gazebo sensor-noise experiment and does not represent measured hardware accuracy. |
| 4 | Lost or ambiguous Nav2 results | **PARTIAL** | Unit/callback regressions exercise lost acceptance/result exceptions, unknown/nonterminal statuses, unconfirmed cancel and cancel/result races. The controller holds `busy` and latches `NAVIGATION_UNCERTAIN`; it does not claim motion stopped. A live Gazebo action server was not faulted to drop or contradict responses. Motor stop/lease acknowledgment needs Team 1. |
| 5 | Watchdog and physical STUCK distinction | **COMPLETE in recorded simulation** | Normal navigation with turning and intentional zero-command periods reached HOME without STUCK. Forward-drive immobilization generated 453 forward command samples with zero measured displacement and asserted STUCK. A separate watchdog case canceled after 26 simulated seconds and continued immobilization independently asserted physical STUCK. |
| 6 | Dynamic frontier retry lifecycle | **COMPLETE for the tested policy scenarios** | Deterministic tests and preserved Task 6 r4 Gazebo evidence cover retries, 15/30 s cooldowns, failure budgets, exhaustion, 120 s expiration, cooldown after re-arm and same-zone reactivation. The current controlled external-cancel run separately reselected the same spatial frontier after a 15 s cancellation-only cooldown with zero failure attempts/zones and reached HOME. |
| 7 | Battery reserve and HOME | **PARTIAL** | Simulated reserve return and HOME are covered by prior missions and current stale-input/external-cancel runs. HOME waited while pose/scan were stale and succeeded after recovery. Sudden measured pack drops, long physical return margins and battery calibration remain unverified without hardware telemetry. |
| 8 | Radio and beacon faults | **PARTIAL** | Software policy and mock tests cover radio quality, persistence, duplicate/late/rejected/timed-out beacon responses. No RF link-loss mission or physical dispenser confirmation was demonstrated in this QA run. Units, freshness, acknowledgment meaning and actuator behavior require Team 1. |
| 9 | Emergency and destruction behavior | **PARTIAL** | Existing software regressions and preserved simulated failure scenarios cover emergency state transitions, DESTROYED/STUCK reporting and mission continuity. Physical E-stop, motor stopping, radio loss, restart and beacon relay behavior require HIL; no live RF or actuation was attempted. |
| 10 | SLAM coverage and stability | **PARTIAL** | The isolated Karto grid-extents fix, native checks and patched runtime missions remain preserved. Positive-edge loss is addressed in that overlay. Lower-passage faces remain occluded/inaccessible with the real padded footprint; no geometry was weakened. No stochastic physical localization or comprehensive noisy-map acceptance is claimed. |
| 11 | Repeatable missions and frontier performance | **PARTIAL** | Bounded Gazebo evidence covers normal/HOME, forward immobilization, watchdog-then-STUCK, stale bridge recovery and external cancellation/HOME. The paired selector benchmark used 15 trials after two warmups across three captured and three seeded synthetic maps, with identical goals and diagnostics. Median time fell 43.1–71.9% depending on map. These scenario runs do not form one single all-fault mission. |
| 12 | Team 1 handoff and demonstration material | **PARTIAL** | Typed ROS interface inventory and Team 1 questions exist in `docs/results/team4-team1-integration-readiness.md`. This report adds a reproducible QA index and demo/presentation outline below. No demo video, formal sign-off or hardware contract is evidenced. |

## Executed scenario results

### Gazebo — PASS within stated simulation scope

- **Normal navigation, turns, zero-command intervals and HOME:** completed without
  false STUCK.
- **Forward-drive immobilization:** forward commands continued while the model
  remained stationary; physical STUCK detector asserted.
- **Watchdog followed by continued immobilization:** Nav2 watchdog cancellation
  preceded independent STUCK assertion.
- **Combined stale scan and localization bridge outage:** active goal
  cancellation was requested; no new frontier/HOME goal was sent while inputs
  were missing; after recovery, battery return reached HOME. Boundary assurance
  across the outage is **INCONCLUSIVE** because of the odometry gap and one
  unaligned-motion observation.
- **External frontier cancellation:** cancellation-only 15 s cooldown expired,
  the same spatial zone was selected, failure-attempt accounting remained
  unchanged, and HOME followed. The observed swept padded footprint plus 5 cm
  uncertainty stayed inside the entrance plane.

Evidence is under
`docs/results/evidence/team4-phase2-advanced-20261120/`, especially
`stuck-regression-20261120/`, `stale-inputs-20261120-181717/` and
`same-zone-swept-retry-20261120/`. Earlier failed setup attempts are retained in
their timestamped directories and are not counted as passes.

### Software fault injection and preserved mission evidence

- Cancellation-response ambiguity and uncertainty latching: navigation tests
  cover pending/unconfirmed cancellation, lost acceptance/result, unknown and
  nonterminal results, canceled/succeeded races, and retry suppression.
- Frontier lifecycle: Task 6 report and preserved `r4-expiry-reactivation-pass`
  evidence cover retry limits, cooldowns, exhaustion and reactivation.
- Battery, beacon, radio, emergency and destruction: existing component tests,
  mocks and previous controlled mission evidence are indexed in the historic
  completion reports. This final QA did not repeat hardware-dependent behavior.
- SLAM: historic patched-overlay mission evidence and `team4-slam-investigation.md`
  document the positive-grid-edge correction and remaining lower-passage
  occlusion.

## Fixes retained

- Freshness gates and stale-input cancellation prevent new navigation from
  relying on old pose/LiDAR; out-of-order and non-finite timestamps are rejected.
- Fallback odometry is transformed through fresh map-to-odom TF instead of
  treating odometry coordinates as map coordinates.
- An unresolved goal acceptance/result or unknown result status latches
  `NAVIGATION_UNCERTAIN`; the controller does not issue another goal or infer a
  physical stop.
- The battery HOME goal is preserved across its own pending acceptance.
- Swept padded-footprint containment evaluates the analytic minimum along the
  configured interpolated pose path without changing footprint or entrance
  geometry.

No additional reproducible production defect was found in the final scoped
review. Uncovered code is not treated as safe by default.

## Final QA and coverage

- Full applicable test suite: **500 passed, 0 failed, 0 skipped, 0 deselected**
  in 8.27 s. The three previously problematic ASGI tests were included. The
  repository declares no package-local ROS test dependencies; pytest is the
  complete declared test suite for this checkout.
- Coverage.py statement coverage over `src/`: **82%** (3,359 statements; 615
  missed). No branch-coverage measurement was made. Coverage is not a safety or
  reliability proof; wrapper/runtime paths remain less covered than pure policy
  code.
- ROS 2 Jazzy isolated build: **12/12 packages finished** in 19.7 s under
  `/tmp/livingmap-team4-master-qa-20261120/`. Four packages emitted the existing
  unused `CATKIN_INSTALL_INTO_PREFIX_ROOT` CMake warning; there were no build
  failures.
- The current-session audit found no session-owned Gazebo, ROS launch, pytest
  or colcon processes remaining.

Logs and coverage output:

- `docs/results/evidence/team4-master-qa-20261120/pytest-full-coverage.log`
- `docs/results/evidence/team4-master-qa-20261120/coverage-summary.txt`
- `docs/results/evidence/team4-master-qa-20261120/ros2-build.log`
- `docs/results/evidence/team4-master-qa-20261120/summary.json`

## Safety assumptions and hardware blockers

The 0.5 s freshness bound, 10 Hz Gazebo sensor cadence, 5 cm translation
allowance and ±5° yaw experiment are explicit software/simulation assumptions.
Team 1 must supply measured rates, timestamp behavior, frames, covariance/error
bounds and transform ownership before those values can become hardware
acceptance limits. The swept path model describes interpolation between
observations; it cannot certify unobserved motion during a prolonged sensor
outage.

Before physical acceptance, Team 1 must confirm the motor command mux/lease,
stop acknowledgment and Nav2-to-motor cancellation contract; LiDAR and fused
odometry frames/QoS/rates/covariance; measured battery telemetry and return
reserve; radio quality scale/freshness; beacon release/confirmation/jam
semantics; emergency-stop path; and safe HIL procedure. No physical stop or
HIL guarantee is claimed.

Remaining software validation gaps are separate live pose-only and scan-only
fault injections, stochastic Gazebo localization perturbation, live Nav2
missing/conflicting cancellation-response injection, and swept-boundary
measurement through stale-data intervals. These are **INCOMPLETE/INCONCLUSIVE**,
not passes. The test suite and current bounded scenarios reveal no known
critical software defect, but the gaps prevent declaring the complete system
accepted.

## Reproduction

Full automated suite (from the repository, in a normal Ubuntu 24.04 WSL shell):

```bash
cd /home/iyed/LivingMap-Team4
source scripts/_env.sh
python3 -m pytest -ra -q tests
```

Isolated ROS build used for this report (build products go to `/tmp`):

```bash
cd /home/iyed/LivingMap-Team4
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/livingmap-team4-master-qa-20261120/log build \
  --merge-install --base-paths "$PWD/src" \
  --build-base /tmp/livingmap-team4-master-qa-20261120/build \
  --install-base /tmp/livingmap-team4-master-qa-20261120/install \
  --parallel-workers 2
```

For individual bounded Gazebo scenarios, use the reviewed scripts
`scripts/validate_pr1_stuck_gazebo.sh`,
`scripts/validate_phase2_stale_inputs_gazebo.sh`, and
`scripts/validate_pr1_external_cancel_gazebo.sh` only from the expected branch,
with the validated isolated SLAM overlay available. Each script writes a new
run directory and has bounded cleanup; inspect its preflight and manifest
before execution. Machine-readable results and launch manifests in the evidence
directories identify the recorded run configuration.

## Demonstration and presentation outline

1. Show the robot/world, HOME, padded footprint and entrance boundary.
2. Show normal exploration, a turn, an intentional pause and a HOME return.
3. Inject forward immobilization; distinguish Nav2 watchdog cancellation from
   the later physical STUCK detector signal.
4. Pause the simulated sensor bridge; show stale-input cancellation and that
   goal dispatch waits for fresh data, then restore inputs and show HOME.
5. Cancel a frontier externally; show cancellation-only cooldown, unchanged
   unreachable-goal budget, same-zone retry and boundary evidence.
6. Show the limitation slide: uncertainty bounds are simulation assumptions;
   motor-stop, sensors, battery, RF and dispenser still need Team 1 HIL.

This is a proposed demonstration procedure and presentation outline, not a
recorded demo or formal sign-off.

## Local Git state

The QA report and logs are intended for a descriptive local commit on
`phase2/reliability-optimization-20261120`, authored as
`Iyed Madhkour <iyedmadhkour4@gmail.com>`. No remote operations, publication,
GitHub account actions or physical deployment were performed.
