# Team 4 software completion checklist

Updated 2026-11-20 on local branch `review/pr1-writer-stuck-detection`.
This review covers the eight milestones in `docs/implementation_plan.md` and
the remaining Writer integration evidence. Statuses separate simulation and
software completion from Phase 2 hardware acceptance.

| Milestone | Status | Evidence / remaining work |
| --- | --- | --- |
| 1 — Radio, beacon placement and persistence | **PARTIAL** | Protocol, placement, crash-tolerant storage, retry handling, hardware mocks and tests are present. A failed predecessor update now repairs itself from the persisted successor record; ambiguous/conflicting links are not guessed. Real RF calibration, physical beacon writes and dispenser acknowledgment/jam behavior need Team 1 hardware contracts and HIL. |
| 2 — Battery reserve and return | **PARTIAL** | Odometry-based travel accounting, reserve return and HOME behavior pass simulation and regression tests. Real pack telemetry, sudden-drop behavior, calibration and reserve uncertainty require hardware. |
| 3 — Emergency and Writer destruction | **PARTIAL** | Dying-gasp deadline, late-deployment gate, continuing beacon advertisements, and STUCK/FAILED behavior pass tests and controlled simulation. Physical motor stop, radio loss and restart behavior need HIL. |
| 4 — Navigation and entrance safety | **PARTIAL** | A* primary / NO_VALID_PATH fallback, footprint boundary protection, rotation and HOME return have test and mission evidence. Unexplained external frontier cancellations now receive the ordinary base cooldown without becoming reachability failures; intentional cancels remain distinct. Some targets remain unreachable and repeated routes do not prove universal reliability; swept-volume safety and noisy localization need dedicated evidence. |
| 5 — SLAM | **PARTIAL** | The Karto positive-edge grid defect has a pinned, isolated 2.8.5 patch with 8/8 native tests and three completed patched missions. `scripts/run_demo.sh` now supports explicit overlay selection and verifies the resolved package prefix; stock ROS remains the default. Occluded lower wall faces, map completeness and physical localization remain unresolved. |
| 6 — Writer exploration/watchdog lifecycle | **COMPLETE for the validated software scenarios** | Jazzy build and regression tests pass. Gazebo verified watchdog cancellation, retries 1/3–3/3, 15/30 s cooldowns, exhaustion, 120 s expiry, post-rearm cooldown and same-zone reactivation. Temporary diagnostic policy overrides were isolated; production defaults are unchanged. Battery-return and HOME evidence remains from prior runs. |
| 7 — End-to-end failure cases | **COMPLETE in recorded simulation** | Preserved Gazebo runs cover Writer destruction and immobilization with mission continuity, beacon relay destruction, and return/HOME. The later PR #1 Gazebo runs cover normal navigation, forward immobilization/STUCK and watchdog-then-STUCK. These are simulated, not physical acceptance. |
| 8 — User documentation and submission | **PARTIAL** | Installation/demo instructions, technical report, diagrams and failure analysis exist. Demo video, pitch rehearsal and formal code-freeze/submission sign-off are not evidenced here. |

## Cross-cutting interfaces

- **Software mocks:** COMPLETE for stale/invalid LiDAR and odometry samples,
  radio quality, battery changes, delayed/rejected/timed-out beacon service
  responses, and late acknowledgments. These are test doubles, not ROS hardware
  drivers and cannot actuate devices.
- **Team 1 interface contract:** BLOCKED pending authoritative sensor frames,
  timestamps, QoS/rates, odometry fusion, command mux and motor lease, measured
  battery, RF units/freshness, and physical beacon/dispenser acknowledgments.
- **Hardware-in-loop and physical acceptance:** BLOCKED until the hardware and
  safe test procedure are available.
- **Task 6 Gazebo evidence:**
  `/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-20/r4-expiry-reactivation-pass/`.
  Existing r3 and earlier r4 evidence was preserved.
- **Task 6 known diagnostic failure:** the first post-fix run used
  `min_cluster_cells=1000`, filtered all candidates and returned
  `NO_REACHABLE_FRONTIER` before expiry. It is preserved separately and is not
  counted as a pass. The successful rerun used the production value 5.

## Current validation

- Isolated ROS 2 Jazzy build: **12 packages finished**.
- Full offline test suite: **463 passed, 3 ASGI API tests deselected**. The
  current environment's AnyIO 4.16.0 worker implementation hangs on sequential
  thread-pool work; the worker exits after one job but is returned to the idle
  queue. `requirements-dev.txt` now bounds AnyIO below 4.16.0. Those three tests
  still need rerunning in an environment with a compatible resolved version.
- New focused navigation/watchdog regressions: **83 passed**. Unexplained
  frontier cancellations are paced for the configured base cooldown and do
  not increment navigation failure attempts.
- Final isolated ROS 2 Jazzy build: **12 packages finished** under
  `/tmp/livingmap-final-20261120/`.
- The existing patched SLAM install was selected and verified through the demo
  entry point with `--check-slam-overlay`; Gazebo was not started by this check.
- STUCK scenarios: normal navigation/HOME, forward-drive immobilization, and
  watchdog cancellation followed by STUCK passed in bounded Gazebo runs.
- Task 6 expiry run: four fault injections were canceled by Nav2 watchdogs;
  maximum measured displacement was 0.0433 m. The runner intentionally stopped
  after the first post-expiry retry failure. It did not claim HOME completion.
- No physical hardware test, GitHub write, publish, merge or remote change was
  performed.

See `team4-final-software-review.md` for the remaining ASGI dependency blocker,
overlay invocation, cancellation policy, and final work-session scope.
