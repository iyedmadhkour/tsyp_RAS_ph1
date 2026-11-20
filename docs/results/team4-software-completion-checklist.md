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
| 4 — Navigation and entrance safety | **COMPLETE for software scenarios; hardware acceptance PARTIAL** | A* / NO_VALID_PATH fallback, entrance mask, padded footprint, rotation and HOME are covered. A controlled Gazebo run verified the exact 15 s cancellation-only cooldown, zero failure attempts, and re-selection of the same spatial frontier zone after expiry. A 5 cm bounded x-pose-error envelope remained clear of the entrance. Actual hardware localization statistics and continuous swept-volume acceptance remain open. |
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

- Latest isolated ROS 2 Jazzy build: **12 packages finished** in the final
  controlled Gazebo run.
- Updated focused navigation, entrance-footprint and watchdog tests: **146
  passed**. Updated complete test suite with the checkout's Writer source first
  on `PYTHONPATH`: **468 passed**, no deselections.
- The three formerly hanging ASGI tests pass with unchanged AnyIO outside the
  workspace sandbox. The prior hang came from sandboxed async worker scheduling;
  no dependency pin or application workaround is required.
- The existing patched SLAM install was selected and verified through the demo
  entry point with `--check-slam-overlay`. A bounded Gazebo mission also
  verified the running SLAM process and its three patched libraries came from
  that overlay.
- External Nav2 cancellation Gazebo scenario: **PASS**, including same-zone
  re-selection. A preconnected probe canceled the first selected goal and
  paused exploration before its next frontier tick; the captured map/pose was
  held constant through the configured 15 s cancellation-only cooldown. The
  same spatial zone was selected after expiry, with zero failure zones or
  attempts. STUCK remained false, the watchdog did not cancel, and the battery
  reserve path returned the Writer to HOME. The minimum padded footprint x was
  -0.3600 m and its conservative 5 cm pose-error envelope was -0.4100 m, both
  on the permitted side of the x=-1.0 m entrance plane. Durable run evidence is in
  `docs/results/evidence/team4-same-zone-retry-20261120/`.
- STUCK scenarios: normal navigation/HOME, forward-drive immobilization, and
  watchdog cancellation followed by STUCK passed in bounded Gazebo runs.
- Task 6 expiry run: four fault injections were canceled by Nav2 watchdogs;
  maximum measured displacement was 0.0433 m. The runner intentionally stopped
  after the first post-expiry retry failure. It did not claim HOME completion.
- No physical hardware test, GitHub write, publish, merge or remote change was
  performed.

The new Gazebo run is under `/tmp/livingmap-pr1-evidence-20261120/`; its selected
logs and results are also committed in the evidence directory above. Earlier
captures remain under `/home/iyed/livingmap-pr1-evidence-20261120/` and were not
modified.

See `team4-final-software-review.md` for remaining hardware interface blockers,
overlay invocation, cancellation policy, and evidence limits.
