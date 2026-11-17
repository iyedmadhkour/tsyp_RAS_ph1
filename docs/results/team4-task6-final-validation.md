# Team 4 Task 6 — Final Validation Checkpoint

Date: 2026-11-17
Branch: `feature/team4-writer-dependencies-and-footprint-boundary`
Local HEAD: `2bbc665752917c3d394aaf1953754f80f8ada872`

## Status: PARTIAL

Task 6 production changes are built and covered by deterministic tests. The
controlled r3 Gazebo run exercised the navigation watchdog and a first frontier
failure. The follow-up r4 scenario for repeated same-zone failures and cooldown
waiting was prepared and safety-reviewed, but could not be launched in this
execution environment. No result is inferred for unrun behavior.

## Verified controlled Gazebo run

Evidence: `/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-watchdog-cooldown-r3/`.

- The zero-velocity fault began at simulation time 6.002 s. Maximum movement
  during injection was 0.01734 m.
- At 30.002 s the goal watchdog canceled the goal, 24.0 simulated seconds
  after injection. It reported `recovery_limit`, after 26.0 s without progress
  and three Nav2 recoveries. This preceded the independent 45 s STUCK interval.
- The frontier failure was recorded as attempt 1/3 with a 15 s cooldown.
  Five other frontier goals were selected; the cooldown-wait and same-zone
  retry paths were not reached.
- The Writer explored other goals and returned HOME with 70.5% battery. No
  unexpected STUCK or FAILED state was recorded.
- There were zero entrance-boundary violations in 669 published footprint
  samples. Minimum sampled footprint x was -0.360003 m; the permitted minimum
  is -1.0 m.
- Runner and monitor exited successfully. The original r3 injector exited 1
  during cleanup after SIGINT raised `KeyboardInterrupt` and an unconditional
  second `rclpy.shutdown()` raised an already-shutdown error. The separate r4
  helper guards shutdown with `rclpy.ok()`; this cleanup path has not been run.

## r4 scenario and remaining simulation checks

The harness is at
`/home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-cooldown-retry-r4/`.
It uses ROS domain 184, separate runtime/log paths, a 560 s runner limit, and
runtime-only `min_cluster_cells=1000`. Fault injection is limited to a single
1.5 m failure zone, maximum 0.25 m movement, 42 simulated seconds per stall,
75 wall seconds for a stalled simulation clock, and at most three failures.
It requests a stop if a fourth goal or another zone is selected. The filter is
map-dependent, so a missing initial candidate yields no validation rather than
an assumed pass.

The runner fails closed if the r4 directory already contains output from a
prior run or if `localhost:8081` is already accepting connections. This
prevents overwriting evidence and avoids sending the mission-start request to
an unrelated dashboard.

From a normal Ubuntu 24.04 WSL terminal with Gazebo/WSLg and NVIDIA D3D12
available, run once:

```bash
cd /home/iyed/LivingMap-Team4
source /opt/ros/jazzy/setup.bash
source scripts/_env.sh
source /home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/install-final/local_setup.bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
python3 /home/iyed/livingmap-setup/writer-task6-k3m9/validation-2026-11-17/controlled-cooldown-retry-r4/run_once.py
```

Review the saved event times for waits of 15 s and 30 s, attempts 1/3 through
3/3, terminal `FRONTIERS_EXHAUSTED`, HOME, battery, footprint and injector
exit. Expiry and map-growth reactivation need a separate long-lived reachable
frontier scenario exceeding 120 s after exhaustion; no such Gazebo scenario is
currently prepared or verified.

The Codex sandbox previously failed WSLg display/GPU access (`/dev/dxg` and
`/dev/dri` absent, display unavailable) and Gazebo Transport interface queries
(`getifaddrs` denied). A headless server-only probe also failed before world
load. Gazebo was not retried here. Run the command above only from the normal
Ubuntu terminal; if its display/GPU prerequisites are absent, stop and preserve
the partial result.

## Automated checks

- Broad pytest regression: **321 passed, 3 deselected**, 6.94 s. The excluded
  tests are known async ASGI hangs:
  `test_command_post_and_harness_views_are_separate`,
  `test_api_validates_actions_and_reports_rejected_commands`, and
  `test_scenario_api_validates_before_restart`.
- Focused Writer/navigation regression recorded earlier in this validation:
  **170 passed**.
- Isolated ROS build recorded earlier: **12 packages finished** in 33.6 s.
- `colcon test` invoked the ament_python package test runners, but eight
  packages returned code 5 with `Ran 0 tests / NO TESTS RAN`; four non-Python
  packages finished. Treat this as test-discovery/tooling failure, not passing
  package tests.
- Deterministic tests already cover 15 s/30 s cooldowns, three attempts,
  exhaustion and re-arm, map-growth release, cancellation versus failure,
  watchdog progress/rotation/recovery, battery return, entrance geometry and
  emergency states. No redundant tests were added in this checkpoint.

## Git and process state

The checkout was clean at the start of this checkpoint. Repository changes are
validation documentation only. The external r4 helper received fail-closed
preflight checks; no production code, SLAM patch, `progress.py`, emergency
handling, PR #1, system packages, or hardware was changed. Do not treat the
unrun r4 scenario as a pass. No push, merge, approval or GitHub write was made.
All Gazebo/ROS simulations must be stopped before closing the Ubuntu session.
