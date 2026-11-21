# PR #1 STUCK validation checkpoint

Checkpointed locally on `review/pr1-writer-stuck-detection`, based on
`0f79291f1bcddaa750462de7b7ac425eaa540b8e`. PR #1 and GitHub were not changed.

## Confirmed

- Isolated ROS 2 Jazzy build succeeded; the full workspace build completed 12
  packages. ROS loaded `living_map_writer` and its detector from the isolated
  build/install.
- Focused Writer, detector, wiring, watchdog and navigation tests: **133 passed**.
- Gazebo Harmonic headless smoke world completed 3,000 iterations.
- In a static Writer simulation, the live node published `writer_stuck=true`
  after 45 seconds of unverified rotation commands without odometry yaw. That run
  did not demonstrate a forward command. The dynamic route exercised translation,
  rotation commands, zero commands and odometry.

## Resumed validation — 2026-11-18

The failed waypoint was a demo-route geometry error, not a false STUCK result.
The segment from `(15.4, 0.0)` to `(14.5, 3.1)` crossed the gas divider near
odom x=14.96, inside the Writer's footprint clearance from the divider end at
x=14.75. The route now passes through `(15.3, 2.0)` in both directions. A
regression test projects the Writer's 0.63 m × 0.465 m footprint through the
divider and verifies clearance from both wall ends.

The runner's clean-tree guard now allows only the four reviewed validation
files, and a failed bag-analysis check marks that scenario failed while still
allowing later bounded scenarios to run. The previous run stopped after the
normal route failed because `set -e` exited on the analysis pipeline.

### Validation results

- Isolated ROS 2 Jazzy build: **12 packages succeeded**. Both `living_map_writer`
  and `living_map_writer.progress` resolved from the isolated build.
- Focused Writer detector/watchdog/navigation suite: **134 passed**.
- Full offline project suite: **460 passed, 3 known hanging async API tests
  deselected**. The excluded tests are recorded in
  `team4-task6-final-validation.md`.
- `normal-home`: PASS. 1,161 translation, 175 rotation and 15 zero command
  samples; 4,014 odometry samples; 42.73 m path; HOME endpoint 0.34 m from the
  initial odometry position. No false STUCK.
- `physical-immobility`: PASS. Static Writer received 454 forward-command
  samples; 1,668 odometry samples showed 0.00 m path and 0.00 m maximum
  deviation; STUCK was published.
- `watchdog-immobility`: PASS. Nav2 cancelled a stalled frontier after 26 s;
  the physically immobilized Writer later published STUCK. This run used a
  static model and recorded fresh odometry.
- Gazebo, ROS launch, bag and test processes were stopped by the runner. The
  preserved evidence is at
  `/home/iyed/livingmap-pr1-evidence-20261118/` (65 MB, 2,535 files).

### Current status

All three bounded PR #1 Gazebo scenarios passed. Intentional stops and
rotation-only commands are represented in the normal route capture; forward
immobilization and watchdog interaction were separately exercised. Safe HOME
return is confirmed by the normal route and remains supported by the earlier
r3 failure/battery-return capture. Three full ASGI tests remain deselected
because they hang under the existing test setup. No hardware or GitHub
validation was performed.

## Historical Nov 17 failures (superseded by the resumed validation above)

- The dynamic route reached waypoint 5 of 13, then latched STUCK after 45 seconds
  without progress. It did not reach HOME; normal navigation without STUCK and
  safe HOME return remain unverified.
- The static Nav2 run logged three watchdog cancellations at 22 seconds, but did
  not publish STUCK within 180 seconds. Recorded odometry was not stationary
  enough to confirm that this run maintained physical immobilization.

## Resume

The reviewed, bounded runner is [scripts/validate_pr1_stuck_gazebo.sh](../../scripts/validate_pr1_stuck_gazebo.sh).
Its static case aligns the Writer with its first route target and checks for
forward commands with stationary odometry.
Run it from Ubuntu WSL with:

```bash
cd /home/iyed/LivingMap-Team4 && bash scripts/validate_pr1_stuck_gazebo.sh
```

The complete logs and MCAP evidence are preserved outside the repository at
`/home/iyed/livingmap-pr1-evidence-20261118/`; the earlier
`/home/iyed/livingmap-pr1-evidence-20261117/` set is also preserved. Only the
reviewed route correction, validation runner, regression test, and reports are
tracked in the local changes. Generated logs, temporary builds, and secrets
are not included.
