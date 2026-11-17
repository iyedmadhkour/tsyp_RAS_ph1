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

## Remaining failures

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
`/home/iyed/livingmap-pr1-evidence-20261117/`. No generated logs, temporary
builds, secrets, or production changes are included in this checkpoint.
