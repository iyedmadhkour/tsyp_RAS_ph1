# Phase 2 validation evidence (2026-11-20)

This directory preserves the bounded local ROS/Gazebo results generated from
`phase2/reliability-optimization-20261120` at source HEAD `8714209`. The
original runs remain in `/tmp/livingmap-phase2-stuck-20261120/20261120-172045`
and `/tmp/livingmap-phase2-gazebo-20261120/external-cancel-20261120-171742`.

- `stuck/`: complete harness summary, focused pytest output, isolated build and
  package-resolution records, per-case compressed launch logs, topic logs, rosbag2 MCAP
  bags and command/odometry analyses. Normal route/HOME, forward physical
  immobility, and watchdog-cancel-then-STUCK scenarios all passed.
- `external-cancellation/`: controlled cancellation probe, full observer event
  stream and compressed launch log, package resolution and live patched-SLAM process map,
  retry/HOME requests, final machine-readable results, and cancellation/SLAM
  lifecycle evidence.
- `../team4-phase2-baseline-20261120/`: captured map inputs and hashes plus
  baseline/optimized selector benchmark JSON and exact-output comparison.

The isolated colcon build/install trees and ROS temporary logs are omitted;
all three MCAP bags, scenario logs, result summaries and runtime resolution
proofs are retained. Simulated evidence is not hardware acceptance. No remote
Git operation was performed.
