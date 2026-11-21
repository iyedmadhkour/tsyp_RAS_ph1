# STUCK Detector and Task 6 Watchdog Integration Test Plan

## Scope and current state

This plan was written before the teammate's PR #1 detector was integrated into
the local review branch. The 2026-11-20 continuation below records its current
command/odometry implementation and live Gazebo results. See
`team4-pr1-stuck-validation-checkpoint.md` for the detailed evidence.

### 2026-11-20 continuation

The PR #1 detector is present locally on `review/pr1-writer-stuck-detection`.
The isolated Jazzy overlay loaded `living_map_writer` and `progress.py` from the
fresh build. Focused integration tests passed (134); the full offline suite
passed 460 with three known hanging async API tests deselected. Bounded Gazebo
scenarios verified normal navigation and HOME, forward-drive immobilization
with stationary odometry, and a Nav2 watchdog cancellation followed by STUCK.

These results support software integration review. They do not settle the
one-second stale-command assumption against Team 1's motor-controller lease,
stale hardware odometry behavior, cancellation-response loss, or physical
motor/RF/dispenser behavior. Those remain hardware-contract or HIL items.

## Existing isolated checks

| Scenario | Current offline result |
| --- | --- |
| Stationary during a frontier goal | Navigation watchdog cancels first; detector is reset when the explorer leaves `NAVIGATING`. |
| Intentional pause | `driving=False` resets the position-window detector. |
| Long in-place rotation | Task 6 watchdog permits rotation grace, then cancels before the 45 s STUCK window. |
| Short Nav2 recovery followed by motion | Progress within watchdog windows prevents cancellation; odometric displacement clears STUCK history. |
| Moving HOME | Feedback and pose progress prevent both timers from expiring. |
| Truly stationary HOME return | Current position-only detector reaches STUCK at 45 s, before the 90 s HOME watchdog stall limit. |
| Missing Nav2 feedback | Watchdog falls back to goal-to-pose distance. |

## Original planned tests (written before PR #1 integration)

Run the same cases against its actual detector and Writer node wiring:

1. Fresh nonzero translation command plus stationary odometry accumulates
   evidence and declares STUCK only at the configured evidence limit.
2. Paused/intentional zero command does not accumulate evidence; starting a
   new navigation period clears previous command state.
3. Rotation whose measured yaw follows angular commands is tolerated, while
   commanded rotation without corresponding yaw eventually counts as a stall.
4. Brief Nav2 spin/wait/backup recovery does not falsely declare STUCK; repeated
   blocked-path recovery with no useful progress is bounded.
5. The Task 6 watchdog cancels a stalled goal before the physical detector
   reports STUCK. A canceled goal must stop contributing navigation evidence.
6. A truly immobilized Writer during a sustained HOME attempt reaches STUCK
   before the HOME action's terminal failure policy, without suppressing the
   emergency status or radio behavior.
7. A stale velocity command expires according to the Team 1 base-controller
   lease, and delayed/missing odometry pauses or degrades detection exactly as
   documented. Confirm that contract before asserting a timeout value.
8. Inject a ROS clock rewind, duplicate/out-of-order inputs, and a delayed goal
   cancel response; confirm no stale goal or evidence crosses sorties.

## Proposed execution sequence

1. Rebase the teammate change onto the Team 4 feature branch in a separate
   review workspace; do not merge as part of this test plan.
2. Add unit and node-wiring tests with fakes for subscriptions, clock and action
   futures. Include source-less shared `Twist` commands and state transitions.
3. Run the full Python regression suite and build the Writer package.
4. In an approved normal WSL terminal, run bounded simulation-only cases for
   normal rotation, recovery, injected immobility, watchdog cancellation and
   HOME return. Preserve each run in a new evidence directory.
5. Compare STUCK state, cancel/result ordering, position/yaw/command timelines,
   battery reserve, and entrance-footprint samples. Do not connect hardware.

## Team 1 dependency questions

- Does the motor driver stop after a finite command lease? What is the exact
  lease and how is command-source arbitration represented?
- Are odometry stamps monotonic, and can it report stale/invalid status or
  covariance? Which frame is continuous during SLAM corrections?
- Does commanded angular velocity imply motion at the base, and what yaw sensor
  should be authoritative when encoders spin against an obstacle?
- What behavior is required when cancel acknowledgment or action result is
  missing? Until these contracts are agreed, PR #1 is **not ready for
  integration** and no hardware safety claim follows.
