# Teammate STUCK Detection PR #1 — Read-only Review

Reviewed 2026-11-17 from GitHub PR metadata and per-file patches. No PR action
was taken.

## Proposed change

PR #1, “Writer stuck detection,” is open against `main` at
`aeb012714c3ebf16262b949bf58c09c45857612b`. Its head is
`alibn07/tsyp_RAS_ph1:writer-stuck-detection` at
`8b30a61ecc895e418fe7f1561d3c864e9aae003e`. The patch changes five files
(1,096 additions, 28 deletions): the Writer node and `progress.py`, a wiring
test, a 578-line mathematical detector simulation, and failure-analysis docs.
Source: [PR #1](https://github.com/iyedmadhkour/tsyp_RAS_ph1/pull/1).

The detector replaces the current pose-window rule with a command/odometry
evidence model. It subscribes to `/writer/cmd_vel` and `/writer/odom`, counts
translation and unverified rotation command intervals, treats fresh zero and
stale commands as neutral after command-stream confirmation, and clears
evidence after measured displacement. It remains in the Writer supervisor and
publishes the existing STUCK state; it does not replace the Task 6 goal
watchdog.

## Compatibility and review findings

- The feature branch’s Task 6 watchdog cancels an individual goal after 25 s
  of inadequate progress; the current physical STUCK detector uses a 45 s
  threshold. These are distinct responsibilities and can coexist. However,
  the PR targets `main`, which predates the Task 6 watchdog, so its Writer-node
  wiring has not been tested together with watchdog cancellation, cooldown
  waits, or battery return.
- The PR test file describes mathematical simulation, not a running ROS node,
  Nav2, Gazebo, or a physical motor controller. It does not establish topic
  delivery, command multiplexing, sensor timing, or field behavior.
- The topic is shared by Nav2, demo fallback and Writer stop commands. The PR
  acknowledges that plain `Twist` has no source identity. The safety meaning
  of its one-second stale-command timeout must be checked against the eventual
  Team 1 motor-driver command lease/watchdog; that contract is not present in
  this repository.
- The algorithm has substantially more state and timing assumptions than the
  current detector. Its edge cases and node-level behavior need isolated ROS
  tests, especially missing/stale odometry, clock reset, successive cancelled
  goals, and continuous short navigation attempts.

## Recommendation

**Not ready for integration yet; request joint validation first.** The design
is plausibly complementary to Task 6 and the PR includes broad deterministic
coverage, but there is no ROS/Gazebo acceptance evidence or motor-driver
timeout contract. Before integration, rebase/test against the feature branch,
add node-level command/odometry tests, verify the stale-command assumption with
Team 1, and run bounded Gazebo checks covering normal rotation/recovery,
watchdog cancellation, battery HOME return, and true immobilization. This is a
review recommendation only; the PR was not edited, approved, merged or
cherry-picked.

## Local validation follow-up — 2026-11-20

The detector was subsequently present on the local review branch and was
validated without any GitHub action. The isolated ROS 2 Jazzy build completed
12 packages; focused tests passed 134 and the broad offline suite passed 460
with three known hanging async API tests deselected. Three bounded Gazebo
scenarios passed: normal navigation with HOME return, forward-drive physical
immobilization, and watchdog cancellation followed by STUCK. The former
waypoint-5 event was traced to the demo fallback crossing a wall outside the
opening; the fallback route now uses a footprint-safe passage waypoint.

This clears the earlier gap in software/Gazebo evidence and supports
integration review on the Team 4 software branch. It is not hardware
acceptance: Team 1 still needs to confirm the motor command lease against the
detector's stale-command timeout, odometry freshness/clock semantics, command
mux/stop acknowledgment, and safe response to missing action results. No PR,
branch, remote or GitHub state was changed by this validation.
