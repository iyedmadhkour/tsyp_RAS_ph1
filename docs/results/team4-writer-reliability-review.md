# Team 4 Writer Reliability Review

Date: 2026-11-18
Branch: `feature/team4-writer-dependencies-and-footprint-boundary`

## Confirmed issue fixed

Frontier, watchdog and sortie configuration accepted NaN and infinity because comparisons
such as `value <= 0` are false for NaN. This allowed unusable watchdog
deadlines, unbounded frontier-zone radii, and an unbounded `StuckDetector`
sample deque when its window was non-finite (the pruning comparison never
became true). The smallest fix rejects non-finite values in
`WatchdogConfig`, `FrontierPolicyConfig`, `Sortie`, and `StuckDetector`; a NaN
sortie budget had silently disabled budget-based HOME return. The detector
still accepts a finite nonpositive window as the existing explicit disable
behavior. Regression cases cover NaN and positive/negative infinity.

A second reproduced bug was that an unexplained canceled HOME action did not
increment the bounded return-attempt count. Repeated cancellations could keep
reissuing HOME indefinitely without reaching `RETURN_FAILED`. `_done()` now
counts a HOME cancellation with no internal reason as a failed return attempt,
but preserves explicitly intentional cancellation behavior. A regression test
reproduced the old failure before the fix and verifies the bounded terminal
state after it.

Focused tests after the fix passed. No Task 6 thresholds, entrance rules,
`progress.py` detection semantics, emergency handling, or SLAM behavior were
otherwise changed.

## Reviewed behavior

- Frontier failures are bounded to three attempts per zone, with 15 s then
  30 s cooldowns and bounded expiry/re-arm. Explicit cancellations are kept
  separate from navigation failures.
- Goal watchdog cancels stalled frontier goals before the 45 s physical STUCK
  timeout. HOME has longer watchdog limits; repeated terminal HOME failures
  are bounded and yield `RETURN_FAILED`.
- Battery return is latched and cancellation of the frontier goal is kept
  separate from failure memory. Existing tests cover reserve, HOME, and
  entrance-footprint constraints.
- A Nav2 server unavailable before goal submission results in no goal being
  sent. The explorer remains enabled and retries discovery on later ticks, but
  there is no timeout or explicit “Nav2 unavailable” state.
- A cancellation with a known internal reason (pause, battery, emergency) is
  correctly not treated as an unreachable frontier. An unexplained external
  cancellation is also counted as a cancellation rather than failure; the next
  selector pass can choose the same frontier again. Repeated external cancels
  can therefore cause rapid goal churn. The requirements do not specify
  whether to apply a separate cancellation backoff or escalate after a bound;
  leave this as an open policy decision.
- An unexplained HOME cancellation now consumes the existing bounded return
  attempt budget. Explicit intentional HOME cancellations remain distinct.
- `_cancel` requests cancellation asynchronously and waits for the action result
  callback to clear `busy`; the cancellation future itself is not observed.
  If Nav2 disappears after accepting a goal and neither cancellation nor
  terminal result arrives, the node can remain busy. This needs an action-level
  recovery policy and a ROS integration reproduction before changing.

## Sensor and hardware limitations

The Writer does not track LiDAR or odometry freshness. `frontier_node` uses the
latest transform and falls back to the most recently received odometry pose;
neither carries a watchdog-age check there. `Writer` similarly retains its
last odometry pose. Nav2's sensor-data timeout behavior is not configured in
the repository. Do not rely on stale-sensor behavior for hardware safety until
Team 1 supplies timestamp, QoS, and command-stop contracts and a focused
integration test is run.

The command/odometry STUCK detector from teammate PR #1 is not part of this
branch. Current Task 6 integration tests compose only the existing
position-window detector and goal watchdog. The PR review and integration plan
remain in `team4-stuck-pr-review.md` and
`team4-stuck-integration-test-plan.md`.

## Verification in this session

- Targeted Writer, frontier-policy, watchdog, navigation, boundary, battery,
  emergency, adapter, and beacon-placement tests: **207 passed**.
- Broad pytest suite excluding only the three previously identified hanging
  async ASGI tests: **373 passed, 3 deselected**.
- Isolated build of `living_map_interfaces` and `living_map_writer`: both
  packages finished successfully in **12.1 s**. Build/install/log outputs were
  written under `/tmp/livingmap-s2c-*`, not into the checkout.
- No Gazebo, hardware, system configuration or remote repository action was
  performed.
