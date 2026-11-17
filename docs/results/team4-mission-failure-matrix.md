# Writer Mission Failure Scenario Matrix

Reviewed 2026-11-18 using source inspection, deterministic tests and
test-only adapters. No Gazebo or physical hardware scenario was run.

| Scenario | Existing behavior/evidence | Result and remaining gap |
| --- | --- | --- |
| LiDAR stops updating during navigation | The obstacle layer consumes `/writer/scan`; no `expected_update_rate` or explicit Writer scan-age guard is configured. The mock adapter reports stale data. | **Gap.** No verified stop-on-stale-scan policy. Team 1 must confirm rate/QoS; Nav2 timeout and safe-stop behavior need a focused integration test. |
| Localization is incorrect or stale | Watchdog prefers Nav2 distance feedback, then uses current map pose; `frontier_node` falls back to the latest received odometry pose without an age check. | **Partial.** Goal timeout bounds some static cases, but there is no stale pose/covariance gate. Do not claim hardware localization safety. |
| Nav2 server unavailable before sending a goal | `tick()` checks `server_is_ready()` and returns without sending; a deterministic test confirms no goal is sent. | **Safe wait, no alarm.** There is no deadline or explicit unavailable state. Server disappearance after acceptance/cancel-response loss is not covered by that check. |
| Radio quality falls below relay threshold | Beacon placement policy compares normalized simulated link quality with configured 0.85 threshold; placement unit tests cover relay priority/cooldown. | **PASS in policy tests only.** Real RF quality, units, freshness and recovery are unvalidated. |
| Beacon deployment response is delayed, rejected or times out | Current beacon manager directly handles simulated logical deployment; no physical acknowledgment client exists. Test-only service mock exercises delay, rejection, timeout and late responses. | **Mock coverage only.** Real dispenser timeout/retry semantics require Team 1 contract. |
| Battery estimate suddenly drops | Current Writer battery is an internal model, not external telemetry. Mock accepts an abrupt percent change; no production telemetry input consumes it. | **Hardware gap.** Threshold response to sudden measured drop is not validated; do not infer it from the continuous simulation energy model. |
| HOME temporarily unreachable | HOME uses longer watchdog bounds and configured bounded return attempts. An unexplained canceled HOME goal now consumes one bounded attempt; explicitly intentional cancels remain separate. | **Bounded offline behavior.** No Gazebo reproduction in this session; transient versus permanent failure remains unclassified. |
| Repeated frontier failures | Policy bounds attempts and cooldowns; deterministic tests cover 15 s, 30 s, exhaustion and re-arm. | **PASS in unit tests; PARTIAL in Gazebo.** The r3 simulation recorded only the first failure. |
| Unexpected external action cancellation | Known internal cancellations preserve their reason and are not marked unreachable. An unexplained canceled HOME goal consumes a bounded return attempt. An unexplained canceled frontier remains cancellation-only and can be selected again on the next tick. | **Partial.** HOME is bounded; repeated frontier cancellations can churn. Define whether to apply separate frontier backoff or bounded escalation. |
| Cancellation acknowledgment/result never arrives | `_cancel()` sends asynchronously; `busy` clears in the result callback, and the cancel future is not monitored. | **Unverified edge.** If Nav2 disappears after accepting the goal, a missing result could leave the explorer busy. Needs ROS action reproduction and a defined recovery policy. |

## Evidence boundaries

The targeted regression suite passed 207 tests, including the new
watchdog/STUCK composition and adapter tests. The broad suite passed 373 tests
with three known hanging ASGI cases deselected. `living_map_interfaces` and
`living_map_writer` also built successfully in an isolated overlay. These are
offline tests, not ROS transport, Nav2, Gazebo, RF, battery hardware or
dispenser tests. Existing
Task 6 r3 evidence remains the only controlled watchdog simulation in the
current evidence set; cooldown retries and expiry remain unverified there.

Do not begin physical integration until Team 1 confirms sensor timestamps and
frames, motor stop behavior, battery telemetry and beacon acknowledgment. Keep
unresolved navigation cancellation and stale-sensor policies visible in future
test plans.
