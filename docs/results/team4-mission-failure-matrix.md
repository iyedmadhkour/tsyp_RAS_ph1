# Writer Mission Failure Scenario Matrix

Initial review: 2026-11-18, using source inspection, deterministic tests and
test-only adapters. A 2026-11-20 continuation added bounded Gazebo validation
for the corrected Writer demo route and PR #1 STUCK scenarios; physical
hardware remains untested.

| Scenario | Existing behavior/evidence | Result and remaining gap |
| --- | --- | --- |
| LiDAR stops updating during navigation | The obstacle layer consumes `/writer/scan`; no `expected_update_rate` or explicit Writer scan-age guard is configured. The mock adapter reports stale data. | **Gap.** No verified stop-on-stale-scan policy. Team 1 must confirm rate/QoS; Nav2 timeout and safe-stop behavior need a focused integration test. |
| Localization is incorrect or stale | Watchdog prefers Nav2 distance feedback, then uses current map pose; `frontier_node` falls back to the latest received odometry pose without an age check. Entrance navigation now reserves a configured 5 cm translational pose-error bound. | **Partial.** Tests and Gazebo check a bounded 5 cm x-error envelope; there is no stale pose/covariance gate or stochastic SLAM localization test. Team 1 must provide sensor rates, timestamps and measured covariance/error before freshness policy and the 5 cm assumption can be accepted for hardware. |
| Nav2 server unavailable before sending a goal | `tick()` checks `server_is_ready()` and returns without sending; a deterministic test confirms no goal is sent. | **Safe wait, no alarm.** There is no deadline or explicit unavailable state. Server disappearance after acceptance/cancel-response loss is not covered by that check. |
| Radio quality falls below relay threshold | Beacon placement policy compares normalized simulated link quality with configured 0.85 threshold; placement unit tests cover relay priority/cooldown. | **PASS in policy tests only.** Real RF quality, units, freshness and recovery are unvalidated. |
| Beacon deployment response is delayed, rejected or times out | Current beacon manager directly handles simulated logical deployment; no physical acknowledgment client exists. Test-only service mock exercises delay, rejection, timeout and late responses. | **Mock coverage only.** Real dispenser timeout/retry semantics require Team 1 contract. |
| Battery estimate suddenly drops | Current Writer battery is an internal model, not external telemetry. Mock accepts an abrupt percent change; no production telemetry input consumes it. | **Hardware gap.** Threshold response to sudden measured drop is not validated; do not infer it from the continuous simulation energy model. |
| HOME temporarily unreachable | HOME uses longer watchdog bounds and configured bounded return attempts. An unexplained canceled HOME goal now consumes one bounded attempt; explicitly intentional cancels remain separate. | **Bounded offline behavior.** Normal route and r3 failure/battery-return simulations reached HOME. Transient versus permanent HOME failure remains unclassified. |
| Repeated frontier failures | Policy bounds attempts, cooldowns, exhaustion and two re-arms. | **PASS in deterministic tests and bounded Gazebo.** r4 recorded attempts 1/3–3/3, 15 s/30 s cooldowns, exhaustion, time expiry, a post-rearm cooldown, and a same-zone attempt 1/3 after expiry. |
| Unexpected external action cancellation | Known internal cancellations preserve their reason and are not marked unreachable. An unexplained canceled HOME goal consumes a bounded return attempt. An unexplained canceled frontier uses the configured 15 s cancellation-only cooldown and does not consume unreachable-goal attempts. | **PASS for the controlled case.** A bounded Gazebo run selected the same spatial frontier after cooldown, with zero failure attempts and safe battery-triggered HOME. Repeated unexplained cancellations can continue to pace indefinitely; they are intentionally not counted as reachability failures. |
| Cancellation acknowledgment/result never arrives | `_cancel()` sends asynchronously; `busy` clears in the result callback, and the cancel future is not monitored. | **Unverified edge.** If Nav2 disappears after accepting the goal, a missing result could leave the explorer busy. Needs ROS action reproduction and a defined recovery policy. |

## Evidence boundaries

The initial targeted regression suite passed 207 tests, including the new
watchdog/STUCK composition and adapter tests. The broad suite passed 373 tests
with three known hanging ASGI cases deselected. `living_map_interfaces` and
`living_map_writer` also built successfully in an isolated overlay. These are
offline tests, not RF, battery hardware or dispenser tests. At the time of the
initial 2026-11-18 review, r3 was the only controlled watchdog simulation;
the later Gazebo runs are summarized below. Cooldown time-expiry and re-arm
were then unverified in simulation.

## 2026-11-20 continuation

The corrected fallback route completed the bounded normal-navigation scenario
without false STUCK and reached HOME. Forward-drive immobilization published
STUCK with stationary odometry. A separate static-Writer Nav2 run logged a
watchdog cancellation followed by STUCK. The isolated ROS build completed 12
packages; the full offline suite passed **463** with the three known hanging
async API tests deselected. The completed r4 run confirmed three retries and
cooldowns, then expiry and same-zone reactivation after 139.99 simulated
seconds. A Task 1 follow-up also added tested repair for a durable beacon drop
whose predecessor write initially fails. Hardware freshness, localization,
command-mux, RF and dispenser gaps remain as listed in the matrix.

Do not begin physical integration until Team 1 confirms sensor timestamps and
frames, motor stop behavior, battery telemetry and beacon acknowledgment. Keep
unresolved navigation cancellation and stale-sensor policies visible in future
test plans.

## 2026-11-20 completion follow-up

The controlled same-zone cancellation scenario passed after a preconnected
probe canceled the first goal and held the Writer/map snapshot through the
15-second cooldown. The same zone was selected after expiry; cancellation-only
memory remained outside failure-attempt budgeting. The 5 cm bounded pose-error
margin was included in frontier goal placement, the private entrance mask, and
the Gazebo observer's worst-case footprint check. The full suite passed 468
tests, focused navigation/entrance/watchdog tests passed 146, and the isolated
Jazzy build completed 12 packages. Captured evidence is in
`evidence/team4-same-zone-retry-20261120/`.

This closes the defined software scenario. Hardware localization accuracy,
sensor freshness thresholds, RF/battery telemetry, motor lease/stop behavior,
and physical dispenser acknowledgments remain blocked on Team 1 interfaces and
hardware-in-loop validation. No continuous swept-volume or stochastic SLAM
localization acceptance is claimed.
