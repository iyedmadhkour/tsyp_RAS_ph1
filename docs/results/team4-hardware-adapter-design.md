# Proposed Writer Hardware Adapter Design

Status: **mock-only planning**. No production topics changed, no firmware or
hardware accessed. Confirmed simulation contracts and open Team 1 questions
are listed in `team4-team1-integration-readiness.md`.

## Boundary

Keep Writer autonomy consuming its current ROS contracts. A future launch
profile selects either mock publishers or Team 1 drivers behind small
adapters; autonomy nodes should not branch on device models. The proposed
test-only implementation is `tests/support/writer_hardware_adapters.py` with
tests in `tests/test_writer_hardware_adapter_mocks.py`. It does not publish ROS
messages and cannot actuate hardware.

| Mock input/output | Existing consumer contract | Freshness/failure behavior in the test double |
| --- | --- | --- |
| LiDAR ranges | `/writer/scan`, `sensor_msgs/msg/LaserScan` | Missing, stale, disconnected and invalid values are distinct. Positive infinity represents a no-return sample; NaN and negative infinity are rejected. |
| IMU acceleration/angular velocity | Proposed `sensor_msgs/msg/Imu` input; no current Writer consumer/topic is confirmed | Finite six-axis sample only; topic, quaternion, QoS and calibration remain unresolved. |
| Odometry | `/writer/odom`, `nav_msgs/msg/Odometry`, `writer/odom` frame | Test requires canonical frame and finite planar pose; age and connection state are explicit. Hardware frame/fusion details remain open. |
| Radio quality | `/living_map/radio/writer_link`, `std_msgs/msg/Float32` | Test assumes the simulation's normalized 0–1 probability. Field metric and calibration are unresolved. |
| Battery | Existing `RobotStatus.battery_percent` represents the simulation's percentage | Test fixture assumes percent in [0,100]; the real battery telemetry interface and sudden-drop policy are unresolved. |
| Beacon acknowledgment | Existing logical `DeployBeacon` service reports success/id/message | Mock models delayed, acknowledged, rejected, timed-out, malformed and disconnected responses. It does not pretend that current simulation response proves physical puck release. |

## Adapter behavior

Every input adapter must preserve received timestamps and expose
`READY`, `MISSING`, `STALE`, `INVALID` or `DISCONNECTED`. It must never replace
missing data with zero-valued pose, range, link quality or battery. Sensor
timeouts should be configurable only after Team 1 confirms publication rates
and time sources. The mock channel's one-second default is a test fixture, not
a proposed hardware timeout.

The beacon mock models an asynchronous service deadline. A late response after
timeout does not change the terminal result. A future hardware adapter must
separate “request accepted,” “mechanism actuated,” “beacon woke,” and “radio
advertisement heard” if those are distinct device events; Team 1 must confirm
which acknowledgment is actually available.

## Validation and assumptions

Offline tests cover absent/disconnected devices, stale and future receipt
times, NaN/infinity/out-of-range values, a sudden battery reading change,
delayed acknowledgment, timeout, late response, rejection and malformed
beacon IDs. The test-only shape assumes current ROS semantic units where known.
It is not a generated ROS interface, hardware protocol, calibrated sensor
model, or physical acceptance test.

Before implementing production adapters, Team 1 must provide topic/service
contracts, frame and unit definitions, QoS/rates, timestamp/covariance rules,
command stop lease, battery measurement semantics, radio metric semantics,
and dispenser acknowledgment/retry behavior. Keep all output paths disabled in
mock mode.
