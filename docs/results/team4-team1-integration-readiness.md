# Team 4 ↔ Team 1 Writer Integration Readiness

Status: **planning only; no hardware integration performed**. Contracts below
are limited to interfaces and design statements found in the repository.

## Confirmed existing ROS contracts

| Function | Existing contract | Evidence and limit |
| --- | --- | --- |
| LiDAR | `/writer/scan`: `sensor_msgs/msg/LaserScan`; frame `writer/laser` | Simulation bridge starts at `/writer/scan_raw`; `scan_frames.py` republishes with the canonical frame. No physical driver is present. |
| Odometry | `/writer/odom`: `nav_msgs/msg/Odometry`; `writer/odom` → `writer/base_link` TF | Simulation derives this from Gazebo physics pose, offsets the spawn origin, and republishes TF. It is ideal odometry, not encoder/IMU fusion. |
| Mapping | `/map`: `nav_msgs/msg/OccupancyGrid`; shared `map` frame | SLAM Toolbox consumer. Physical sensor noise/localization behavior is unvalidated. |
| Velocity command | `/writer/cmd_vel`: `geometry_msgs/msg/Twist` | Shared by Nav2, demo fallback and Writer stop commands in simulation. There is no confirmed hardware mux, driver acknowledgment, command lease or emergency-stop contract. |
| Detected event | `/writer/detected_event`: `living_map_interfaces/msg/DetectedEvent` | Current event simulator supplies configured events. Physical camera/gas/flame drivers are not present. |
| Beacon request | `/living_map/deploy_beacon`: `living_map_interfaces/srv/DeployBeacon`; request contains `DetectedEvent`, response has success, beacon ID and message | Current beacon manager creates persistent simulated memory. The response does not prove a physical servo release or puck wake-up. |
| Radio link | `/living_map/radio/writer_link`: `std_msgs/msg/Float32` | Simulated uplink probability used by placement policy; field scale, update rate, freshness and calibration are unspecified. |
| Status | `/living_map/inside/writer_status`: `living_map_interfaces/msg/RobotStatus` | Carries `map` pose, state and battery percentage; current battery is modeled. |

## Planned or unresolved inputs

The implementation plan names a 2D LiDAR, wheel encoders, IMU, camera, gas
sensor, flame sensor, ESP32 drive controller and servo dispenser. These are
planned components, not verified device specifications. This checkout has no
confirmed Writer IMU topic or `sensor_msgs/msg/Imu` consumer. The sensor
extrinsics, topic namespace, frame IDs, rates, covariances, synchronization,
and encoder/IMU fusion owner remain open. Do not assume a topic or hardware
model until Team 1 confirms it.

## Proposed future mock-adapter boundary

Keep Team 4 behavior consuming the canonical ROS contracts above. For each
device, use a replaceable adapter that can select either deterministic mock
input or the eventual Team 1 driver:

1. Sensor adapters publish `LaserScan`, `Odometry`/TF, and
   `DetectedEvent` with documented frames, timestamps, covariance, freshness,
   and invalid-data behavior.
2. A future localization adapter fuses encoders and IMU into the agreed
   odometry/TF contract. It must expose stale or invalid measurements rather
   than silently publishing plausible zeros.
3. A command adapter routes Nav2 `Twist` through a safety/velocity mux to the
   base. Its command timeout, source arbitration, stop acknowledgment and
   failure state must be agreed before the Task 6 watchdog or physical STUCK
   policy is relied upon.
4. A dispenser adapter should expose an explicit physical drop result
   (accepted, released, confirmed, timed out, jammed). Map that result to
   logical `DeployBeacon` success only after Team 1 and Team 4 agree whether the
   existing service can represent physical acknowledgment or needs a separate
   hardware-facing service.
5. A radio adapter converts measured link quality into the agreed normalized
   contract consumed by `/living_map/radio/writer_link`; retain raw RSSI,
   freshness and link-loss diagnostics separately.

Mocks must implement the same typed contracts and support deterministic
timeouts, malformed/stale samples, rejected deployment and dispenser failure.
No adapter should drive physical outputs in mock mode.

## Team 1 checklist and questions

- Confirm ROS distribution, device drivers, topic/service names, QoS, rates,
  frames, units, timestamp source and covariance for LiDAR, encoders, IMU and
  fused odometry.
- Who owns encoder/IMU fusion and the `writer/odom` → `writer/base_link` TF?
  How are dropped, stale, reset or invalid sensor messages reported?
- What command mux and motor-controller watchdog stop motion after command
  loss? Is the lease one second or another value? How are source priority,
  emergency stop and stop acknowledgment represented?
- What calibrated battery measurement/service is available, and what reserve
  uncertainty should the existing return-home model use?
- What radio measurement is available (RSSI, packet delivery, or both), with
  what units, update period, calibration and stale-data behavior?
- What does the dispenser acknowledge: servo movement, puck release, beacon
  wake, or radio advertisement? What are timeout, jam recovery and retry limits?
- Confirm physical beacon radio protocol, persistence, identity allocation,
  clock source and CRC compatibility before firmware interoperability work.
- Provide a hardware-in-loop test procedure and a safe output-disabled mock
  mode before connecting any physical robot.

Until these answers are confirmed, physical Writer, emergency, radio, battery
and beacon-deployment behavior remains **NOT TESTED**.
