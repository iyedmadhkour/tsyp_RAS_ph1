# Team 4 ROS Interface and Team 1 Hardware Dependency Checklist

Status: **interface review pending Team 1 confirmation; physical acceptance
blocked.** This sheet records existing software contracts and open hardware
decisions. Do not infer that simulation settings are approved hardware limits.

| Area | Current Team 4 contract | Team 1 must supply/confirm | Gate |
| --- | --- | --- | --- |
| ROS/runtime | ROS 2 Jazzy packages; namespaced Writer/Nav2 graph | Exact ROS distro/patch level, DDS/RMW, domain, namespace, QoS and lifecycle owner | Before driver integration |
| LiDAR | `/writer/scan`, `sensor_msgs/msg/LaserScan`, frame `writer/laser` | Driver name/topic, frame/extrinsic, scan direction, FOV, resolution, units, min/max, invalid values, timestamp clock, update frequency, QoS, transport delay | Before SLAM/Nav2 HIL |
| Fused pose | `/writer/odom`, `nav_msgs/msg/Odometry`; expected frames `writer/odom` → `writer/base_link` | Encoder/IMU fusion owner, covariance and validity semantics, rate, timestamp source, reset behavior, TF publisher, dropout/reorder detection, physical error data | Before any autonomous goal |
| TF and map | `map → writer/odom → writer/base_link → writer/laser`; `/map` is OccupancyGrid | Authoritative publishers, static extrinsics, transform latency, SLAM correction and localization-loss behavior | Before boundary/HOME test |
| Localization integrity | Age gate currently uses 0.5 s pose age; 0.05 m uncertainty in entrance model | Approved age/error/covariance bounds and detection for fresh-stamped frozen, replayed or drifting estimate; safe response/clear condition | **Open blocker after frozen-pose boundary failure** |
| Scan freshness | Current simulation config uses 0.35 s for Writer scan age; older configs/tests may use 0.5 s | Measured rate, max latency/jitter, allowed outage and Nav2 costmap timeout | Before autonomous test |
| Base commands | `/writer/cmd_vel`, `geometry_msgs/msg/Twist` | Mux owner, arbitration priorities, velocity/acceleration limits, lease timeout, controller heartbeat, watchdog state, physical stop behavior | Before wheel-on-ground test |
| Stop acknowledgment | No hardware acknowledgment in current Writer contract | Typed request/ack, applied-stop state, measured wheel velocity, timestamp, error/fault status and late response rules | **Must precede cancel/stop acceptance** |
| Braking/E-stop | Simulation publishes zero Twist on STUCK/FAILED; this is not physical braking evidence | Hardwired E-stop, host-loss behavior, stop latency/distance measurements, latch/reset procedure, independent operator stop | Before motion tests |
| Nav2 action | `/writer/navigate_to_pose`, `nav2_msgs/action/NavigateToPose` | Action owner, cancel → command mux → controller acknowledgment chain, stuck/lost action recovery and driver interaction | Before Nav2 HIL |
| Battery | `RobotStatus.battery_percent`; simulated model | Measured telemetry topic/service/type, units, timestamp/validity, uncertainty, sudden-drop policy, reserve calibration, temperature/load behavior | Before battery HOME acceptance |
| Event sensors | `/writer/detected_event`, `living_map_interfaces/msg/DetectedEvent` | Actual camera/gas/flame driver, calibration, coordinate frame, confidence meaning, timestamp, invalid state | Before event/beacon HIL |
| Beacon request | `/living_map/deploy_beacon`, `DeployBeacon.srv` | Actuator command, physical release/boot/advertisement acknowledgment, idempotency, jam detection and retries | Before dispenser test |
| RF | `/living_map/radio/writer_link` currently normalized Float32; `inside/rf_tx`/gateway RX use ByteMultiArray | Radio hardware/protocol, metric and units, calibration, packet limits, CRC, sequence, ack/retry, stale link and clock | Before RF HIL |
| Writer STUCK | `/living_map/writer_stuck` Bool; status string STUCK; detection uses commanded motion vs odometry | Independent motion signal, progress bound, state latch/clear, operator recovery; confirm safety behavior if odometry itself freezes | Before immobilization acceptance |
| Writer FAILED/DESTROYED | `writer_failed` Bool; RobotStatus `FAILED`; no DESTROYED enum | Physical fatal/destroyed trigger, last-gasp contract, radio silence, persistent beacon semantics, reset/restart policy | Before emergency acceptance |
| Mission/status | `/living_map/start_mission` StartMission; `RobotStatus`, explorer state and summary | Which interfaces are operator-facing/authoritative, heartbeat loss behavior, typed-state needs and audit logging | Before integrated mission |
| HIL procedure | Software-only mocks and Gazebo evidence exist | Named safety operator, restraint/zone, approved thresholds, independent pose/velocity measurement, abort triggers and sign-off | Before all physical testing |

## Current simulation-only values (not approved hardware limits)

- Pose age: 0.5 s; scan age: 0.35 s in the validated Writer config.
- Entrance position uncertainty: 0.05 m; footprint padding: 0.01 m.
- Gazebo model scan assumption: 10 Hz; model maximum linear speed: 1.2 m/s.
- Nav2 action response deadline: 3 s steady wall time.
- Test-only delayed pose injection: 1.5 s.
- Seeded software uncertainty study: ±0.05 m position and ±5° yaw.

These values must not be copied into a robot deployment profile without measured
data and Team 1/Team 4 approval. See the frozen-pose failure in
`team4-final-software-validation-20261120.md`.
