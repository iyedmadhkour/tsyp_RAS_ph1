# LivingMap Team 4 → Team 1 Writer Integration Handoff

Date: 2026-11-20
Purpose: interface review and safe HIL planning. **No hardware has been
connected or accepted.** The Team 4 branch is
`phase2/reliability-optimization-20261120`; its latest validation is
`team4-final-software-validation-20261120.md`.

## Readiness first

Team 4 can hand over typed ROS contracts, software behavior, mocks, and
repeatable simulation evidence. Do not enable autonomous physical movement yet:
the latest fresh-stamped frozen-pose Gazebo injection triggered watchdog retries
and STUCK, but independent ground truth measured a crossing of the configured
entrance plane before STUCK. The production pose gate checks timestamp age; a
freshly stamped but motion-inconsistent estimate is not currently distinguished
from a trustworthy pose. Resolve this software/sensor contract gap and pass the
frozen-pose boundary regression before physical navigation acceptance.

## Software architecture

| Node/component | Role | Main interfaces |
| --- | --- | --- |
| `living_map_writer/writer` | Writer mission state, battery model, command progress/STUCK detection, status, return and failure signaling | `/writer/odom`, `/writer/cmd_vel`; `/living_map/writer_*`; `inside/writer_status`; enable/fail/STUCK services |
| `living_map_writer/frontier_explorer` | SLAM map/frontier selection, Nav2 goals, freshness gate, navigation watchdog, cooldown and HOME return | `/map`, `/writer/odom`, `/writer/scan`, `navigate_to_pose`; explorer state/summary |
| `living_map_writer/entrance_boundary` | Publishes the Writer-specific entrance occupancy mask to Nav2 costmaps | `/map` → `/writer/entrance_mask` |
| `living_map_writer/scan_frames` (simulation) | Canonical scan frames and simulated odometry/TF relay | `scan_raw` → `/writer/scan`; `world_odom` → `/writer/odom` and TF |
| Nav2 lifecycle, planner, controller and behavior servers | Path planning, control, recovery and `NavigateToPose` action execution | Namespaced `/writer/*`; command output `/writer/cmd_vel` |
| `living_map_beacons/beacon_manager` | Beacon memory, deployment policy and RF advertisement model | `/living_map/deploy_beacon`, event, status, link and simulated RF topics |
| Radio/gateway/link simulation | Encodes messages, simulates packet/link behavior, command-post relay | `ByteMultiArray`, status, map and link topics |
| `living_map_events/event_sensor` | Current configured event simulator | `/writer/detected_event` |

The Gazebo demo's `/writer/odom` is ideal physics-derived odometry. The
simulation's `/writer/world_odom` is a diagnostic source, not a proposed
hardware contract. `RobotStatus.battery_percent` is simulated state, not a
measured battery-driver interface.

## Confirmed ROS-facing contracts

Names are as implemented by the current launch and Python nodes. Confirm the
production namespace and QoS before deployment. Default integer queue depths
are not a field QoS specification.

| Interface | ROS type | Consumer/meaning | Team 1 confirmation |
| --- | --- | --- | --- |
| `/writer/scan` | `sensor_msgs/msg/LaserScan` | SLAM, Nav2 costmaps and freshness gate; expected frame `writer/laser` | Driver topic, frame/extrinsic, FOV, angular/range resolution, units, rate, QoS, stamp source, invalid/no-return encoding, latency |
| `/writer/odom` | `nav_msgs/msg/Odometry` | Writer state, frontier pose fallback, progress/STUCK logic; expected parent `writer/odom`, child `writer/base_link` | Encoder/IMU fusion owner, frame IDs, TF publisher, covariance semantics, update rate, clock source, reset/reorder behavior, validity/quality signaling |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transform chain for `map → writer/odom → writer/base_link → writer/laser` | Confirm authoritative publishers, static sensor extrinsics, time synchronization and behavior after localization reset |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM result used for frontiers and Nav2 costmaps; frame `map` | Confirm SLAM/localization ownership, resolution/metadata, map correction behavior, and whether map remains available when localization degrades |
| `/writer/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2-to-base command stream in current demo | Confirm command mux, topic/namespace, maximum velocities, source arbitration, watchdog lease, stop command semantics, acknowledgment, and fault reporting |
| `/writer/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Frontier/HOME goal path | Confirm Nav2 action ownership and how action cancel maps to controller/base stop; terminal result is not a motor-stop acknowledgment |
| `/living_map/writer_enabled` | `std_msgs/msg/Bool` | Mission enabled state | Confirm command-post permission/arming contract |
| `/living_map/writer_failed` | `std_msgs/msg/Bool` | Writer failure/lost state | Confirm hardware definition and source of destroyed signal |
| `/living_map/writer_stuck` | `std_msgs/msg/Bool` | Writer physically immobilized state from software progress policy | Confirm how independent wheel/base feedback establishes immobility and how it latches/clears |
| `/living_map/writer_return_request` | `std_msgs/msg/Bool` | Reserve-triggered HOME request | Confirm measured battery source, critical reserve path, and whether return remains permitted under degraded localization |
| `/living_map/writer_explorer_state` | `std_msgs/msg/String` | States include SEARCHING, NAVIGATING, RETURNING, HOME, STUCK and return-failure conditions | Confirm status mapping to operator and hardware states |
| `/living_map/writer_exploration_summary` | `std_msgs/msg/String` | JSON summary diagnostics | Confirm whether field telemetry uses this interface or a typed replacement |
| `/living_map/deploy_beacon` | `living_map_interfaces/srv/DeployBeacon` | Request contains `DetectedEvent`; response has `success`, `beacon_id`, `message` | Clarify whether success means accepted, servo moved, puck physically released, beacon booted, or beacon radio heard |
| `/writer/detected_event` | `living_map_interfaces/msg/DetectedEvent` | Event ID/type, `PoseStamped`, severity, confidence and detected time | Confirm camera/gas/flame sensor publishers, coordinate frame, calibration, confidence and timestamp semantics |
| `/living_map/inside/writer_status` | `living_map_interfaces/msg/RobotStatus` | Robot ID, state, `PoseStamped`, heartbeat, detail and `battery_percent` | Confirm battery meaning/validity; negative currently means unknown. Confirm valid field states and destroyed mapping |
| `/living_map/radio/writer_link` | `std_msgs/msg/Float32` | Current simulated normalized link probability in `[0,1]` | Confirm physical metric (RSSI, packet-delivery ratio or other), units, calibration, update period and stale/unknown representation |
| `/living_map/inside/rf_tx`, `/living_map/gateway/rf_rx` | `std_msgs/msg/ByteMultiArray` | Simulated encoded RF uplink/downlink payloads | Confirm radio protocol, packet limits, CRC, sequence/mission identity, acknowledgments, retry and storage behavior |

Existing services include `/living_map/writer/set_enabled`
(`std_srvs/srv/SetBool`), `/living_map/writer/fail`
(`std_srvs/srv/Trigger`), `/living_map/writer/set_stuck`
(`std_srvs/srv/SetBool`, currently also used for simulation fault injection),
`/living_map/reset` (`living_map_interfaces/srv/ResetSystem`) and
`/living_map/start_mission` (`living_map_interfaces/srv/StartMission`). Keep
administrative/fault-injection services unavailable to normal field control.

## Freshness, uncertainty and time assumptions

The validated Writer configuration uses a **0.5 s** fallback-pose maximum age,
**0.35 s** scan age, **0.05 m** translational entrance uncertainty, **0.01 m**
footprint padding and a **3 s steady-wall** Nav2 acceptance/cancel response
deadline. Existing STUCK and command-progress logic has its own configured
command/odometry freshness and progress windows. These values are current
software/simulation settings, **not Team 1 sensor specifications or calibrated
field limits**. Prior Gazebo model used a 10 Hz scan assumption and a 1.2 m/s
maximum model speed for conservative inter-sample motion bounds.

The age gate rejects old or invalid timestamp data but cannot alone prove that a
freshly stamped pose is physically current. The latest frozen-pose run
demonstrated this limit. Team 1 must provide sensor rates, worst-case transport
delay, clock source/synchronization, odometry covariance/error data, localization
reset behavior and an independently observable motion/pose-integrity signal.
Team 4 and Team 1 must agree on integrity thresholds and fault response before
those values are set for the robot. Do not treat the 5 cm simulation envelope
or ±5° seeded software perturbation as measured robot accuracy.

## Drive safety, stop and acknowledgment requirements

Nav2 cancellation is a ROS action operation. A terminal CANCELED result or a
published zero `Twist` is **not proof that the motor controller has stopped**.
Before HIL movement, Team 1 must supply and jointly document:

1. A single safety mux between all command sources and the motor driver.
2. A bounded command lease enforced by the base controller when commands or
   the host heartbeat disappear.
3. A higher-priority stop/E-stop path that can preempt Nav2 and remains active
   during ROS, CPU, network or localization failure.
4. A typed stop/drive acknowledgment containing at least request identity,
   accepted/applied state, controller timestamp, measured wheel/base state and
   fault reason; specify how late or missing acknowledgments are handled.
5. Measured coast/braking distance and stopping time at test speeds and battery
   conditions, with a safe test area and independent observer.
6. A localization-integrity response that prevents retries from driving using
   a fresh-stamped but frozen pose. The present Gazebo test crossed the entrance
   plane before the 45 s STUCK outcome; no physical guarantee is implied.

Command lease duration, stop timeout, safe speed, braking bounds and integrity
thresholds all require Team 1 measurements and joint approval.

## Battery, RF, beacon and emergency contracts

- **Battery:** Provide measured voltage/charge, timestamp, validity/fault,
  temperature if available, load behavior and source. Team 4's reserve is a
  modeled percentage plus distance/power policy. Team 1 must identify return
  reserve margins under expected floor, payload, battery aging and localization
  uncertainty; define a critical shutdown/E-stop state.
- **RF:** Provide physical radio protocol, packet size/rate, mission and beacon
  identity, CRC, sequence/TTL, link metric and units, receive/transmit
  acknowledgment, retries and behavior under intermittent link. The simulation
  uses normalized probability and byte-array payloads; that is not calibrated
  RSSI or a field radio contract.
- **Relay beacons:** The existing `DeployBeacon` service is a logical request.
  Agree whether the device reports accepted, servo motion, confirmed release,
  boot, or received advertisement. Define idempotency, jam detection, bounded
  retry and safe recovery, and map the physical result to typed ROS state.
- **STUCK:** Current software infers commanded motion without measured odometry
  progress over a configured policy window and publishes STUCK while preserving
  radio relay behavior. Confirm independent wheel/base sensing, command source,
  status latch/clear and recovery policy. The detector cannot distinguish a
  frozen reported pose from physical immobilization by timestamps alone.
- **DESTROYED/FAILED:** `RobotStatus.state` has no literal `DESTROYED` enum;
  existing simulation uses `FAILED`/`writer_failed` for lost/destroyed behavior.
  Team 1 must define actual destruction/fatal-fault signal, last-gasp data,
  radio silence timing, beacon persistence, restart policy and whether
  `FAILED` is the accepted mapping.
- **Emergency:** Specify hardwired E-stop, stop-latch reset, manual recovery,
  operator authority, safe status/telemetry, and how loss of ROS or RF affects
  base drive. These are not demonstrated by software simulation.

## Recommended HIL order and acceptance evidence

Run in a controlled, physically restrained or low-energy area; do not begin with
autonomous exploration.

1. **Interface-only, outputs disabled:** Verify topic types, frame tree,
   timestamp progression, QoS, rate and covariance against signed interface
   values. Inject missing, delayed, out-of-order and invalid sensor samples.
2. **Stationary sensors:** Verify LiDAR orientation/range/invalid encoding and
   odometry consistency against independent measurement. Test localization
   reset and deliberately frozen/replayed odometry with timestamps advancing.
   Acceptance: fault is detected by the agreed integrity mechanism and the
   autonomy state blocks additional motion goals.
3. **Drive lease and stop:** With wheels lifted or a safe restraint, issue
   bounded low-speed commands. Remove host commands, ROS process and network in
   separate cases; trigger software stop and physical E-stop. Record command,
   acknowledgment, encoder motion and stop latency. Acceptance: lease and
   measured stopping envelope satisfy the approved hazard analysis.
4. **Nav2 cancellation ambiguity:** Delay, drop and conflict action responses
   while observing driver-level command lease/ack. Acceptance: no second goal
   or unbounded drive occurs while action/stop state is unresolved; record
   physical stopping independently.
5. **Localization and boundary:** In a marked test zone, introduce bounded
   localization disturbances and freeze/replay. Measure the swept padded
   footprint with independent tracking. Acceptance: remain inside the physical
   permitted boundary or enter the confirmed stop state before its limit.
6. **Battery/HOME:** Vary measured state-of-charge and load; test reserve
   threshold, stale battery data, sudden drop, HOME route and low-power stop.
   Acceptance: agreed reserve margin and reliable entrance return.
7. **STUCK/DESTROYED and relays:** Restrain motion, interrupt sensors, trigger
   fatal faults, lose RF and test beacon release/acknowledgment. Confirm state
   reporting, last-gasp delivery, persistent memory and recovery/reset rules.
8. **Integrated mission:** Only after the prior gates pass, run a bounded
   supervised mission; save ROS bags, controller logs, calibration, test
   configuration, measured trajectories, safety-stop events and operator
   sign-off.

Every numeric acceptance threshold, frame/time contract, interface owner and
physical stop criterion above requires Team 1 confirmation before testing.
Track answers in `team4-ros-hardware-interface-checklist-20261120.md`.
