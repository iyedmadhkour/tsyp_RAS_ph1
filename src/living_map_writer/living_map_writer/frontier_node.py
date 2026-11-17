from __future__ import annotations

from collections import Counter
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import choose_frontier, frontier_cells
from .entrance import padded_footprint, rotation_safe_center_x
from .frontier_policy import FrontierPolicy, FrontierPolicyConfig
from .goal_watchdog import GoalWatchdog, WatchdogConfig
from .sortie import Sortie


class FrontierExplorer(Node):
    """Occupancy-grid frontier explorer using Nav2 goals in the SLAM map frame.

    When no safe reachable frontier remains, all remaining frontier candidates
    exhausted their bounded retries, or the sortie budget (battery reserve) is
    spent, the Writer drives back to the entrance: the best case, in which it
    hands over a complete map in person.
    """

    def __init__(self) -> None:
        super().__init__("frontier_explorer", namespace="/writer")
        self.map: OccupancyGrid | None = None
        self.odom_fallback = (0.0, 0.0)
        self.angular_z = 0.0
        self.busy = False
        self.failed = False
        self.stuck = False
        self.enabled = False
        self.goal_handle = None
        self.active_goal: dict | None = None
        self.feedback_distance: float | None = None
        self.feedback_recoveries = 0
        self.known_area_m2 = 0.0
        self.counts: Counter[str] = Counter()
        self.last_wait_log_key: float | None = None
        self.behavior_tree = str(Path(get_package_share_directory("living_map_writer"))
                                 / "behavior_trees" / "navigate_with_planner_fallback.xml")
        if not Path(self.behavior_tree).is_file():
            raise RuntimeError("Writer planner behavior tree is missing; build living_map_writer")

        self.declare_parameter("min_cluster_cells", 5)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("exploration_budget_s", 0.0)
        self.declare_parameter("empty_ticks_to_complete", 4)
        self.declare_parameter("max_return_attempts", 3)
        # Map-frame x of the mine entrance; the Writer explores only beyond it.
        self.declare_parameter("entrance_x", -1.0)
        self.declare_parameter("footprint", "[[0.35,0.32],[0.35,-0.32],[-0.35,-0.32],[-0.35,0.32]]")
        self.declare_parameter("footprint_padding", 0.01)
        self.declare_parameter("goal_tolerance_xy", 0.25)
        # Frontier lifecycle (Task 6). See docs/results/team4-task6-exploration.md.
        self.declare_parameter("frontier_zone_radius_m", 1.5)
        self.declare_parameter("frontier_max_attempts", 3)
        self.declare_parameter("frontier_cooldown_base_s", 15.0)
        self.declare_parameter("frontier_cooldown_max_s", 60.0)
        self.declare_parameter("frontier_exhausted_expiry_s", 120.0)
        self.declare_parameter("frontier_max_rearms", 2)
        self.declare_parameter("frontier_visited_radius_m", 1.0)
        self.declare_parameter("frontier_visited_min_age_s", 30.0)
        self.declare_parameter("frontier_visited_expiry_s", 120.0)
        self.declare_parameter("frontier_max_visits", 2)
        self.declare_parameter("frontier_map_growth_threshold_m2", 2.0)
        # Navigation-progress watchdog, distinct from the STUCK detector.
        self.declare_parameter("nav_watchdog_progress_window_s", 20.0)
        self.declare_parameter("nav_watchdog_min_progress_m", 0.25)
        self.declare_parameter("nav_watchdog_rotation_grace_s", 10.0)
        self.declare_parameter("nav_watchdog_stall_s", 45.0)
        self.declare_parameter("nav_watchdog_goal_timeout_s", 150.0)
        self.declare_parameter("nav_watchdog_home_stall_s", 90.0)
        self.declare_parameter("nav_watchdog_home_timeout_s", 300.0)
        self.declare_parameter("nav_watchdog_max_recoveries", 3)
        self.declare_parameter("nav_watchdog_reset_jump_m", 1.0)
        footprint = padded_footprint(self.get_parameter("footprint").value,
                                     float(self.get_parameter("footprint_padding").value))
        self.frontier_clearance = max(math.hypot(x, y) for x, y in footprint)
        self.minimum_goal_x = rotation_safe_center_x(
            float(self.get_parameter("entrance_x").value), footprint,
            float(self.get_parameter("goal_tolerance_xy").value))
        self.sortie = Sortie(
            float(self.get_parameter("exploration_budget_s").value),
            int(self.get_parameter("empty_ticks_to_complete").value),
            int(self.get_parameter("max_return_attempts").value),
        )
        self.policy = FrontierPolicy(FrontierPolicyConfig(
            zone_radius_m=float(self.get_parameter("frontier_zone_radius_m").value),
            max_attempts=int(self.get_parameter("frontier_max_attempts").value),
            base_cooldown_s=float(self.get_parameter("frontier_cooldown_base_s").value),
            max_cooldown_s=float(self.get_parameter("frontier_cooldown_max_s").value),
            exhausted_expiry_s=float(self.get_parameter("frontier_exhausted_expiry_s").value),
            max_rearms=int(self.get_parameter("frontier_max_rearms").value),
            visited_radius_m=float(self.get_parameter("frontier_visited_radius_m").value),
            visited_min_age_s=float(self.get_parameter("frontier_visited_min_age_s").value),
            visited_expiry_s=float(self.get_parameter("frontier_visited_expiry_s").value),
            max_visits=int(self.get_parameter("frontier_max_visits").value),
            map_growth_threshold_m2=float(
                self.get_parameter("frontier_map_growth_threshold_m2").value),
        ))
        self.watchdog = GoalWatchdog(WatchdogConfig(
            progress_window_s=float(self.get_parameter("nav_watchdog_progress_window_s").value),
            min_progress_m=float(self.get_parameter("nav_watchdog_min_progress_m").value),
            rotation_grace_s=float(self.get_parameter("nav_watchdog_rotation_grace_s").value),
            stall_timeout_s=float(self.get_parameter("nav_watchdog_stall_s").value),
            goal_timeout_s=float(self.get_parameter("nav_watchdog_goal_timeout_s").value),
            home_stall_timeout_s=float(self.get_parameter("nav_watchdog_home_stall_s").value),
            home_timeout_s=float(self.get_parameter("nav_watchdog_home_timeout_s").value),
            max_recoveries=int(self.get_parameter("nav_watchdog_max_recoveries").value),
            reset_jump_m=float(self.get_parameter("nav_watchdog_reset_jump_m").value),
        ))
        self.last_tick_s: float | None = None

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.create_subscription(Odometry, "/writer/odom", self.on_odom, 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/living_map/writer_failed", self.on_failed, state_qos)
        self.create_subscription(Bool, "/living_map/writer_enabled", self.on_enabled, state_qos)
        self.create_subscription(Bool, "/living_map/writer_stuck", self.on_stuck, state_qos)
        self.create_subscription(Bool, "/living_map/writer_return_request", self.on_return_request, state_qos)
        self.state_pub = self.create_publisher(String, "/living_map/writer_explorer_state", state_qos)
        self.summary_pub = self.create_publisher(
            String, "/living_map/writer_exploration_summary", state_qos)
        self.published_state = ""
        self.create_timer(2.0, self.tick)

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg
        resolution = float(msg.info.resolution)
        self.known_area_m2 = sum(1 for value in msg.data if value >= 0) * resolution * resolution

    def on_odom(self, msg: Odometry) -> None:
        self.odom_fallback = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.angular_z = float(msg.twist.twist.angular.z)

    def map_pose(self) -> tuple[float, float]:
        try:
            transform = self.tf_buffer.lookup_transform("map", "writer/base_link", Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException:
            return self.odom_fallback

    def _cancel(self, reason: str) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().warning(f"Active Writer goal cancelled: {reason}")
        active = getattr(self, "active_goal", None)
        if active is not None and active.get("cancel_reason") is None:
            active["cancel_reason"] = reason

    def on_enabled(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self._cancel("paused")

    def on_failed(self, msg: Bool) -> None:
        self.failed = bool(msg.data)
        if self.failed:
            self._cancel("Writer failure")

    def on_stuck(self, msg: Bool) -> None:
        self.stuck = bool(msg.data)
        if self.stuck:
            self._cancel("Writer immobilised")

    def on_return_request(self, msg: Bool) -> None:
        if msg.data and self.sortie.phase == "EXPLORING":
            self.sortie.start_return("battery reserve reached", "BATTERY_RESERVE")
            self._cancel("battery reserve reached; heading home")
            self.get_logger().warning("Battery reserve reached: returning to the entrance")

    def _publish_state(self) -> None:
        if self.failed:
            state = "HALTED"
        elif self.stuck:
            state = "STUCK"
        elif self.sortie.phase in {"RETURNING", "HOME", "RETURN_FAILED"}:
            state = self.sortie.phase
        elif not self.enabled:
            state = "PAUSED"
        else:
            state = "NAVIGATING" if self.busy else "SEARCHING"
        if state != self.published_state:
            self.published_state = state
            self.state_pub.publish(String(data=state))
            self.get_logger().info(f"Explorer state {state}")

    def _publish_summary(self) -> None:
        now = self._now_s()
        payload = {
            "phase": self.sortie.phase,
            "reason": self.sortie.reason,
            "reason_code": self.sortie.reason_code,
            "explored_s": round(self.sortie.explored_s, 2),
            "known_area_m2": round(self.known_area_m2, 3),
            "counts": dict(sorted(self.counts.items())),
            "frontier_policy": self.policy.summary(now),
            "watchdog": self.watchdog.snapshot(now),
            "active_goal": self.active_goal,
        }
        self.summary_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self) -> None:
        now = self._now_s()
        elapsed = 0.0 if self.last_tick_s is None else max(0.0, now - self.last_tick_s)
        self.last_tick_s = now
        active = self.enabled and not self.failed and not self.stuck
        if active and self.sortie.phase == "EXPLORING":
            self.sortie.spend(elapsed)
        self.policy.refresh(now, self.known_area_m2)
        self._publish_state()
        if not active:
            return
        if self.busy:
            self._update_watchdog(now)
            return
        if self.map is None or not self.nav.server_is_ready():
            return

        if self.sortie.phase == "EXPLORING" and self.sortie.budget_spent():
            self.sortie.start_return(
                "sortie budget spent; keeping the reserve to get home", "BUDGET")
            self._publish_summary()
            self.get_logger().warning("Exploration budget spent: returning to the entrance")
        if self.sortie.phase == "RETURNING":
            self._send_goal(float(self.get_parameter("home_x").value),
                            float(self.get_parameter("home_y").value), "home")
            self._publish_state()
            return
        if self.sortie.phase != "EXPLORING":
            return

        robot_x, robot_y = self.map_pose()
        visited = self.policy.visited_points(now, self.known_area_m2)
        stats: dict = {}
        candidate = self._choose(robot_x, robot_y, self.policy.blacklist(now),
                                 visited, stats)
        if candidate is None:
            self._no_candidate(now, robot_x, robot_y, visited, stats)
            return
        self.sortie.frontier_found()
        self.last_wait_log_key = None
        _score, x, y, cell_count = candidate
        self.counts["frontier_selected"] += 1
        self.get_logger().info(
            f"Frontier selected x={x:.2f} y={y:.2f} cells={cell_count} "
            f"score={_score:.2f} raw={stats.get('raw_frontier_cells')} "
            f"clusters={stats.get('clusters')} blocked_zones={len(self.policy.zones)}")
        self._send_goal(x, y, "frontier")

    def _choose(self, robot_x: float, robot_y: float, blacklist, visited,
                stats: dict | None = None):
        """Run the geometric selector with the Writer's safety filters."""
        info = self.map.info
        return choose_frontier(
            self.map.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            robot_x,
            robot_y,
            int(self.get_parameter("min_cluster_cells").value),
            # Leave room to turn the padded polygon, including occupied-cell
            # area beyond its center. Nav2 remains the live collision authority.
            goal_clearance=self.frontier_clearance + info.resolution / math.sqrt(2.0),
            blacklist=blacklist,
            blacklist_radius=self.policy.cfg.zone_radius_m,
            visited=visited,
            visited_radius=self.policy.cfg.visited_radius_m,
            min_x=float(self.get_parameter("entrance_x").value),
            boundary_clearance=self.minimum_goal_x - float(self.get_parameter("entrance_x").value),
            stats=stats,
        )

    def _no_candidate(self, now: float, robot_x: float, robot_y: float,
                      visited, stats: dict) -> None:
        # Was the failure memory the only reason nothing was selectable?
        without_blacklist = self._choose(robot_x, robot_y, (), visited)
        if without_blacklist is not None:
            x, y = without_blacklist[1], without_blacklist[2]
            covering = [zone for zone in self.policy.blocked_zones(now)
                        if zone.distance(x, y) <= self.policy.cfg.zone_radius_m]
            cooling = [zone for zone in covering if zone.cooling(now)]
            if cooling:
                # A cooling failure zone is not evidence that the mine is mapped.
                self.sortie.wait_for_retry()
                retry = self.policy.earliest_retry_s(now)
                key = None if retry is None else round(retry, 1)
                if key != self.last_wait_log_key:
                    self.last_wait_log_key = key
                    zone = min(cooling, key=lambda z: z.cooldown_until)
                    wait_s = max(0.0, zone.cooldown_until - now)
                    self.get_logger().info(
                        f"Waiting {wait_s:.0f} s for frontier cooldown at "
                        f"({zone.x:.2f},{zone.y:.2f}); attempt {zone.attempts}/"
                        f"{self.policy.cfg.max_attempts}")
                return
            code = "FRONTIERS_EXHAUSTED"
        else:
            without_memory = self._choose(robot_x, robot_y, (), ())
            if without_memory is not None:
                code = "NO_REACHABLE_FRONTIER"  # only already-visited cells remained
            elif frontier_cells(self.map.data, self.map.info.width, self.map.info.height):
                code = "NO_REACHABLE_FRONTIER"
            else:
                code = "EXPLORED"
        self.last_wait_log_key = None
        self.get_logger().info(
            f"Frontier search: raw={stats.get('raw_frontier_cells')} "
            f"clusters={stats.get('clusters')} too_small={stats.get('clusters_too_small')} "
            f"no_eligible={stats.get('clusters_without_eligible_goal')} "
            f"blocked_zones={len(self.policy.blocked_zones(now))} -> {code}")
        if self.sortie.no_frontier(code):
            self._publish_summary()
            self.get_logger().info(
                f"Exploration complete: {code} - {self.sortie.reason}; "
                "returning to the entrance")

    def _send_goal(self, x: float, y: float, kind: str) -> None:
        if x < self.minimum_goal_x:
            self.get_logger().error(f"Rejected {kind} goal: footprint/tolerance crosses entrance")
            self._goal_failed(x, y, kind, "entrance_rejected")
            return
        goal = NavigateToPose.Goal()
        goal.behavior_tree = self.behavior_tree
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # The Writer LiDAR is 360 degrees; yaw 0 keeps the existing kinematics
        # contract and does not reduce unknown-space visibility (Task 6 Phase 3).
        goal.pose.pose.orientation.w = 1.0
        self.busy = True
        self.active_goal = {"x": x, "y": y, "kind": kind,
                            "watchdog_reason": None, "cancel_reason": None}
        self.feedback_distance = None
        self.feedback_recoveries = 0
        self.watchdog.start(kind, x, y, self._now_s())
        self.nav.send_goal_async(goal, feedback_callback=self._on_feedback).add_done_callback(
            lambda future: self._goal_sent(future, x, y, kind)
        )

    def _on_feedback(self, message) -> None:
        try:
            feedback = message.feedback
            distance = float(feedback.distance_remaining)
            if math.isfinite(distance):
                self.feedback_distance = distance
            self.feedback_recoveries = int(feedback.number_of_recoveries)
        except Exception:
            pass

    def _update_watchdog(self, now: float) -> None:
        active = getattr(self, "active_goal", None)
        if (active is None or active.get("watchdog_reason") is not None
                or active.get("cancel_reason") is not None):
            return
        robot_x, robot_y = self.map_pose()
        rotating = abs(self.angular_z) >= 0.15
        verdict = self.watchdog.update(
            now, self.feedback_distance, self.feedback_recoveries,
            robot_x, robot_y, rotating)
        if verdict is None:
            return
        active["watchdog_reason"] = verdict
        snapshot = self.watchdog.snapshot(now)
        self.counts["watchdog_cancels"] += 1
        self.get_logger().warning(
            f"Navigation watchdog cancelling {active['kind']} goal at "
            f"({active['x']:.2f},{active['y']:.2f}): {verdict} after "
            f"{snapshot['elapsed_s']} s (source={snapshot['source']}, distance="
            f"{snapshot['last_distance_m']} m, recoveries="
            f"{snapshot['recoveries_seen']}, jumps={snapshot['distance_jumps']})")
        self._cancel(f"navigation watchdog: {verdict}")

    def _goal_sent(self, future, x: float, y: float, kind: str) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.busy = False
            self._goal_failed(x, y, kind, "send_failed")
            self.get_logger().warning(f"{kind} goal send failed: {exc}")
            return

        self.goal_handle = handle
        if not handle.accepted:
            self.busy = False
            self._goal_failed(x, y, kind, "rejected")
            return
        # A battery-return request may arrive while send_goal_async is pending,
        # before _cancel has a handle. Cancel that frontier once it is accepted.
        if (self.failed or self.stuck or not self.enabled
                or (kind == "frontier" and self.sortie.phase != "EXPLORING")):
            handle.cancel_goal_async()
            active = getattr(self, "active_goal", None)
            if active is not None and active.get("cancel_reason") is None:
                active["cancel_reason"] = "pending goal stopped"
        handle.get_result_async().add_done_callback(
            lambda result_future: self._done(result_future, x, y, kind)
        )

    def _goal_failed(self, x: float, y: float, kind: str, reason: str = "aborted") -> None:
        now = self._now_s()
        self.active_goal = None
        self.watchdog.reset()
        if kind == "home":
            self.counts["home_failed"] += 1
            if self.sortie.return_failed():
                self.get_logger().error(
                    "Return to the entrance failed repeatedly; Writer cannot get home")
                self._publish_summary()
            else:
                self.get_logger().warning(
                    f"Return attempt failed ({self.sortie.return_attempts_failed}/"
                    f"{self.sortie.max_return_attempts}); retrying HOME")
            return
        if self.failed or self.stuck or not self.enabled:
            return
        self.counts["frontier_failed"] += 1
        zone = self.policy.note_failure(x, y, now, self.known_area_m2, reason)
        if zone.exhausted:
            state = (f"exhausted until {zone.exhausted_until - now:.0f} s "
                     f"(or map growth); re-arms={zone.rearms}/{self.policy.cfg.max_rearms}")
        else:
            state = f"cooldown {max(0.0, zone.cooldown_until - now):.0f} s"
        self.get_logger().warning(
            f"Frontier goal failed at ({x:.2f},{y:.2f}) [{reason}]; zone attempt "
            f"{zone.attempts}/{self.policy.cfg.max_attempts}, {state}")
        self._publish_summary()

    def _done(self, future, x: float, y: float, kind: str) -> None:
        self.busy = False
        self.goal_handle = None
        active = getattr(self, "active_goal", None) or {}
        watchdog_reason = active.get("watchdog_reason")
        cancel_reason = active.get("cancel_reason")
        self.active_goal = None
        try:
            status = future.result().status
        except Exception:
            status = -1
        now = self._now_s()
        if status == 4:
            if kind == "home":
                self.counts["home_succeeded"] += 1
                self.sortie.arrived_home()
                self._publish_summary()
                self.get_logger().info("Writer is back at the entrance with the full map")
            else:
                self.counts["frontier_succeeded"] += 1
                record = self.policy.note_success(x, y, now, self.known_area_m2)
                self.get_logger().info(
                    f"Frontier goal reached at ({x:.2f},{y:.2f}); "
                    f"visit {record.count}/{self.policy.cfg.max_visits}")
        elif status == 5:
            if watchdog_reason is not None:
                self._goal_failed(x, y, kind, f"watchdog_{watchdog_reason}")
            else:
                if kind == "home":
                    self.counts["home_cancelled"] += 1
                else:
                    self.counts["frontier_cancelled"] += 1
                    self.policy.note_cancel(x, y, now, cancel_reason or "cancelled")
                self.get_logger().info(
                    f"{kind} goal at ({x:.2f},{y:.2f}) cancelled externally "
                    f"({cancel_reason or 'no reason'}); not counted as failure")
        elif not self.failed and not self.stuck and self.enabled:
            self._goal_failed(x, y, kind, "aborted")
        self._publish_state()


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
