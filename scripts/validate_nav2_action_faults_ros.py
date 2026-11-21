#!/usr/bin/env python3
"""Bounded ROS 2 action-server fault injection for Writer cancellation state.

This test uses a real rclpy NavigateToPose action transport and a simulated
server only. It has no cmd_vel publisher and cannot establish motor stopping.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Keep the action client logic under test tied to this checkout, not a possibly
# stale merged install that scripts/_env.sh may have sourced.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src/living_map_writer"))

from living_map_writer.frontier_node import FrontierExplorer
from living_map_writer.frontier_policy import FrontierPolicy
from living_map_writer.sortie import Sortie


def wait(event: threading.Event, name: str, timeout_s: float = 5.0) -> None:
    if not event.wait(timeout_s):
        raise TimeoutError(f"bounded wait expired: {name} ({timeout_s}s)")


def controller_for(goal_handle, kind: str):
    sortie = Sortie(budget_s=0)
    if kind == "home":
        sortie.start_return("test HOME recovery", "BATTERY_RESERVE")
    policy = FrontierPolicy()
    ctl = SimpleNamespace(
        goal_handle=goal_handle,
        cancel_future=None,
        busy=True,
        failed=False,
        stuck=False,
        enabled=True,
        navigation_uncertain=False,
        navigation_uncertain_reason=None,
        active_goal={"x": 0.5, "y": 0.5, "kind": kind,
                     "watchdog_reason": None, "cancel_reason": None,
                     "action_timeout_reported": False},
        counts=Counter(),
        policy=policy,
        sortie=sortie,
        watchdog=Mock(),
        known_area_m2=1.0,
        get_parameter=lambda _name: SimpleNamespace(value=3.0),
        _now_s=lambda: time.monotonic(),
        get_logger=lambda: Mock(),
        _publish_summary=Mock(),
        _publish_state=Mock(),
    )
    ctl._on_cancel_response = lambda future: FrontierExplorer._on_cancel_response(
        ctl, future)
    ctl._latch_navigation_uncertain = lambda reason: (
        FrontierExplorer._latch_navigation_uncertain(ctl, reason))
    return ctl


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    # Isolate discovery from any other ROS session. A caller may override this
    # for reproducibility, but the selected domain is always recorded.
    os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 80))
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    ros_log_dir = args.output.parent / "ros-logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_log_dir)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    server_node = Node("writer_action_fault_server")
    client_node = Node("writer_action_fault_client", namespace="/writer")
    callback_group = ReentrantCallbackGroup()
    execute_started = {0: threading.Event(), 1: threading.Event()}
    cancel_started = {0: threading.Event(), 1: threading.Event()}
    cancel_decided = {0: threading.Event(), 1: threading.Event()}
    continue_running = threading.Event()
    cancel_calls = Counter()
    server_errors: list[str] = []
    result_events: list[threading.Event] = []
    result_errors: list[str] = []
    clients: list[object] = []
    server = None

    def execute(goal_handle):
        mode = round(goal_handle.request.pose.pose.position.z)
        execute_started[mode].set()
        deadline = time.monotonic() + 12.0
        try:
            while time.monotonic() < deadline:
                if mode == 1 and cancel_decided[mode].is_set():
                    time.sleep(0.2)
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                if cancel_started[mode].is_set():
                    continue_running.set()
                time.sleep(0.01)
            goal_handle.abort()
            return NavigateToPose.Result()
        except Exception as exc:  # retain diagnostics for cleanup reporting
            server_errors.append(f"execute: {type(exc).__name__}: {exc}")
            goal_handle.abort()
            return NavigateToPose.Result()

    def cancel(goal_handle):
        mode = round(goal_handle.request.pose.pose.position.z)
        cancel_calls[mode] += 1
        cancel_started[mode].set()
        if mode == 0:
            # Simulate a server that received the request but withholds its
            # cancellation acknowledgment beyond the controller's 3 s bound.
            time.sleep(3.6)
            cancel_decided[mode].set()
            return CancelResponse.ACCEPT
        cancel_decided[mode].set()
        # Deliberately reject cancellation; execution returns late SUCCEEDED.
        return CancelResponse.REJECT

    try:
        server = ActionServer(
            server_node, NavigateToPose, "/writer/navigate_to_pose",
            execute_callback=execute,
            goal_callback=lambda _request: GoalResponse.ACCEPT,
            cancel_callback=cancel,
            callback_group=callback_group,
        )
        client = ActionClient(client_node, NavigateToPose,
                              "/writer/navigate_to_pose")
        clients.append(client)
        executor.add_node(server_node)
        executor.add_node(client_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        if not client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError("isolated NavigateToPose action server not ready")

        records = []
        for mode, kind in ((0, "frontier"), (1, "home")):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.orientation.w = 1.0
            goal.pose.pose.position.x = 0.5
            goal.pose.pose.position.y = 0.5
            goal.pose.pose.position.z = float(mode)  # test-only fault selector
            accepted_event = threading.Event()
            accepted_value: list[object] = []
            send_future = client.send_goal_async(goal)

            def accepted(future):
                try:
                    accepted_value.append(future.result())
                except Exception as exc:
                    result_errors.append(f"acceptance: {type(exc).__name__}: {exc}")
                finally:
                    accepted_event.set()

            send_future.add_done_callback(accepted)
            wait(accepted_event, "goal acceptance")
            if not accepted_value or not accepted_value[0].accepted:
                raise RuntimeError(f"test goal mode={mode} was not accepted")
            handle = accepted_value[0]
            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_events.append(result_event)
            ctl = controller_for(handle, kind)

            def result_done(future, controller=ctl, result_event=result_event,
                            x=goal.pose.pose.position.x,
                            y=goal.pose.pose.position.y, goal_kind=kind):
                try:
                    FrontierExplorer._done(controller, future, x, y, goal_kind)
                except Exception as exc:
                    result_errors.append(f"result: {type(exc).__name__}: {exc}")
                finally:
                    result_event.set()

            result_future.add_done_callback(result_done)
            wait(execute_started[mode], "server action execution")
            start = time.monotonic()
            FrontierExplorer._cancel(ctl, "injected cancellation fault")
            FrontierExplorer._cancel(ctl, "duplicate injected cancellation")
            wait(cancel_started[mode], "server receives cancel request")
            if cancel_calls[mode] != 1:
                raise AssertionError("repeated cancellation generated duplicate server requests")

            if mode == 0:
                wait(continue_running, "goal continues before delayed cancel acknowledgment")
                time.sleep(3.1)
                if ctl.navigation_uncertain is False:
                    FrontierExplorer._update_watchdog(ctl, time.monotonic())
                if not ctl.navigation_uncertain or not ctl.busy:
                    diagnostic = {
                        "navigation_uncertain": ctl.navigation_uncertain,
                        "navigation_uncertain_reason": ctl.navigation_uncertain_reason,
                        "busy": ctl.busy,
                        "cancel_sent_at": ctl.active_goal.get("cancel_sent_at"),
                        "action_timeout_reported": ctl.active_goal.get(
                            "action_timeout_reported"),
                        "cancel_future_done": ctl.cancel_future.done(),
                        "result_future_done": result_future.done(),
                        "server_cancel_calls": cancel_calls[mode],
                    }
                    raise AssertionError(
                        "cancel timeout did not latch while retaining busy: "
                        + json.dumps(diagnostic, sort_keys=True))
                if result_future.done():
                    raise AssertionError("goal terminated before delayed cancel acknowledgment")
                record = {
                    "case": "delayed_cancel_ack_then_canceled",
                    "status": "PASS",
                    "uncertain_while_server_withheld_ack": ctl.navigation_uncertain,
                    "busy_while_goal_remained_active": ctl.busy,
                    "result_pending_before_ack": not result_future.done(),
                    "duplicate_cancel_server_requests": cancel_calls[mode],
                    "timeout_seconds": 3.0,
                }
            else:
                cancel_response_event = threading.Event()
                ctl.cancel_future.add_done_callback(
                    lambda _future: cancel_response_event.set())
                wait(cancel_response_event, "rejected cancel response")
                if not ctl.navigation_uncertain or not ctl.busy:
                    raise AssertionError(
                        "rejected cancellation did not retain busy uncertain state")
                wait(result_event, "late successful HOME result after rejected cancel")
                if ctl.navigation_uncertain or ctl.busy:
                    raise AssertionError("late terminal success did not resolve action uncertainty")
                if ctl.sortie.phase != "HOME":
                    raise AssertionError("late HOME success did not complete HOME state")
                record = {
                    "case": "cancel_rejected_then_late_home_success",
                    "status": "PASS",
                    "uncertain_before_late_success": True,
                    "uncertain_cleared_by_terminal_result": not ctl.navigation_uncertain,
                    "busy_after_terminal_result": ctl.busy,
                    "home_state": ctl.sortie.phase,
                    "duplicate_cancel_server_requests": cancel_calls[mode],
                }
            wait(result_event, "terminal action result")
            record["terminal_status"] = int(result_future.result().status)
            record["elapsed_seconds"] = round(time.monotonic() - start, 3)
            if record["terminal_status"] not in (GoalStatus.STATUS_CANCELED,
                                                  GoalStatus.STATUS_SUCCEEDED):
                raise AssertionError(f"unexpected terminal action status: {record}")
            records.append(record)

        output = {
            "scenario": "writer_navigate_to_pose_cancel_fault_injection",
            "result": "PASS",
            "transport": "real rclpy ActionClient/ActionServer",
            "action": "/writer/navigate_to_pose",
            "ros_domain_id": int(os.environ["ROS_DOMAIN_ID"]),
            "server_thread_count": 4,
            "cases": records,
            "server_errors": server_errors,
            "client_errors": result_errors,
            "physical_motor_stop_guaranteed": False,
            "limitation": "ROS action state only; test server publishes no cmd_vel and cannot validate base stopping.",
        }
        if server_errors or result_errors:
            output["result"] = "FAIL"
        args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
        print(json.dumps(output, indent=2, sort_keys=True))
        return 0 if output["result"] == "PASS" else 1
    finally:
        for event in cancel_decided.values():
            event.set()
        if server is not None:
            server.destroy()
        executor.shutdown(timeout_sec=3.0)
        executor.remove_node(client_node)
        executor.remove_node(server_node)
        client_node.destroy_node()
        server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
