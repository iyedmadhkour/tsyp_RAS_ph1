"""Emergency lifetimes and recovery, without starting ROS nodes or hardware."""
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from living_map_writer.node import Writer


def writer():
    now = [10.]
    def stamp():
        sec = int(now[0])
        return Time(sec=sec, nanosec=round((now[0] - sec) * 1e9))
    w = SimpleNamespace(
        failed=False, failed_at=None, failure_reason="", stuck=False, stuck_reason="",
        enabled=True, explorer_state="NAVIGATING", sortie_started=True,
        cmd=Mock(), status=Mock(), failed_pub=Mock(), stuck_pub=Mock(), enabled_pub=Mock(),
        detector=Mock(), battery=SimpleNamespace(level=100.), _update_battery=Mock(),
        map_pose=lambda: Pose(), _now_s=lambda: now[0],
        get_parameter=lambda name: SimpleNamespace(value=3.), get_logger=Mock(return_value=Mock()),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=stamp)))
    for name in ["_state", "_go_down", "_become_stuck"]:
        setattr(w, name, MethodType(getattr(Writer, name), w))
    w._sync_detector = lambda: False
    return w, now


def test_repeated_failures_preserve_first_deadline_and_failure_reason():
    w, now = writer()
    assert Writer.fail(w, Trigger.Request(), Trigger.Response()).success
    now[0] = 12.9
    Writer._go_down(w, "battery depleted")
    now[0] = 13.2
    assert Writer.fail(w, Trigger.Request(), Trigger.Response()).success
    assert w.failed_at == 10.
    assert w.failure_reason == "destroyed (fault injection)"
    w.failed_pub.publish.assert_called_once()


def test_repeated_fail_does_not_extend_six_half_second_failed_transmissions():
    w, now = writer()
    Writer.fail(w, Trigger.Request(), Trigger.Response())
    for t in [10.5, 11., 11.5, 12., 12.5, 13.]:
        now[0] = t
        Writer.tick(w)
        Writer.fail(w, Trigger.Request(), Trigger.Response())
    assert w.status.publish.call_count == 6
    assert all(call.args[0].state == "FAILED" for call in w.status.publish.call_args_list)
    for t in [13.5, 14., 15.]:
        now[0] = t
        Writer.tick(w)
    assert w.status.publish.call_count == 6


def test_stuck_is_idempotent_keeps_reporting_and_can_be_freed():
    w, now = writer()
    for _ in range(3):
        response = Writer.set_stuck(w, SetBool.Request(data=True), SetBool.Response())
        assert response.success
    assert w.stuck and w._state()[0] == "STUCK"
    w.stuck_pub.publish.assert_called_once()
    for t in [10.5, 11., 20.]:
        now[0] = t
        Writer.tick(w)
    assert all(call.args[0].state == "STUCK" for call in w.status.publish.call_args_list)
    assert not Writer.set_enabled(w, SetBool.Request(data=True), SetBool.Response()).success
    response = Writer.set_stuck(w, SetBool.Request(data=False), SetBool.Response())
    assert response.success and not w.stuck
    w.detector.reset.assert_called_once()
    assert Writer.set_enabled(w, SetBool.Request(data=True), SetBool.Response()).success


def test_destroyed_writer_cannot_be_freed_or_reenabled_by_late_return_failure():
    w, _ = writer()
    Writer.fail(w, Trigger.Request(), Trigger.Response())
    assert not Writer.set_stuck(w, SetBool.Request(data=False), SetBool.Response()).success
    assert not Writer.set_enabled(w, SetBool.Request(data=True), SetBool.Response()).success
    Writer.on_explorer_state(w, String(data="RETURN_FAILED"))
    assert w.failed and not w.stuck
    assert w._state()[0] == "FAILED"
    w.stuck_pub.publish.assert_not_called()
