"""Writer-side deployment failures, without ROS node or hardware startup."""
import math
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("living_map_interfaces.msg")
from living_map_beacons.node import BeaconManager
from living_map_beacons.placement import PlacementPolicy
from living_map_beacons.protocol import BeaconPacket, FLAG_ENTRANCE, FLAG_RELAY
from living_map_interfaces.msg import DetectedEvent, RobotStatus
from living_map_interfaces.srv import DeployBeacon


def manager(existing=False):
    items = {1: BeaconPacket(beacon_id=1, sequence=1, flags=FLAG_ENTRANCE)} if existing else {}
    params = {"ttl_sec": 1800, "standoff_min_m": .3, "reuse_radius_m": 1., "entrance_x": -1.}
    e = SimpleNamespace(items=items, seq=int(existing), prev=int(existing),
                        writer_pose=(0., 0.), writer_state="ONLINE", uplink=None,
                        store=SimpleNamespace(save=Mock()), policy=PlacementPolicy(),
                        _publish_frame=Mock(), _publish_memory=Mock(),
                        _openings=lambda point: 0, get_logger=Mock(return_value=Mock()),
                        get_parameter=lambda key: SimpleNamespace(value=params[key]))
    for name in ["_duplicate", "_store_and_send", "_link_predecessor", "_deploy",
                 "_reuse_navigation_beacon", "_deploy_event", "_record_event"]:
        setattr(e, name, MethodType(getattr(BeaconManager, name), e))
    return e


def request(x=2., confidence=1.):
    event = DetectedEvent(event_id="WRITE_FAILURE_TEST", event_type=1, confidence=confidence)
    event.pose.header.frame_id = "map"
    event.pose.pose.position.x = x
    return DeployBeacon.Request(event=event)


def deploy(e, req=None):
    return BeaconManager.on_deploy_service(e, req or request(), DeployBeacon.Response())


def status(x=2.):
    msg = RobotStatus(robot_id="writer", state="ONLINE")
    msg.pose.header.frame_id = "map"
    msg.pose.pose.position.x = x
    return msg


def test_failed_initial_record_consumes_no_stock_or_chain_and_can_retry():
    e = manager()
    e.store.save.side_effect = OSError("simulated unavailable storage")
    response = deploy(e)
    assert not response.success and response.beacon_id == 0
    assert e.items == {} and e.prev == e.seq == 0
    e._publish_frame.assert_not_called()
    e._publish_memory.assert_not_called()
    e.store.save.side_effect = None
    response = deploy(e)
    assert response.success and response.beacon_id == 1
    assert len(e.items) == 1 and e.prev == e.seq == 1


def test_failed_rewrite_keeps_old_memory_and_does_not_fall_back_to_new_drop():
    e = manager(existing=True)
    original = e.items[1]
    e.store.save.side_effect = OSError("simulated unavailable storage")
    assert not deploy(e).success
    assert e.items == {1: original} and e.seq == e.prev == 1
    e._publish_frame.assert_not_called()
    e.store.save.side_effect = None
    assert deploy(e).beacon_id == 1
    assert len(e.items) == 1 and e.items[1].event_type == 1


def test_failed_relay_record_preserves_retry_and_next_success_arms_cooldown():
    e = manager(existing=True)
    e.uplink = .5
    e.store.save.side_effect = OSError("simulated unavailable storage")
    with patch("living_map_beacons.node.time.monotonic", return_value=100.):
        BeaconManager.on_writer_status(e, status())
    assert e.items.keys() == {1} and e.seq == e.prev == 1
    assert e.policy.last_relay_at == -math.inf
    e.store.save.side_effect = None
    with patch("living_map_beacons.node.time.monotonic", return_value=100.5):
        BeaconManager.on_writer_status(e, status())
    assert e.items[2].flags & FLAG_RELAY
    assert e.items[1].next_beacon_id == 2 and e.items[2].previous_beacon_id == 1
    assert e.policy.last_relay_at == 100.5


def test_rejected_relay_can_retry_without_spending_success_cooldown():
    e = manager(existing=True)
    e.uplink = .5
    e._deploy = Mock(side_effect=[None, 2])
    with patch("living_map_beacons.node.time.monotonic", return_value=100.):
        BeaconManager.on_writer_status(e, status())
    with patch("living_map_beacons.node.time.monotonic", return_value=100.5):
        BeaconManager.on_writer_status(e, status())
    assert e._deploy.call_count == 2
    assert e.policy.last_relay_at == 100.5


def test_failed_predecessor_update_does_not_undo_or_duplicate_durable_drop():
    e = manager(existing=True)
    e.writer_pose = (3., 0.)
    e.store.save.side_effect = [None, OSError("predecessor rewrite failed")]
    response = deploy(e, request(5.))
    assert response.success and response.beacon_id == 2
    assert len(e.items) == 2 and e.prev == 2 and e.seq == 2
    assert e.items[1].next_beacon_id == 0  # The failed forward-link write is explicit.
    assert e.items[2].previous_beacon_id == 1
    assert any("predecessor link update failed" in str(c)
               for c in e.get_logger().error.call_args_list)
    e.store.save.side_effect = None
    assert not deploy(e, request(5.)).success  # Duplicate event: no second puck.
    assert len(e.items) == 2


def test_invalid_wire_value_is_rejected_before_persistence():
    e = manager()
    assert not deploy(e, request(confidence=float("nan"))).success
    e.store.save.assert_not_called()
    assert not e.items and e.seq == e.prev == 0


def test_event_callback_survives_write_failure():
    e = manager()
    e.store.save.side_effect = OSError("simulated unavailable storage")
    BeaconManager.on_event(e, request().event)
    assert e.items == {} and e.seq == e.prev == 0
