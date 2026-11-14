from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("living_map_interfaces.msg")
from living_map_interfaces.msg import DetectedEvent, RobotStatus
from living_map_interfaces.srv import DeployBeacon
from living_map_beacons.node import BeaconManager
from living_map_beacons.placement import PlacementConfig, PlacementPolicy
from living_map_beacons.protocol import BeaconPacket, FLAG_ENTRANCE, target_position


def manager(capacity=1, items=None):
    e = SimpleNamespace(
        policy=PlacementPolicy(PlacementConfig(capacity=capacity)),
        items=items if items is not None else {}, seq=1, writer_pose=(0., 0.),
        writer_state="ONLINE", writer_failed=False, get_logger=lambda: Mock(),
        get_parameter=lambda name: SimpleNamespace(value=1.),
        _publish_memory=Mock(), _publish_frame=Mock(), _deploy=Mock(return_value=2))
    def store(packet):
        e.items[packet.beacon_id] = packet
    e._store_and_send = Mock(side_effect=store)
    for name in ["_duplicate", "_reuse_navigation_beacon", "_deploy_event", "_record_event"]:
        setattr(e, name, MethodType(getattr(BeaconManager, name), e))
    return e


def event(x=2.):
    e = DetectedEvent()
    e.event_type, e.event_id, e.confidence = DetectedEvent.VICTIM, "TEST", 1.
    e.pose.header.frame_id = "map"
    e.pose.pose.position.x = x
    return e


def test_empty_magazine_can_rewrite_a_nearby_navigation_beacon():
    packet = BeaconPacket(beacon_id=1, flags=FLAG_ENTRANCE,
                          previous_beacon_id=7, next_beacon_id=9)
    e = manager(items={1: packet})
    response = BeaconManager.on_deploy_service(e, DeployBeacon.Request(event=event()),
                                              DeployBeacon.Response())
    assert response.success and response.beacon_id == 1
    assert len(e.items) == 1 and e.policy.remaining(len(e.items)) == 0
    e._deploy.assert_not_called()
    assert e.items[1].previous_beacon_id == 7 and e.items[1].next_beacon_id == 9
    assert e.items[1].flags & FLAG_ENTRANCE
    assert target_position(e.items[1]) == pytest.approx((2., 0.))


def test_empty_magazine_rejects_a_new_drop_without_altering_memories():
    packet = BeaconPacket(beacon_id=1, x_m=5.)
    e = manager(items={1: packet})
    response = BeaconManager.on_deploy_service(e, DeployBeacon.Request(event=event()),
                                              DeployBeacon.Response())
    assert not response.success and response.beacon_id == 0
    assert e.items == {1: packet}
    e._deploy.assert_not_called()
    e._store_and_send.assert_not_called()


def test_rejected_backend_does_not_prevent_a_later_successful_request():
    e = manager()
    e._deploy.side_effect = [None, 1]
    request = DeployBeacon.Request(event=event())
    first = BeaconManager.on_deploy_service(e, request, DeployBeacon.Response())
    second = BeaconManager.on_deploy_service(e, request, DeployBeacon.Response())
    assert not first.success and first.beacon_id == 0
    assert second.success and second.beacon_id == 1
    assert e.writer_state == "ONLINE"


@pytest.mark.parametrize("state", ["FAILED", "STUCK", "PAUSED", "HOME"])
def test_non_exploring_status_does_not_trigger_navigation_drops(state):
    e = manager(items={1: BeaconPacket(beacon_id=1)})
    status = RobotStatus(robot_id="writer", state=state)
    status.pose.header.frame_id = "map"
    BeaconManager.on_writer_status(e, status)
    e._deploy.assert_not_called()
    # Existing beacons remain independent of Writer emergency state.
    BeaconManager.rebroadcast_all(e)
    e._publish_frame.assert_called_once()
