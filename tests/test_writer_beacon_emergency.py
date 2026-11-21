"""Destruction blocks Writer deployment, not independent beacon operation."""
from dataclasses import replace

import pytest

pytest.importorskip("rclpy")
from std_msgs.msg import Bool, ByteMultiArray
from living_map_beacons.node import BeaconManager
from living_map_beacons.protocol import FLAG_RESOLVED, encode
from test_writer_beacon_failures import manager, deploy, request, status


@pytest.mark.parametrize("existing", [False, True])
def test_destroyed_writer_cannot_create_or_rewrite_event_memories(existing):
    e = manager(existing=existing)
    e.writer_state = "FAILED"
    original = dict(e.items)
    response = deploy(e)
    assert not response.success and response.beacon_id == 0
    assert "destroyed" in response.message
    BeaconManager.on_event(e, request().event)
    assert e.items == original
    e.store.save.assert_not_called()
    e._publish_memory.assert_not_called()


def test_local_failure_topic_gates_deployment_before_next_status_and_stays_latched():
    e = manager(existing=True)
    original = dict(e.items)
    BeaconManager.on_writer_failed(e, Bool(data=True))
    assert e.writer_failed and e.writer_state == "FAILED"
    assert not deploy(e).success
    BeaconManager.on_writer_status(e, status(3.))  # Late pre-failure ONLINE sample.
    assert e.writer_state == "FAILED" and e.items == original
    e.store.save.assert_not_called()
    BeaconManager.rebroadcast_all(e)
    e._publish_frame.assert_called_once()


def test_failed_status_alone_also_gates_late_online_and_event_input():
    e = manager()
    failed = status()
    failed.state = "FAILED"
    BeaconManager.on_writer_status(e, failed)
    BeaconManager.on_writer_status(e, status(4.))
    assert e.writer_failed and e.writer_state == "FAILED"
    assert not deploy(e).success and not e.items


def test_new_writer_initial_false_reopens_gate_without_erasing_deployed_beacons():
    e = manager(existing=True)
    original = dict(e.items)
    BeaconManager.on_writer_failed(e, Bool(data=True))
    BeaconManager.on_writer_failed(e, Bool(data=False))
    assert not e.writer_failed and e.writer_state == "UNKNOWN" and e.writer_pose is None
    assert e.items == original and e.prev == e.seq == 1
    BeaconManager.on_writer_status(e, status(0.))
    assert deploy(e).success  # Existing anchor can now be reused by the new Writer.
    assert len(e.items) == 1


def test_stuck_writer_can_record_an_event_and_resume_without_destroyed_reset():
    e = manager(existing=True)
    stuck = status(0.)
    stuck.state = "STUCK"
    BeaconManager.on_writer_status(e, stuck)
    assert not e.writer_failed
    assert deploy(e).success
    BeaconManager.on_writer_status(e, status(0.))
    assert e.writer_state == "ONLINE" and not e.writer_failed


def test_rescue_write_to_existing_beacon_still_works_after_writer_destroyed():
    e = manager(existing=True)
    assert deploy(e).success  # Record an event into the existing beacon.
    packet = e.items[1]
    BeaconManager.on_writer_failed(e, Bool(data=True))
    frame = encode(replace(packet, flags=packet.flags | FLAG_RESOLVED,
                           sequence=packet.sequence + 1))
    msg = ByteMultiArray()
    msg.data = frame
    BeaconManager.on_beacon_write(e, msg)
    assert e.items[1].flags & FLAG_RESOLVED
    assert e.writer_failed and len(e.items) == 1
