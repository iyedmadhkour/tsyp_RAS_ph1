"""Offline tests for the proposed, test-only Team 1 adapter boundary."""
import math

import pytest

from support.writer_hardware_adapters import (
    DeployAck,
    MockDeployService,
    MockWriterHardware,
)


def test_all_mock_channels_report_missing_then_disconnected_without_fabricating_data():
    mock = MockWriterHardware()
    assert all(channel.read(0.0).state == "disconnected"
               for channel in mock.channels.values())
    for channel in mock.channels.values():
        channel.connect()
    assert all(channel.read(0.0).state == "missing"
               for channel in mock.channels.values())
    mock.channels["lidar"].disconnect()
    assert mock.channels["lidar"].read(0.0).state == "disconnected"


def test_canonical_mock_samples_are_available_and_battery_step_is_visible():
    mock = MockWriterHardware(max_age_s=2.0)
    for channel in mock.channels.values():
        channel.connect()
    samples = {
        "lidar": (1.0, 2.0, math.inf),
        "imu": (0.0, 0.0, 9.8, 0.0, 0.0, 0.0),
        "odometry": {"frame_id": "writer/odom", "pose": (1.0, 2.0, 0.2)},
        "radio_quality": 0.87,
        "battery_percent": 80.0,
    }
    for name, value in samples.items():
        mock.channels[name].publish(value, 1.0)
        assert mock.channels[name].read(1.5).state == "ready"
    mock.channels["battery_percent"].publish(12.0, 1.6)
    assert mock.channels["battery_percent"].read(1.7).value == 12.0


@pytest.mark.parametrize(("name", "value"), [
    ("lidar", (1.0, float("nan"))),
    ("lidar", (-math.inf,)),
    ("imu", (0.0, 0.0, 0.0, 0.0, 0.0, float("nan"))),
    ("odometry", {"frame_id": "map", "pose": (0.0, 0.0, 0.0)}),
    ("odometry", {"frame_id": "writer/odom", "pose": (0.0, float("inf"), 0.0)}),
    ("radio_quality", -0.01),
    ("radio_quality", float("nan")),
    ("battery_percent", 100.1),
    ("battery_percent", float("nan")),
])
def test_invalid_sensor_and_telemetry_values_are_explicit(name, value):
    mock = MockWriterHardware()
    channel = mock.channels[name]
    channel.connect()
    channel.publish(value, 1.0)
    assert channel.read(1.1).state == "invalid"


def test_stale_or_future_dated_receipts_are_not_treated_as_current():
    mock = MockWriterHardware(max_age_s=0.5)
    odom = mock.channels["odometry"]
    odom.connect()
    value = {"frame_id": "writer/odom", "pose": (1.0, 2.0, 0.0)}
    odom.publish(value, 1.0)
    assert odom.read(1.6).state == "stale"
    assert odom.read(0.9).state == "invalid"


def test_delayed_old_sensor_message_is_stale_even_when_just_received():
    mock = MockWriterHardware(max_age_s=0.5)
    imu = mock.channels["imu"]
    imu.connect()
    imu.publish((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                received_at=10.0, source_stamp=8.0)
    assert imu.read(10.0).state == "stale"


def test_beacon_acknowledgment_supports_delayed_response_within_deadline():
    service = MockDeployService(delay_s=0.8, response=DeployAck(True, 5, "released"))
    assert service.begin(now=10.0, timeout_s=1.0) == "pending"
    assert service.poll(10.7) == "pending"
    assert service.poll(10.8) == "acknowledged"


def test_beacon_ack_timeout_is_terminal_and_late_reply_is_ignored():
    service = MockDeployService(delay_s=1.5)
    assert service.begin(now=0.0, timeout_s=1.0) == "pending"
    assert service.poll(1.0) == "timeout"
    assert service.poll(2.0) == "timeout"


def test_beacon_rejection_disconnect_and_malformed_ack_are_distinguished():
    rejected = MockDeployService(response=DeployAck(False, 0, "jammed"))
    assert rejected.begin(0.0, 1.0) == "rejected"
    disconnected = MockDeployService()
    disconnected.connected = False
    assert disconnected.begin(0.0, 1.0) == "disconnected"
    malformed = MockDeployService(response=DeployAck(True, -1, "bad id"))
    assert malformed.begin(0.0, 1.0) == "invalid"
