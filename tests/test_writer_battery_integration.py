"""Battery integration uses odometry for travel and map pose for HOME reserve."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import Pose
from living_map_writer.battery import Battery
from living_map_writer.node import Writer


def pose(x=0., y=0.):
    p = Pose()
    p.position.x, p.position.y = x, y
    return p


def writer(charge=100.):
    return SimpleNamespace(
        battery=Battery(level_pct=charge), last_tick_s=None,
        failed=False, sortie_started=True, last_odom_pose=pose(),
        enabled=True, explorer_state="NAVIGATING", return_requested=False,
        return_pub=Mock(), _go_down=Mock(), get_logger=Mock(return_value=Mock()),
        get_parameter=lambda name: SimpleNamespace(value=0.))


def test_stationary_robot_is_not_charged_motor_energy_for_slam_correction():
    w = writer()
    Writer._update_battery(w, 0., pose())
    Writer._update_battery(w, 1., pose(.5))  # SLAM shifts, physical odom does not.
    assert w.battery.level == pytest.approx(99.97)
    w.return_pub.publish.assert_not_called()


def test_actual_odom_travel_is_charged_even_if_map_pose_is_unchanged():
    w = writer()
    Writer._update_battery(w, 0., pose(5.))
    w.last_odom_pose = pose(1.)
    Writer._update_battery(w, 1., pose(5.))
    assert w.battery.level == pytest.approx(100. - .03 - .3)


def test_missing_then_first_odom_does_not_mix_map_and_odom_coordinates():
    w = writer()
    w.last_odom_pose = None
    Writer._update_battery(w, 0., pose(10.))
    w.last_odom_pose = pose(1.)
    Writer._update_battery(w, 1., pose(11.))
    assert w.battery.level == pytest.approx(99.97)
    w.last_odom_pose = pose(1.5)
    Writer._update_battery(w, 2., pose(17.))
    assert w.battery.level == pytest.approx(99.79)


def test_return_energy_formula_matches_documented_factor_and_floor():
    b = Battery()
    distance = 10.
    route_distance = 1.6 * distance
    expected_energy = .3 * route_distance + .03 * route_distance / .4
    assert b.energy_home((distance, 0.), (0., 0.)) == pytest.approx(expected_energy)
    assert b.reserve((distance, 0.), (0., 0.)) == pytest.approx(10. + 1.5 * expected_energy)
    assert b.reserve((distance, 0.), (0., 0.)) == pytest.approx(19.)
    assert b.reserve((0., 0.), (0., 0.)) == 10.


def test_charge_and_map_distance_changes_trigger_one_latched_return_request():
    w = writer(charge=60.)
    Writer._update_battery(w, 0., pose(10.))
    w.battery.level = 20.  # Simulated charge measurement; no runtime API assumed.
    Writer._update_battery(w, 1., pose(10.))  # Reserve 19%, still above it.
    w.return_pub.publish.assert_not_called()
    Writer._update_battery(w, 2., pose(20.))  # Map-frame HOME estimate now needs 28%.
    assert w.return_requested
    assert w.return_pub.publish.call_args.args[0].data is True
    w.battery.level = 100.
    Writer._update_battery(w, 3., pose(1.))   # Recovery does not restart exploration.
    w.battery.level = 15.
    Writer._update_battery(w, 4., pose(20.))
    w.return_pub.publish.assert_called_once()


def test_reserve_boundary_is_inclusive_and_returning_does_not_reissue_request():
    w = writer(charge=19.)
    Writer._update_battery(w, 0., pose(10.))
    assert w.return_requested
    w.explorer_state = "RETURNING"
    Writer._update_battery(w, 1., pose(9.))
    Writer._update_battery(w, 2., pose(8.))
    w.return_pub.publish.assert_called_once()


def test_standby_does_not_drain_or_request_return_before_sortie_starts():
    w = writer(charge=12.)
    w.enabled = w.sortie_started = False
    Writer._update_battery(w, 0., pose())
    Writer._update_battery(w, 100., pose(20.))
    assert w.battery.level == 12.
    w.return_pub.publish.assert_not_called()
