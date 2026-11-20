"""Task 6 frontier lifecycle policy: bounded retries, cooldown, expiry, re-arm."""
import pytest

from living_map_writer.frontier_policy import FrontierPolicy, FrontierPolicyConfig


def policy(**overrides):
    values = dict(zone_radius_m=1.5, max_attempts=3, base_cooldown_s=15.0,
                  max_cooldown_s=60.0, exhausted_expiry_s=120.0, max_rearms=2,
                  visited_radius_m=1.0, visited_min_age_s=30.0,
                  visited_expiry_s=120.0, max_visits=2,
                  map_growth_threshold_m2=2.0)
    values.update(overrides)
    return FrontierPolicy(FrontierPolicyConfig(**values))


def test_failure_enters_a_bounded_cooldown_instead_of_immediate_retry():
    p = policy()
    p.note_failure(3.0, 4.0, now=100.0, known_area_m2=50.0)

    assert p.blacklist(100.0) == [(3.0, 4.0)]
    assert p.blacklist(114.9) == [(3.0, 4.0)]
    assert p.blacklist(115.0) == []          # cooldown ends exactly at 15 s
    assert len(p.cooling_zones(114.9)) == 1
    assert p.cooling_zones(115.0) == []
    assert p.earliest_retry_s(100.0) == 115.0


def test_nearby_failures_share_one_zone_and_double_the_cooldown():
    p = policy()
    p.note_failure(3.0, 4.0, now=0.0, known_area_m2=50.0)
    p.note_failure(4.0, 4.0, now=1.0, known_area_m2=50.0)

    assert len(p.zones) == 1
    zone = p.zones[0]
    assert zone.attempts == 2 and zone.total_attempts == 2
    assert zone.cooldown_until == pytest.approx(31.0)  # 1 s + 2 * 15 s


def test_attempts_are_bounded_then_expire_and_rearm_only_twice():
    p = policy()
    p.note_failure(3.0, 4.0, now=0.0, known_area_m2=50.0)
    p.note_failure(3.0, 4.0, now=16.0, known_area_m2=50.0)
    p.note_failure(3.0, 4.0, now=47.0, known_area_m2=50.0)

    zone = p.zones[0]
    assert zone.exhausted and zone.attempts == 3
    assert p.blacklist(50.0) and p.earliest_retry_s(50.0) is None
    p.refresh(166.0, 50.0)                    # exhausted until 167 s
    assert zone.exhausted
    p.refresh(167.0, 50.0)                    # expiry re-arms, short cooldown
    assert not zone.exhausted and zone.rearms == 1 and zone.attempts == 0
    assert p.cooling_zones(167.0)

    # Use up the second re-arm.
    for now in (200.0, 216.0, 247.0):
        p.note_failure(3.0, 4.0, now=now, known_area_m2=50.0)
    assert zone.exhausted and zone.rearms == 1
    p.refresh(367.0, 50.0)
    assert not zone.exhausted and zone.rearms == 2

    # The third exhaustion is final for this sortie.
    for now in (382.0, 398.0, 429.0):
        p.note_failure(3.0, 4.0, now=now, known_area_m2=50.0)
    p.refresh(600.0, 50.0)
    assert zone.exhausted and zone.rearms == 2
    assert p.blacklist(600.0)


def test_earliest_expiry_ignores_finally_exhausted_zones():
    p = policy(max_rearms=1)
    for now in (0.0, 16.0, 47.0):
        p.note_failure(3.0, 4.0, now=now, known_area_m2=50.0)

    assert p.earliest_expiry_s(100.0) == 167.0
    p.refresh(167.0, 50.0)
    assert p.earliest_expiry_s(167.0) is None  # cooldown is active, not exhausted

    for now in (183.0, 199.0, 230.0):
        p.note_failure(3.0, 4.0, now=now, known_area_m2=50.0)
    assert p.earliest_expiry_s(230.0) is None  # re-arm budget consumed


def test_substantial_map_growth_rearms_an_exhausted_zone():
    p = policy()
    for now in (0.0, 1.0, 2.0):
        p.note_failure(3.0, 4.0, now=now, known_area_m2=50.0)
    assert p.zones[0].exhausted and p.zones[0].exhausted_until == 122.0

    p.refresh(10.0, 51.9)
    assert p.zones[0].exhausted            # not enough new known area
    p.refresh(11.0, 52.0)
    assert not p.zones[0].exhausted and p.zones[0].rearms == 1
    assert p.cooling_zones(11.0)


def test_cancellation_is_not_recorded_as_a_frontier_failure():
    p = policy()
    p.note_cancel(3.0, 4.0, now=10.0, reason="battery reserve reached")

    assert p.zones == [] and p.blacklist(20.0) == []
    assert p.counts["cancels"] == 1
    assert p.counts["cancel_battery reserve reached"] == 1


def test_visited_goal_is_suppressed_then_allowed_until_it_is_visited_twice():
    p = policy()
    p.note_success(5.0, 5.0, now=100.0, known_area_m2=50.0)

    assert p.visited_points(120.0, 50.0) == [(5.0, 5.0)]   # minimum age 30 s
    assert p.visited_points(131.0, 50.0) == [(5.0, 5.0)]   # < 120 s, no growth
    assert p.visited_points(221.0, 50.0) == []             # expired
    p.note_success(5.0, 5.0, now=221.0, known_area_m2=50.0)
    assert p.visited_points(1000.0, 50.0) == [(5.0, 5.0)]  # two visits: final


def test_map_growth_releases_a_visited_goal_before_its_expiry():
    p = policy()
    p.note_success(5.0, 5.0, now=100.0, known_area_m2=50.0)
    assert p.visited_points(150.0, 51.9) == [(5.0, 5.0)]
    assert p.visited_points(150.0, 52.0) == []


@pytest.mark.parametrize("kwargs", [
    dict(zone_radius_m=0), dict(visited_radius_m=0), dict(max_attempts=0),
    dict(max_rearms=-1), dict(base_cooldown_s=-1), dict(map_growth_threshold_m2=0),
    dict(zone_radius_m=float("nan")), dict(zone_radius_m=float("inf")),
    dict(base_cooldown_s=float("nan")), dict(exhausted_expiry_s=float("inf")),
    dict(map_growth_threshold_m2=float("-inf")),
    dict(max_attempts=float("nan")), dict(max_rearms=float("inf")),
    dict(max_visits=float("-inf")),
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        FrontierPolicyConfig(**kwargs)
