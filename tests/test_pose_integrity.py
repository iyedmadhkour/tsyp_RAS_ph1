from living_map_writer.pose_integrity import PoseIntegrityMonitor


def monitor():
    return PoseIntegrityMonitor(
        position_tolerance_m=0.05,
        yaw_tolerance_rad=0.02,
        max_stamp_skew_s=0.01,
    )


def pair(m, t, estimate, reference):
    m.observe_estimate(t, *estimate)
    return m.observe_reference(t, *reference)


def test_stationary_robot_with_fresh_advancing_timestamps_is_valid():
    m = monitor()
    for t in (1.00, 1.03, 1.06, 1.09):
        assert pair(m, t, (0.0, 0.0, 0.0), (14.0, -3.0, 0.0)) is None
    assert m.paired_fresh(1.09, 0.1)
    assert m.fault_reason is None


def test_different_pose_origins_are_ok_when_relative_motion_agrees():
    m = monitor()
    for index in range(6):
        t = 2.0 + index * 0.03
        x = 0.02 * index
        assert pair(m, t, (x, 0.0, 0.0), (10.0 + x, 4.0, 0.0)) is None
    assert m.fault_reason is None


def test_frozen_fresh_pose_is_detected_from_independent_translation():
    m = monitor()
    assert pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is None
    assert pair(m, 1.03, (0.0, 0.0, 0.0), (10.02, 4.0, 0.0)) is None
    reason = pair(m, 1.06, (0.0, 0.0, 0.0), (10.06, 4.0, 0.0))
    assert reason is not None
    assert "disagrees" in reason


def test_frozen_fresh_pose_is_detected_during_rotation():
    m = monitor()
    assert pair(m, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)) is None
    assert pair(m, 1.03, (0.0, 0.0, 0.0), (0.0, 0.0, 0.015)) is None
    reason = pair(m, 1.06, (0.0, 0.0, 0.0), (0.0, 0.0, 0.025))
    assert reason is not None
    assert "rotation" in reason


def test_disagreement_fault_stays_latched_until_operator_acknowledges_recovery():
    m = monitor()
    pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0))
    assert pair(m, 1.03, (0.0, 0.0, 0.0), (10.06, 4.0, 0.0)) is not None
    assert pair(m, 1.06, (0.06, 0.0, 0.0), (10.06, 4.0, 0.0)) is not None
    assert pair(m, 1.09, (0.09, 0.0, 0.0), (10.09, 4.0, 0.0)) is not None
    assert m.recovery_ready
    assert m.acknowledge_recovery()
    assert m.fault_reason is None
    assert not m.recovery_ready


def test_external_pair_freshness_fault_uses_same_explicit_recovery_gate():
    m = monitor()
    pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0))
    m.latch_fault("independent pose reference became unavailable")
    assert not m.recovery_ready
    assert pair(m, 1.03, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is not None
    assert not m.recovery_ready
    assert pair(m, 1.06, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is not None
    assert m.recovery_ready
    assert m.acknowledge_recovery()
    assert m.fault_reason is None


def test_external_fault_before_first_pair_can_recover_after_pair_baseline():
    m = monitor()
    m.latch_fault("reference missing before first paired sample")
    assert pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is not None
    assert not m.recovery_ready
    assert pair(m, 1.03, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is not None
    assert not m.recovery_ready
    assert pair(m, 1.06, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0)) is not None
    assert m.recovery_ready


def test_stale_or_unpaired_reference_is_not_ready():
    m = monitor()
    pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0))
    assert not m.paired_fresh(1.2, 0.1)
    m.observe_reference(1.2, 10.0, 4.0, 0.0)
    assert not m.paired_fresh(1.2, 0.1)  # timestamps are not paired


def test_small_delivery_skew_within_configured_pairing_allowance_is_valid():
    m = PoseIntegrityMonitor(max_stamp_skew_s=0.05)
    m.observe_estimate(0.97, 0.0, 0.0, 0.0)
    m.observe_estimate(1.03, 0.0, 0.0, 0.0)
    m.observe_reference(1.00, 10.0, 4.0, 0.0)
    assert m.paired_fresh(1.03, 0.1)
    assert m.fault_reason is None


def test_timestamp_skew_during_fast_rotation_is_compared_at_common_time():
    m = PoseIntegrityMonitor(max_stamp_skew_s=0.05)
    for t in (1.00, 1.03, 1.06):
        m.observe_estimate(t, 0.0, 0.0, t - 1.0)
    for t in (1.01, 1.04, 1.07):
        m.observe_reference(t, 10.0, 4.0, t - 1.0)
    assert m.fault_reason is None


def test_nonfinite_pose_samples_are_ignored():
    m = monitor()
    pair(m, 1.0, (0.0, 0.0, 0.0), (10.0, 4.0, 0.0))
    m.observe_estimate(1.03, float("nan"), 0.0, 0.0)
    assert m.estimate.stamp_s == 1.0
