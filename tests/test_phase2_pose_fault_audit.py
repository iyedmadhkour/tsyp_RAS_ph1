from pathlib import Path


def test_frozen_pose_audit_applies_the_matching_translation_or_rotation_criteria(
        monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    from analyze_phase2_pose_fault_evidence import _frozen_phase_pass

    common = {
        "fresh_stamped_pose_mismatch_latched": True,
        "detection_delay_after_injection_s": 0.5,
        "no_new_frontier_goal_while_locked": True,
        "explicit_recovery_acknowledged": True,
        "home_after_operator_ack": True,
        "lockout_preceded_any_center_boundary_crossing": True,
        "frozen_stream": {
            "freshly_stamped_pose_stream": True,
            "reported_pose_frozen": True,
            "independent_reference_yaw_range_rad": 0.04,
        },
    }
    translation = {
        **common,
        "pre_fault_motion_measurement": 0.04,
        "lockout_translation_disagreement_m": 0.06,
        "lockout_rotation_disagreement_rad": None,
        "pre_fault_rotation_evidence": None,
    }
    rotation = {
        **common,
        "pre_fault_motion_measurement": 0.1,
        "lockout_translation_disagreement_m": None,
        "lockout_rotation_disagreement_rad": 0.03,
        "pre_fault_rotation_evidence": True,
    }

    assert _frozen_phase_pass(
        "fresh-stamped-frozen-translation", translation, 0.05, 0.02)
    assert _frozen_phase_pass(
        "fresh-stamped-frozen-rotation", rotation, 0.05, 0.02)
