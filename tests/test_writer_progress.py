"""Validation of the physical STUCK detector's bounded configuration."""
import pytest

from living_map_writer.progress import StuckDetector


@pytest.mark.parametrize("window_s", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_window_is_rejected_before_it_can_disable_sample_pruning(window_s):
    with pytest.raises(ValueError, match="finite"):
        StuckDetector(window_s=window_s)


@pytest.mark.parametrize("min_progress_m", [float("nan"), float("inf"), float("-inf"), 0.0])
def test_non_finite_or_non_positive_progress_threshold_is_rejected(min_progress_m):
    with pytest.raises(ValueError):
        StuckDetector(min_progress_m=min_progress_m)


def test_finite_non_positive_window_keeps_the_existing_explicit_disable_behavior():
    detector = StuckDetector(window_s=0.0)
    assert not detector.update(1.0, 0.0, 0.0, driving=True)
    assert len(detector.samples) == 0
