import pandas as pd
import pytest

from src.experiments.postprocess_priority_customer_analysis import (
    TIE_TOLERANCE,
    _comparison_counts,
    _statistical_test,
    reconstruct_customer_delivery,
)


def _solution_payload(truck_assignments, drone_assignments, truck_times, drone_times, tardiness):
    return {
        "u_truck": [{"key": list(key), "value": value} for key, value in truck_assignments.items()],
        "u_drone": [{"key": list(key), "value": value} for key, value in drone_assignments.items()],
        "a_truck": [{"key": list(key), "value": value} for key, value in truck_times.items()],
        "a_drone": [{"key": list(key), "value": value} for key, value in drone_times.items()],
        "tardiness": {str(key): value for key, value in tardiness.items()},
    }


def test_reconstructs_active_truck_and_drone_delivery_variables():
    payload = _solution_payload(
        {(1, 1): 1, (1, 2): 0},
        {(1, 1): 0, (2, 2): 1},
        {(1, 1): 12.5, (1, 2): 0},
        {(1, 1): 0, (2, 2): 20.0},
        {1: 0, 2: 3},
    )

    result = reconstruct_customer_delivery(payload, [1, 2])

    assert result[1] == {
        "service_mode": "truck",
        "serving_vehicle_id": 1,
        "delivery_time": 12.5,
        "stored_tardiness": 0.0,
    }
    assert result[2] == {
        "service_mode": "drone",
        "serving_vehicle_id": 2,
        "delivery_time": 20.0,
        "stored_tardiness": 3.0,
    }


def test_reconstruction_rejects_competing_truck_and_drone_assignments():
    payload = _solution_payload(
        {(1, 1): 1}, {(1, 1): 1}, {(1, 1): 10}, {(1, 1): 11}, {1: 0}
    )

    with pytest.raises(ValueError, match="has 2 active assignments"):
        reconstruct_customer_delivery(payload, [1])


def test_earlier_later_tie_counts_apply_the_documented_tolerance():
    frame = pd.DataFrame({"advantage": [5.0, -2.0, TIE_TOLERANCE / 2]})

    counts = _comparison_counts(frame, "advantage")

    assert counts["nils_earlier_count"] == 1
    assert counts["alns_earlier_count"] == 1
    assert counts["tie_count"] == 1


def test_statistical_test_keeps_empty_high_priority_instances_as_zero_difference_units():
    frame = pd.DataFrame({
        "customer_earlier_fraction_difference_nils_minus_alns": [0.5, -0.25, 0.0],
        "no_high_priority_customers": [False, False, True],
    })

    result = _statistical_test(frame).iloc[0]

    assert result.independent_paired_units == 3
    assert result.instances_without_high_priority_customers == 1
    assert result.nonzero_instance_pairs == 2
