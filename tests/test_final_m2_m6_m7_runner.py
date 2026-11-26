from __future__ import annotations

from src.experiments.run_final_m2_m6_m7_720 import (
    METHODS,
    _customer_metric_summary_fields,
    _merge_search_and_objective_components,
    _method_name,
)


def test_m1_reporting_name_does_not_expand_new_run_methods() -> None:
    assert _method_name("M1") == "truck_only"
    assert set(METHODS) == {"M2", "M6", "M7"}


def test_customer_metrics_are_flattened_to_summary_columns() -> None:
    fields = _customer_metric_summary_fields(
        {
            "high_priority_tardiness_minutes": 12.0,
            "medium_priority_tardiness_minutes": 5.0,
            "low_priority_tardiness_minutes": 1.0,
            "high_priority_on_time_rate": 0.75,
            "high_priority_mean_delivery_time": 42.0,
        }
    )

    assert fields == {
        "high_priority_tardiness": 12.0,
        "medium_priority_tardiness_minutes": 5.0,
        "low_priority_tardiness_minutes": 1.0,
        "high_priority_on_time_rate": 0.75,
        "mean_high_priority_delivery_time": 42.0,
    }


def test_objective_revalidation_preserves_ablation_and_backbone_diagnostics() -> None:
    search_components = {
        "initial_backbone_reused": 1.0,
        "local_search_enabled": 0.0,
        "perturbation_enabled": 1.0,
        "iterations": 25.0,
        "truck_cost": 99.0,
    }
    objective_components = {"truck_cost": 12.0, "drone_cost": 3.0, "tardiness_cost": 4.0}

    merged = _merge_search_and_objective_components(search_components, objective_components)

    assert merged["initial_backbone_reused"] == 1.0
    assert merged["local_search_enabled"] == 0.0
    assert merged["perturbation_enabled"] == 1.0
    assert merged["iterations"] == 25.0
    assert merged["truck_cost"] == 12.0
