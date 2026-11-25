from __future__ import annotations

from src.experiments.run_final_m2_m6_m7_720 import _merge_search_and_objective_components


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
