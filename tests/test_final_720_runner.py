from __future__ import annotations

from copy import deepcopy

import pytest
import yaml

from src.data_models import SolutionData
from src.experiments import run_final_720_semantics_corrected as final_run


def test_final_run_config_only_disables_harness_repair_and_sets_new_output() -> None:
    original = yaml.safe_load(final_run.SOURCE_CONFIG.read_text(encoding="utf-8"))
    original_before = deepcopy(original)

    effective = final_run._make_effective_config(original)

    assert original == original_before
    assert effective["experiment"]["study_force_feasible_outputs"] is False
    assert effective["experiment"]["study_resume"] is False
    assert effective["experiment"]["output_dir"] == final_run.OUTPUT.relative_to(final_run.REPO).as_posix()
    assert effective["experiment"]["study_methods"] == final_run.METHODS
    assert effective["heuristics"] == original["heuristics"]
    for key in (
        "alns_max_iter",
        "alns_time_limit_seconds",
        "alns_max_positions_per_route",
        "alns_max_destroy",
        "alns_battery_slack_ratio",
        "battery_slack_ratio",
    ):
        assert effective["experiment"][key] == original["experiment"][key]


def test_final_run_rejects_changed_candidate_caps() -> None:
    config = yaml.safe_load(final_run.SOURCE_CONFIG.read_text(encoding="utf-8"))
    config["heuristics"]["nils"]["max_displacement_candidates"] = 101

    with pytest.raises(RuntimeError, match="NILS displacement cap"):
        final_run._assert_frozen_settings(config)


def test_paired_rank_biserial_effect_sign_matches_improvement_direction() -> None:
    import numpy as np

    assert final_run._rank_biserial_improvement(np.array([2.0, 1.0, -1.0])) > 0
    assert final_run._rank_biserial_improvement(np.array([-2.0, -1.0, 1.0])) < 0
    assert final_run._rank_biserial_improvement(np.array([0.0, 0.0])) == 0


def test_raw_solution_serializer_preserves_routes_and_tuple_indexed_events() -> None:
    solution = SolutionData(
        instance_name="S0001_R01",
        status="test",
        objective=1.0,
        components={"initial_backbone_reused": 1.0},
        truck_routes={1: [0, 1, 0]},
        drone_routes={1: [0, 2, 1, 0]},
        u_sync={(1, 1, 1): 1},
        z1={(1, 1, 1): 0},
    )

    payload = final_run._solution_payload(solution)

    assert payload["truck_routes"] == {"1": [0, 1, 0]}
    assert payload["drone_routes"] == {"1": [0, 2, 1, 0]}
    assert payload["u_sync"] == [{"key": [1, 1, 1], "value": 1}]
    assert payload["z1"] == [{"key": [1, 1, 1], "value": 0}]


def test_pairwise_table_uses_matched_valid_runs_and_keeps_size_group() -> None:
    import pandas as pd

    rows = []
    method_values = {
        "paired_baseline": 105.0,
        "no_unpairing": 95.0,
        "nils": 80.0,
        "feasibility_aware_alns": 100.0,
    }
    for instance_index in range(3):
        for method, objective in method_values.items():
            rows.append(
                {
                    "instance_id": f"I{instance_index}",
                    "method_name": method,
                    "n": 50,
                    "eligible_share_target": 0.5,
                    "objective_total": objective + instance_index,
                    "cpu_time_seconds": 2.0,
                    "elapsed_wall_time_seconds": 2.0,
                    "full_feasibility": True,
                    "architecture_valid": True,
                    "high_priority_tardiness_minutes": objective / 10,
                    "medium_priority_tardiness_minutes": objective / 20,
                    "low_priority_tardiness_minutes": objective / 30,
                    "priority_tardiness_cost": objective * 2,
                }
            )

    wide = final_run._comparison_wide(
        pd.DataFrame(rows),
        ["eligible_share_target"],
    )
    result = final_run._paired_test_rows(wide, "nils", "feasibility_aware_alns")

    assert wide["n"].eq(50).all()
    assert result["valid_pairs"] == 3
    assert result["wins_a_objective"] == 3
    assert result["wins_b_objective"] == 0
    assert result["ties_objective"] == 0


def test_evaluation_component_merge_preserves_search_diagnostics() -> None:
    search_components = {
        "initial_backbone_reused": 1.0,
        "iterations": 25.0,
        "max_displacement_candidates": 100.0,
        "truck_cost": 10.0,
    }
    objective_components = {"truck_cost": 12.0, "drone_cost": 3.0, "tardiness_cost": 4.0}

    combined = final_run._combine_search_and_objective_components(
        search_components,
        objective_components,
    )

    assert combined["initial_backbone_reused"] == 1.0
    assert combined["iterations"] == 25.0
    assert combined["max_displacement_candidates"] == 100.0
    assert combined["truck_cost"] == 12.0
    assert final_run._shared_backbone_reused(
        "paired_baseline",
        SolutionData(
            instance_name="I",
            status="raw",
            objective=0.0,
            components=search_components,
        ),
    )
    assert not final_run._shared_backbone_reused(
        "feasibility_aware_alns",
        SolutionData(
            instance_name="I",
            status="raw",
            objective=0.0,
            components=search_components,
        ),
    )
