from __future__ import annotations

from src.data_models import SolutionData
from src.heuristics import baselines
from src.instance_generator import InstanceGenerator
from src.parameters import GenerationSettings, HeuristicConfig, SearchConfig


def _configuration() -> SearchConfig:
    return SearchConfig(
        seed=2026,
        generation=GenerationSettings(
            num_customers=4,
            num_trucks=2,
            num_drones=2,
            region="dense_urban",
            priority_share=(0.25, 0.25, 0.50),
        ),
        heuristics=HeuristicConfig(
            max_outer_iter=4,
            max_no_improve=2,
            time_limit_seconds=120,
            random_seed=2026,
            max_displacement_candidates=100,
            max_repair_candidates=100,
        ),
    )


def test_nils_ablation_wrappers_accept_matched_seed_and_backbone(monkeypatch) -> None:
    configuration = _configuration()
    instance = InstanceGenerator.from_search_config(configuration).generate_single(
        77, "matched_ablation_input"
    )
    initial_routes = {1: [0, 1, 2, 0], 2: [0, 3, 4, 0]}
    calls = []

    def fake_run_nils(instance_arg, **kwargs):
        calls.append(kwargs)
        return SolutionData(
            instance_name=instance_arg.name,
            status="test",
            objective=1.0,
            components={},
        )

    monkeypatch.setattr(baselines, "run_nils", fake_run_nils)

    baselines.nils_no_local_search_baseline(
        instance, configuration, initial_truck_routes=initial_routes, seed=2026
    )
    baselines.nils_no_perturbation_baseline(
        instance, configuration, initial_truck_routes=initial_routes, seed=2026
    )

    local_search_call, perturbation_call = calls
    assert local_search_call["seed"] == perturbation_call["seed"] == 2026
    assert local_search_call["initial_truck_routes"] == initial_routes
    assert perturbation_call["initial_truck_routes"] == initial_routes
    assert local_search_call["enable_local_search"] is False
    assert "enable_perturbation" not in local_search_call
    assert perturbation_call["enable_perturbation"] is False
    assert "enable_local_search" not in perturbation_call
    assert local_search_call["max_displacement_candidates"] == 100
    assert perturbation_call["max_repair_candidates"] == 100
