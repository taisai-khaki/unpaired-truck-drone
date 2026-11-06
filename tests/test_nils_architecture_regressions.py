from __future__ import annotations

import json
from pathlib import Path

from src.data_models import SolutionData
from src.feasibility import is_feasible, validate_solution
from src.heuristics import nils as nils_module
from src.heuristics.architecture import count_cross_truck_recoveries, validate_drone_architecture
from src.heuristics.nils import _sync_route_assignments, default_drone_to_truck, evaluate_solution, move_to_drone
from src.instance_generator import InstanceGenerator
from src.instance_generator import load_instance_json
from src.parameters import GenerationSettings, SearchConfig


ROOT = Path(__file__).resolve().parents[1]


def _small_instance():
    config = SearchConfig(
        generation=GenerationSettings(
            num_customers=4,
            num_trucks=2,
            num_drones=1,
            region="dense_urban",
            priority_share=(0.25, 0.25, 0.50),
        )
    )
    return InstanceGenerator.from_search_config(config).generate_single(71, "architecture_regression")


def _cross_truck_solution(instance):
    solution = SolutionData(
        instance_name=instance.name,
        status="architecture_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 0], 2: [0, 3, 4, 0]},
        drone_routes={1: [0, 1, 2, 4, 0]},
        u_drone={(1, 2): 1},
        z1={(1, 1, 1): 1},
        z2={(4, 1, 2): 1},
    )
    _sync_route_assignments(solution, instance)
    solution.objective, solution.components = evaluate_solution(instance, solution)
    return solution


def test_architecture_assertions_distinguish_m3_m4_and_m5() -> None:
    instance = _small_instance()
    solution = _cross_truck_solution(instance)
    fixed_pair = SolutionData(
        instance_name=instance.name,
        status="fixed_pair_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 0], 2: [0, 4, 0]},
        drone_routes={1: [0, 1, 2, 3, 0]},
        u_drone={(1, 2): 1},
        z1={(1, 1, 1): 1},
        z2={(3, 1, 1): 1},
    )
    _sync_route_assignments(fixed_pair, instance)

    assert validate_drone_architecture(instance, solution, "M5") == []
    assert count_cross_truck_recoveries(solution) == 1
    assert any("recovers with truck 2" in item for item in validate_drone_architecture(instance, solution, "M4"))
    assert any("unpaired truck 2" in item for item in validate_drone_architecture(instance, solution, "M3", {1: 1}))
    assert validate_drone_architecture(instance, fixed_pair, "M3", {1: 1}) == []
    assert validate_drone_architecture(instance, fixed_pair, "M4") == []
    assert count_cross_truck_recoveries(fixed_pair) == 0


def test_multi_sortie_must_relaunch_at_previous_physical_recovery() -> None:
    instance = _small_instance()
    solution = SolutionData(
        instance_name=instance.name,
        status="continuity_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 0], 2: [0, 4, 0]},
        drone_routes={1: [0, 1, 2, 3, 0]},
        u_drone={(1, 2): 1},
        z1={(1, 1, 1): 1},
        z2={(3, 1, 1): 1},
        u_sync={(1, 1, 1): 1, (3, 1, 1): 1},
    )
    _sync_route_assignments(solution, instance)

    continuous = move_to_drone(
        instance,
        solution,
        4,
        1,
        launch_node=3,
        recovery_node=0,
        source_truck=2,
        launch_truck=1,
        recovery_truck=2,
        enforce_battery=False,
    )
    teleport = move_to_drone(
        instance,
        solution,
        4,
        1,
        launch_node=1,
        recovery_node=0,
        source_truck=2,
        launch_truck=1,
        recovery_truck=2,
        enforce_battery=False,
    )

    assert continuous is not None
    assert continuous.drone_routes[1] == [0, 1, 2, 3, 4, 0]
    assert validate_drone_architecture(instance, continuous, "M5") == []
    assert teleport is None


def test_terminal_depot_recovery_forbids_a_later_sortie() -> None:
    instance = _small_instance()
    solution = SolutionData(
        instance_name=instance.name,
        status="depot_terminal_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 0], 2: [0, 4, 0]},
        drone_routes={1: [0, 2, 0]},
        u_drone={(1, 2): 1},
    )
    _sync_route_assignments(solution, instance)

    candidate = move_to_drone(
        instance,
        solution,
        4,
        1,
        launch_node=3,
        recovery_node=0,
        source_truck=2,
        launch_truck=1,
        recovery_truck=2,
        enforce_battery=False,
    )

    assert candidate is None


def test_post_local_search_reverts_s0045_drone_shift_failure(monkeypatch) -> None:
    instance_path = ROOT / "outputs" / "full_revised_usync_20261101_paper_ready" / "raw" / "S0045_R02.json"
    backbone_path = ROOT / "outputs" / "matched_m3_m4_m5_alns_common_backbone_20261106" / "initial_truck_backbones.json"
    failure_path = ROOT / "tests" / "fixtures" / "S0045_R02_post_local_search_failure.json"
    instance = load_instance_json(instance_path)
    backbone = json.loads(backbone_path.read_text(encoding="utf-8"))["instances"][instance.name]["truck_routes"]
    truck_routes = {int(truck): list(route) for truck, route in backbone.items()}

    feasible_before = SolutionData(
        instance_name=instance.name,
        status="pre_local_search",
        objective=0.0,
        components={},
        truck_routes=truck_routes,
        drone_routes={drone: [0, 0] for drone in instance.drone_ids()},
    )
    _sync_route_assignments(feasible_before, instance)
    feasible_before.objective, feasible_before.components = evaluate_solution(instance, feasible_before)
    assert is_feasible(instance, feasible_before)

    failure = json.loads(failure_path.read_text(encoding="utf-8"))
    infeasible_after_two_opt = SolutionData(
        instance_name=instance.name,
        status="S0045_R02_saved_failure",
        objective=0.0,
        components={},
        truck_routes={int(truck): route for truck, route in failure["truck_routes"].items()},
        drone_routes={int(drone): route for drone, route in failure["drone_routes"].items()},
        u_drone={(3, customer): 1 for customer in failure["drone_customers"]},
        z1={(event["node"], event["drone"], event["truck"]): 1 for event in failure["z1"]},
        z2={(event["node"], event["drone"], event["truck"]): 1 for event in failure["z2"]},
    )
    _sync_route_assignments(infeasible_after_two_opt, instance)
    infeasible_after_two_opt.objective, infeasible_after_two_opt.components = evaluate_solution(
        instance, infeasible_after_two_opt
    )
    findings = validate_solution(instance, infeasible_after_two_opt)
    assert any(item.code == "SHIFT" and item.vehicle == "drone" for item in findings)
    monkeypatch.setattr(nils_module, "apply_two_opt_to_solution", lambda *_args: infeasible_after_two_opt)

    after, accepted = nils_module._apply_feasibility_guarded_two_opt(
        instance,
        feasible_before,
        "M3",
        default_drone_to_truck(instance),
    )

    assert not accepted
    assert after is feasible_before
    assert is_feasible(instance, after)
