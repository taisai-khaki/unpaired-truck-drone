from __future__ import annotations

from dataclasses import replace

import pytest

from src.data_models import SolutionData
from src.feasibility import is_feasible
from src.heuristics.nils import (
    _sync_route_assignments,
    compute_drone_operation_metrics,
    evaluate_solution,
    move_to_drone,
    summarize_drone_usage,
)
from src.heuristics.local_search import apply_two_opt_to_solution
from src.instance_generator import InstanceGenerator
from src.parameters import GenerationSettings, SearchConfig
from src.timing import compute_synchronization_waiting


def _instance():
    config = SearchConfig(
        generation=GenerationSettings(
            num_customers=4,
            num_trucks=2,
            num_drones=1,
            region="dense_urban",
            priority_share=(0.25, 0.25, 0.50),
        )
    )
    return InstanceGenerator.from_search_config(config).generate_single(7, "timing_architecture")


def test_sync_waiting_uses_post_service_readiness() -> None:
    instance = _instance()
    solution = SolutionData(
        instance_name=instance.name,
        status="test",
        objective=0.0,
        components={},
        u_truck={(1, 1): 1},
        a_truck={(1, 1): 10.0},
        a_drone={(1, 1): 12.0},
        z1={(1, 1, 1): 1},
    )
    diagnostics = compute_synchronization_waiting(instance, solution)
    service = instance.customer_map()[1].service_time_min
    assert diagnostics["waiting_truck"][(1, 1)] == 2.0 - service
    assert diagnostics["waiting_drone"][(1, 1)] == 0.0


def test_cross_truck_sortie_is_loaded_and_reconciled() -> None:
    instance = _instance()
    solution = SolutionData(
        instance_name=instance.name,
        status="test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 2, 0], 2: [0, 3, 4, 0]},
        drone_routes={1: [0, 0]},
    )
    _sync_route_assignments(solution, instance)
    candidate = move_to_drone(
        instance,
        solution,
        2,
        1,
        launch_node=1,
        recovery_node=3,
        source_truck=1,
        launch_truck=1,
        recovery_truck=2,
    )
    assert candidate is not None
    _sync_route_assignments(candidate, instance)
    candidate.objective, candidate.components = evaluate_solution(instance, candidate)
    metrics = compute_drone_operation_metrics(instance, candidate)
    assert metrics["all_drone_service_reconciled"] is True
    assert metrics["drone_launches"] == 1
    assert metrics["drone_retrievals"] == 1
    assert is_feasible(instance, candidate)
    legacy_metrics = summarize_drone_usage(candidate)
    assert legacy_metrics["active_drones_used"] == 1
    assert legacy_metrics["drone_launches"] == 1
    assert legacy_metrics["drone_retrievals"] == 1
    route_before = list(candidate.drone_routes[1])
    optimized = apply_two_opt_to_solution(candidate, instance)
    assert optimized.drone_routes[1] == route_before


def test_handling_time_is_not_added_without_an_event() -> None:
    instance = _instance()
    solution = SolutionData(
        instance_name=instance.name,
        status="test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 0], 2: [0, 2, 3, 4, 0]},
        drone_routes={1: [0, 0]},
    )
    _sync_route_assignments(solution, instance)
    before, _ = evaluate_solution(instance, solution)
    longer_handling = replace(
        instance,
        constants=replace(instance.constants, reload_time_s=9999.0, swap_time_s=9999.0),
    )
    after, _ = evaluate_solution(longer_handling, solution)
    assert after == before


def _rendezvous_solution(instance):
    solution = SolutionData(
        instance_name=instance.name,
        status="test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 0], 2: [0, 3, 4, 0]},
        drone_routes={1: [0, 1, 2, 0]},
        u_drone={(1, 2): 1},
    )
    _sync_route_assignments(solution, instance)
    return solution


def test_rendezvous_without_reload_or_swap_charges_full_cycle() -> None:
    instance = _instance()
    solution = _rendezvous_solution(instance)

    evaluate_solution(instance, solution)

    assert solution.u_sync[(1, 1, 1)] == 1
    assert solution.z1 == {}
    assert solution.z2 == {}
    truck_service = instance.customer_map()[1].service_time_min
    truck_delta = (
        solution.l_truck[(1, 1)]
        - solution.a_truck[(1, 1)]
        - truck_service
        - solution.waiting_truck.get((1, 1), 0.0)
    )
    drone_delta = (
        solution.l_drone[(1, 1)]
        - solution.a_drone[(1, 1)]
        - solution.waiting_drone.get((1, 1), 0.0)
    )
    assert truck_delta == pytest.approx(2.0)
    assert drone_delta == pytest.approx(2.0)


def test_rendezvous_with_reload_and_swap_adds_operations_to_cycle() -> None:
    instance = _instance()
    solution = _rendezvous_solution(instance)
    solution.z1[(1, 1, 1)] = 1
    solution.z2[(1, 1, 1)] = 1

    evaluate_solution(instance, solution)
    diagnostics = compute_synchronization_waiting(instance, solution)
    assert diagnostics["events"][0]["handling_minutes"] == 4.0

    truck_service = instance.customer_map()[1].service_time_min
    truck_delta = (
        solution.l_truck[(1, 1)]
        - solution.a_truck[(1, 1)]
        - truck_service
        - solution.waiting_truck.get((1, 1), 0.0)
    )
    drone_delta = (
        solution.l_drone[(1, 1)]
        - solution.a_drone[(1, 1)]
        - solution.waiting_drone.get((1, 1), 0.0)
    )
    assert truck_delta == pytest.approx(4.0)
    assert drone_delta == pytest.approx(4.0)
