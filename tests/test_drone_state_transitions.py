from __future__ import annotations

from collections import Counter
from copy import deepcopy

import pytest

from src.data_models import Customer, Drone, InstanceData, ProblemConstants, SolutionData, Truck
from src.drone_state import reconstruct_support_events
from src.feasibility import is_feasible, validate_solution
from src.heuristics import alns as alns_module
from src.heuristics.alns import _candidate_drone_insertions, _candidate_insertions, _refresh, _repair
from src.heuristics.architecture import sortie_associations, validate_drone_architecture
from src.heuristics.nils import _sync_route_assignments, evaluate_solution


def _instance(*, max_battery_wh: float = 40.0) -> InstanceData:
    coordinates = {
        1: (1.0, 0.0),
        2: (2.0, 0.0),
        3: (3.0, 0.0),
        4: (4.0, 0.0),
        5: (0.0, 3.0),
        6: (0.0, 4.0),
    }
    customers = [
        Customer(node, x, y, 1.0, "medium", 0.0, 1000.0, 0.0)
        for node, (x, y) in coordinates.items()
    ]
    return InstanceData(
        name="drone_state_regression",
        seed=2026,
        region="test",
        customers=customers,
        trucks=[
            Truck(1, 20.0, 30.0, 1.0),
            Truck(2, 20.0, 30.0, 1.0),
        ],
        drones=[Drone(1, 5.0, 60.0, max_battery_wh, 5.0, 10.0, 0.1, parcel_weight_kg=1.0)],
        priority_penalties={"high": 10.0, "medium": 3.0, "low": 1.0},
        constants=ProblemConstants(
            swap_time_s=60.0,
            reload_time_s=60.0,
            handling_time_s=60.0,
            max_shift_minutes=480.0,
        ),
        metadata={"battery_slack_ratio": 0.1},
    )


def _same_truck_solution(instance: InstanceData) -> SolutionData:
    solution = SolutionData(
        instance_name=instance.name,
        status="state_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 5, 0], 2: [0, 6, 0]},
        drone_routes={1: [0, 1, 2, 3, 4, 0]},
    )
    _sync_route_assignments(solution, instance)
    return solution


def _truck_supported_instance(*, max_battery_wh: float = 100.0) -> InstanceData:
    coordinates = {1: (5.0, 0.0), 2: (10.0, 0.0), 3: (6.0, 0.0), 4: (0.0, 1.0)}
    customers = [
        Customer(
            node,
            x,
            y,
            1.0,
            "high" if node == 2 else "medium",
            0.0,
            15.0 if node == 2 else 1000.0,
            0.0,
        )
        for node, (x, y) in coordinates.items()
    ]
    return InstanceData(
        name="truck_supported_alns_regression",
        seed=2026,
        region="test",
        customers=customers,
        trucks=[Truck(1, 20.0, 30.0, 1.0), Truck(2, 20.0, 30.0, 1.0)],
        drones=[Drone(1, 5.0, 60.0, max_battery_wh, 5.0, 10.0, 0.01, parcel_weight_kg=1.0)],
        priority_penalties={"high": 100.0, "medium": 3.0, "low": 1.0},
        constants=ProblemConstants(
            swap_time_s=60.0,
            reload_time_s=60.0,
            handling_time_s=60.0,
            max_shift_minutes=480.0,
        ),
        metadata={"battery_slack_ratio": 0.1},
    )


def _truck_supported_partial(instance: InstanceData) -> SolutionData:
    solution = SolutionData(
        instance_name=instance.name,
        status="alns_repair_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 0], 2: [0, 4, 0]},
        drone_routes={1: [0, 0]},
    )
    _sync_route_assignments(solution, instance)
    return solution


def test_both_reload_and_battery_swap_are_required_and_feasible() -> None:
    instance = _instance()
    solution = _same_truck_solution(instance)

    analysis = reconstruct_support_events(instance, solution)

    assert solution.u_sync == {(1, 1, 1): 1, (3, 1, 1): 1}
    assert solution.z1 == {(3, 1, 1): 1}
    assert solution.z2 == {(3, 1, 1): 1}
    assert not analysis.issues
    rendezvous = {state.event_key: state for state in analysis.rendezvous}
    assert not rendezvous[(1, 1, 1)].z1_required
    assert not rendezvous[(1, 1, 1)].z2_required
    assert rendezvous[(3, 1, 1)].z1_required
    assert rendezvous[(3, 1, 1)].z2_required
    assert rendezvous[(3, 1, 1)].battery_before_support_wh == pytest.approx(15.0)
    assert rendezvous[(3, 1, 1)].energy_to_next_event_or_depot_wh == pytest.approx(30.0)
    assert is_feasible(instance, solution)
    assert validate_drone_architecture(instance, solution, "M5") == []


def test_rendezvous_needing_neither_support_is_feasible() -> None:
    instance = _truck_supported_instance(max_battery_wh=1000.0)
    solution = _truck_supported_partial(instance)
    solution.drone_routes[1] = [0, 1, 2, 3, 0]
    _sync_route_assignments(solution, instance)

    analysis = reconstruct_support_events(instance, solution)

    assert not analysis.issues
    assert solution.u_sync == {(1, 1, 1): 1, (3, 1, 1): 1}
    assert solution.z1 == {}
    assert solution.z2 == {}
    assert not analysis.required_z1
    assert not analysis.required_z2
    assert is_feasible(instance, solution)
    assert validate_drone_architecture(instance, solution, "M5") == []


def test_battery_swap_without_parcel_reload_is_required() -> None:
    instance = _truck_supported_instance()
    solution = _truck_supported_partial(instance)
    solution.drone_routes[1] = [0, 1, 2, 3, 0]
    _sync_route_assignments(solution, instance)

    analysis = reconstruct_support_events(instance, solution)

    assert not analysis.issues
    assert solution.z1 == {}
    assert solution.z2 == {(1, 1, 1): 1, (3, 1, 1): 1}
    rendezvous = {state.event_key: state for state in analysis.rendezvous}
    assert not rendezvous[(1, 1, 1)].z1_required
    assert rendezvous[(1, 1, 1)].z2_required
    assert not rendezvous[(3, 1, 1)].z1_required
    assert rendezvous[(3, 1, 1)].z2_required
    assert is_feasible(instance, solution)
    assert validate_drone_architecture(instance, solution, "M5") == []


def test_full_battery_that_cannot_cover_next_segment_is_rejected() -> None:
    instance = _instance(max_battery_wh=25.0)
    solution = _same_truck_solution(instance)

    analysis = reconstruct_support_events(instance, solution)

    assert (3, 1, 1) not in solution.z2
    assert any(issue.code == "BATTERY_SEGMENT" for issue in analysis.issues)


def test_unnecessary_launch_reload_and_recovery_swap_are_rejected() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _same_truck_solution(instance)
    solution.z1[(1, 1, 1)] = 1
    solution.z2[(3, 1, 1)] = 1

    architecture_errors = validate_drone_architecture(instance, solution, "M5")

    assert any("unnecessary parcel reload" in message for message in architecture_errors)
    assert any("unnecessary battery swap" in message for message in architecture_errors)


def test_missing_required_reload_is_caught_by_full_feasibility() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _same_truck_solution(instance)
    solution.z1 = {}

    findings = validate_solution(instance, solution)

    assert any(item.code == "PARCEL_STATE" and item.severity == "ERROR" for item in findings)


def test_reload_is_required_before_service_at_the_next_rendezvous_node() -> None:
    instance = _instance(max_battery_wh=1000.0)
    solution = SolutionData(
        instance_name=instance.name,
        status="next_event_service_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 3, 5, 0], 2: [0, 6, 0]},
        drone_routes={1: [0, 1, 2, 3, 4, 5, 0]},
    )
    _sync_route_assignments(solution, instance)

    analysis = reconstruct_support_events(instance, solution)

    assert not analysis.issues
    assert analysis.required_z1 == {(3, 1, 1): 1}
    assert analysis.required_z2 == {}
    assert solution.z1 == {(3, 1, 1): 1}
    assert solution.z2 == {}
    assert is_feasible(instance, solution)
    assert validate_drone_architecture(instance, solution, "M5") == []


def test_route_intersection_requires_explicit_usync_and_physical_handling() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _same_truck_solution(instance)
    solution.u_sync.pop((1, 1, 1))
    evaluate_solution(instance, solution)

    assert solution.l_truck[(1, 1)] - solution.a_truck[(1, 1)] == pytest.approx(2.0)
    findings = validate_solution(instance, solution)
    assert any(item.code == "SYNC_MISSING" for item in findings)


def _cross_truck_solution(instance: InstanceData) -> SolutionData:
    solution = SolutionData(
        instance_name=instance.name,
        status="cross_truck_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 5, 0], 2: [0, 3, 6, 0]},
        drone_routes={1: [0, 1, 2, 3, 4, 0]},
    )
    _sync_route_assignments(solution, instance)
    return solution


def test_architecture_checks_apply_to_rendezvous_and_support_events() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _cross_truck_solution(instance)

    assert validate_drone_architecture(instance, solution, "M5") == []
    assert any("recovers with truck 2" in item for item in validate_drone_architecture(instance, solution, "M4"))
    assert any(
        "M3 drone 1 uses truck 2, not paired truck 1" in item
        for item in validate_drone_architecture(instance, solution, "M3", {1: 1})
    )


def test_m3_rejects_unpaired_usync_event_independently() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _cross_truck_solution(instance)
    solution.z1 = {}
    solution.z2 = {}

    violations = validate_drone_architecture(instance, solution, "M3", {1: 1})

    assert any("M3 drone 1 uses truck 2, not paired truck 1" in item for item in violations)


def test_m3_rejects_unpaired_z1_event_independently() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _cross_truck_solution(instance)
    solution.z1 = {(3, 1, 2): 1}
    solution.z2 = {}

    violations = validate_drone_architecture(instance, solution, "M3", {1: 1})

    assert any("M3 drone 1 z1 uses unpaired truck 2" in item for item in violations)


def test_m3_rejects_unpaired_z2_event_independently() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _cross_truck_solution(instance)
    solution.z1 = {}
    solution.z2 = {(3, 1, 2): 1}

    violations = validate_drone_architecture(instance, solution, "M3", {1: 1})

    assert any("M3 drone 1 z2 uses unpaired truck 2" in item for item in violations)


def test_explicit_sync_without_joint_route_visit_is_rejected() -> None:
    instance = _instance(max_battery_wh=100.0)
    solution = _same_truck_solution(instance)
    solution.u_sync[(5, 1, 1)] = 1

    assert any("lacks a joint route visit" in item for item in validate_drone_architecture(instance, solution, "M5"))


def test_alns_candidate_generation_covers_all_m5_sortie_types() -> None:
    instance = _instance(max_battery_wh=1000.0)
    solution = SolutionData(
        instance_name=instance.name,
        status="alns_candidate_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 2, 3, 5, 0], 2: [0, 4, 6, 0]},
        drone_routes={1: [0, 0]},
    )
    _sync_route_assignments(solution, instance)

    candidates = _candidate_drone_insertions(
        instance,
        solution,
        customer=2,
        drone_id=1,
        beta=0.1,
        max_positions=2,
        max_candidates=None,
    )

    sortie_types = Counter()
    for candidate in candidates:
        associations = [
            item for item in sortie_associations(candidate)
            if item["drone_id"] == 1 and item["customer_id"] == 2
        ]
        assert len(associations) == 1
        item = associations[0]
        if item["launch_node"] == 0 and item["recovery_node"] == 0:
            sortie_types["depot-depot"] += 1
        elif item["launch_node"] == 0:
            sortie_types["depot-truck"] += 1
        elif item["recovery_node"] == 0:
            sortie_types["truck-depot"] += 1
        elif item["launch_truck"] == item["recovery_truck"]:
            sortie_types["same-truck"] += 1
        else:
            sortie_types["cross-truck"] += 1
        assert validate_drone_architecture(instance, candidate, "M5") == []

    assert set(sortie_types) == {
        "depot-depot",
        "depot-truck",
        "truck-depot",
        "same-truck",
        "cross-truck",
    }


@pytest.mark.parametrize(
    ("clock_values", "expected_candidate_count"),
    [([1.0], 0), ([0.0, 1.0], 0), ([0.0, 0.0, 1.0], 1)],
)
def test_alns_candidate_insertions_salvage_candidates_at_deadline(
    monkeypatch,
    clock_values: list[float],
    expected_candidate_count: int,
) -> None:
    instance = _instance(max_battery_wh=1000.0)
    solution = _same_truck_solution(instance)
    candidate = deepcopy(solution)
    candidate.objective = 1.0
    clock = iter(clock_values)
    monkeypatch.setattr(alns_module.time, "time", lambda: next(clock, 1.0))
    monkeypatch.setattr(alns_module, "_route_positions", lambda *_args: [1])
    monkeypatch.setattr(alns_module, "_insert_truck", lambda *_args: candidate)

    candidates = _candidate_insertions(
        instance,
        solution,
        customer=2,
        beta=0.1,
        max_positions=1,
        deadline=0.5,
        max_candidates=4,
    )

    assert len(candidates) == expected_candidate_count
    if expected_candidate_count:
        assert candidates[0] is candidate
    for returned in candidates:
        assert is_feasible(instance, returned)
        assert validate_drone_architecture(instance, returned, "M5") == []


def test_alns_candidate_insertions_keep_feasible_drone_candidates_at_deadline(monkeypatch) -> None:
    instance = _instance(max_battery_wh=1000.0)
    solution = SolutionData(
        instance_name=instance.name,
        status="alns_deadline_drone_candidate_test",
        objective=0.0,
        components={},
        truck_routes={1: [0, 1, 2, 3, 5, 0], 2: [0, 4, 6, 0]},
        drone_routes={1: [0, 0]},
    )
    _sync_route_assignments(solution, instance)
    clock_values = iter([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    monkeypatch.setattr(alns_module.time, "time", lambda: next(clock_values, 1.0))
    monkeypatch.setattr(alns_module, "_route_positions", lambda *_args: [1])
    monkeypatch.setattr(alns_module, "_insert_truck", lambda *_args: None)

    candidates = _candidate_insertions(
        instance,
        solution,
        customer=2,
        beta=0.1,
        max_positions=1,
        deadline=0.5,
        max_candidates=10,
    )

    assert candidates
    for candidate in candidates:
        assert is_feasible(instance, candidate)
        assert validate_drone_architecture(instance, candidate, "M5") == []
    assert any(candidate.u_drone[(1, 2)] == 1 for candidate in candidates)


def test_alns_repair_can_select_a_feasible_truck_supported_sortie() -> None:
    instance = _truck_supported_instance()
    partial = _truck_supported_partial(instance)
    partial = _refresh(instance, partial)

    repaired = _repair(
        instance,
        partial,
        removed=[2],
        operator="greedy",
        beta=0.1,
        max_positions=2,
        max_candidates_per_customer=12,
    )

    assert repaired is not None
    assert is_feasible(instance, repaired)
    assert repaired.u_drone[(1, 2)] == 1
    association = next(row for row in sortie_associations(repaired) if row["customer_id"] == 2)
    assert association["launch_node"] == 1
    assert association["launch_truck"] == 1
    assert association["recovery_node"] == 3
    assert association["recovery_truck"] == 1
    assert repaired.z1 == {}
    assert repaired.z2 == {(1, 1, 1): 1, (3, 1, 1): 1}
    assert repaired.u_sync == {(1, 1, 1): 1, (3, 1, 1): 1}
    assert validate_drone_architecture(instance, repaired, "M5") == []
