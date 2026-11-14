from src.data_models import SolutionData
from src.experiments.refresh_legacy_semantics import (
    final_run_status,
    solution_from_payload,
    solution_payload,
    sortie_diagnostics,
)


def test_solution_payload_round_trip_preserves_route_and_event_maps():
    original = SolutionData(
        instance_name="tiny",
        status="feasible",
        objective=12.5,
        components={"truck_cost": 10.0, "drone_cost": 2.5},
        u_sync={(2, 1, 1): 1},
        z1={(2, 1, 1): 1},
        truck_routes={1: [0, 2, 0]},
        drone_routes={1: [0, 1, 2, 0]},
    )

    restored = solution_from_payload(solution_payload(original))

    assert restored.u_sync == {(2, 1, 1): 1}
    assert restored.z1 == {(2, 1, 1): 1}
    assert restored.truck_routes == {1: [0, 2, 0]}
    assert restored.drone_routes == {1: [0, 1, 2, 0]}
    assert restored.objective == original.objective


def test_sortie_diagnostics_classify_depot_same_truck_and_cross_truck_recovery():
    solution = SolutionData(
        instance_name="tiny",
        status="feasible",
        objective=0.0,
        components={},
        u_drone={(1, 1): 1, (1, 4): 1, (1, 5): 1, (2, 10): 1},
        truck_routes={1: [0, 2, 6, 7, 0], 2: [0, 3, 0]},
        drone_routes={1: [0, 1, 2, 4, 3, 5, 0], 2: [0, 6, 10, 7, 0]},
    )

    counts = sortie_diagnostics(None, solution)

    assert counts["drone_sorties"] == 4
    assert counts["depot_truck_sorties"] == 1
    assert counts["cross_truck_recovery_sorties"] == 1
    assert counts["truck_depot_sorties"] == 1
    assert counts["same_truck_recovery_sorties"] == 1


def test_final_run_status_preserves_validation_findings_without_hiding_completion():
    assert final_run_status({"passed": False, "failures": ["objective mismatch"]}) == (
        "complete_with_integrity_findings"
    )
    assert final_run_status(
        {"passed": True, "findings": [], "expected_infeasible_real_records": [{"lambda": 0.2}]}
    ) == "complete_with_findings"
    assert final_run_status({"passed": True, "findings": []}) == "complete"
