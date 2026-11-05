from __future__ import annotations

from src.heuristics import build_initial_solution
from src.heuristics import run_nils
from src.heuristics import nils as nils_module
from src.instance_generator import InstanceGenerator
from src.parameters import HeuristicConfig, SearchConfig, GenerationSettings


def _config() -> SearchConfig:
    return SearchConfig(
        seed=11,
        generation=GenerationSettings(
            num_customers=12,
            num_trucks=2,
            num_drones=2,
            region="dense_urban",
            priority_share=(0.3, 0.3, 0.4),
        ),
        heuristics=HeuristicConfig(max_outer_iter=3, max_no_improve=2, time_limit_seconds=30, random_seed=11),
    )


def test_constructive_solution_valid_structure() -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(21, "constr_case")
    sol = build_initial_solution(instance, seed=99)
    assert sol.status == "heuristic_feasible"
    assert len(sol.truck_routes) == instance.num_trucks
    for route in sol.truck_routes.values():
        assert route[0] == 0 and route[-1] == 0
        assert len(route) >= 2


def test_nils_improves_or_matches_constructive() -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(22, "nils_case")
    config = _config()
    initial = build_initial_solution(instance, seed=22)
    from src.heuristics.nils import evaluate_solution

    initial_obj, _ = evaluate_solution(instance, initial)
    solved = run_nils(
        instance,
        seed=config.heuristics.random_seed,
        max_iter=config.heuristics.max_outer_iter,
        max_no_improve=config.heuristics.max_no_improve,
        time_limit=30,
    )
    assert solved.objective <= initial_obj + 1e-9 or solved.status != "nils_complete"


def test_m5_displacement_returns_feasible_candidate_found_before_deadline(monkeypatch) -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(23, "deadline_case")
    solution = build_initial_solution(instance, seed=23)
    clock = {"now": 0.0}
    candidate = nils_module._empty_solution_clone(solution)

    monkeypatch.setattr(nils_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(
        nils_module,
        "_m5_sortie_options",
        lambda *_args: [(0, 0, 1, 1), (0, 2, 1, 1)],
    )
    monkeypatch.setattr(nils_module, "move_to_drone", lambda *_args, **_kwargs: candidate)
    monkeypatch.setattr(nils_module, "_quick_local_feasible", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(nils_module, "evaluate_solution", lambda *_args: (clock.update(now=11.0) or 1.0, {}))
    monkeypatch.setattr(nils_module, "_sync_route_assignments", lambda *_args: None)
    monkeypatch.setattr(nils_module, "is_feasible", lambda *_args: True)

    stats = {
        "displacement_customers_scanned": 0,
        "displacement_sortie_options_generated": 0,
        "displacement_deadline_terminated": 0,
        "candidate_reassignments_evaluated": 0,
        "local_screen_rejections": 0,
        "strict_feasibility_checks": 0,
        "infeasible_candidate_rejections": 0,
    }
    selected = nils_module._extract_best_displacement(
        instance,
        solution,
        coords=instance.coordinate_map(),
        eligible_customers={1},
        battery_aware_screening=False,
        battery_slack_ratio=0.0,
        strict_check_cap=12,
        stats=stats,
        deadline=10.0,
    )

    assert selected is candidate
    assert stats["displacement_customers_scanned"] == 1
    assert stats["displacement_sortie_options_generated"] == 2
    assert stats["candidate_reassignments_evaluated"] == 1
    assert stats["strict_feasibility_checks"] == 1
    assert stats["displacement_deadline_terminated"] == 1


def test_m3_paired_displacement_returns_candidate_found_before_deadline(monkeypatch) -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(25, "paired_deadline_case")
    solution = build_initial_solution(instance, seed=25)
    truck = next(truck for truck, route in solution.truck_routes.items() if len(route) > 2)
    customer = solution.truck_routes[truck][1]
    drone = instance.drone_ids()[0]
    clock = {"now": 0.0}
    candidate = nils_module._empty_solution_clone(solution)

    monkeypatch.setattr(nils_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(nils_module, "move_to_drone", lambda *_args, **_kwargs: candidate)
    monkeypatch.setattr(nils_module, "_quick_local_feasible", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(nils_module, "evaluate_solution", lambda *_args: (clock.update(now=11.0) or 1.0, {}))
    monkeypatch.setattr(nils_module, "_sync_route_assignments", lambda *_args: None)
    monkeypatch.setattr(nils_module, "is_feasible", lambda *_args: True)

    stats = {
        "candidate_reassignments_evaluated": 0,
        "local_screen_rejections": 0,
        "strict_feasibility_checks": 0,
        "infeasible_candidate_rejections": 0,
        "displacement_deadline_terminated": 0,
    }
    selected = nils_module._extract_best_paired_displacement(
        instance,
        solution,
        {drone: truck},
        instance.coordinate_map(),
        eligible_customers={customer},
        battery_aware_screening=False,
        battery_slack_ratio=0.0,
        strict_check_cap=12,
        stats=stats,
        deadline=10.0,
    )

    assert selected is candidate
    assert stats["candidate_reassignments_evaluated"] == 1
    assert stats["strict_feasibility_checks"] == 1
    assert stats["displacement_deadline_terminated"] == 1


def test_m4_no_unpairing_displacement_returns_candidate_found_before_deadline(monkeypatch) -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(26, "no_unpair_deadline_case")
    solution = build_initial_solution(instance, seed=26)
    truck = next(truck for truck, route in solution.truck_routes.items() if len(route) > 2)
    customer = solution.truck_routes[truck][1]
    clock = {"now": 0.0}
    candidate = nils_module._empty_solution_clone(solution)

    monkeypatch.setattr(nils_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(nils_module, "move_to_drone", lambda *_args, **_kwargs: candidate)
    monkeypatch.setattr(nils_module, "_quick_local_feasible", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(nils_module, "evaluate_solution", lambda *_args: (clock.update(now=11.0) or 1.0, {}))
    monkeypatch.setattr(nils_module, "_sync_route_assignments", lambda *_args: None)
    monkeypatch.setattr(nils_module, "is_feasible", lambda *_args: True)

    stats = {
        "candidate_reassignments_evaluated": 0,
        "local_screen_rejections": 0,
        "strict_feasibility_checks": 0,
        "infeasible_candidate_rejections": 0,
        "displacement_deadline_terminated": 0,
    }
    selected = nils_module._extract_best_no_unpairing_displacement(
        instance,
        solution,
        instance.coordinate_map(),
        eligible_customers={customer},
        battery_aware_screening=False,
        battery_slack_ratio=0.0,
        strict_check_cap=12,
        stats=stats,
        deadline=10.0,
    )

    assert selected is candidate
    assert stats["candidate_reassignments_evaluated"] == 1
    assert stats["strict_feasibility_checks"] == 1
    assert stats["displacement_deadline_terminated"] == 1


def test_nils_accepts_salvaged_candidate_after_search_deadline(monkeypatch) -> None:
    instance = InstanceGenerator.from_search_config(_config()).generate_single(24, "deadline_accept_case")
    clock = {"now": 0.0}
    initial = build_initial_solution(instance, seed=24)
    initial_objective, _ = nils_module.evaluate_solution(instance, initial)

    monkeypatch.setattr(nils_module.time, "monotonic", lambda: clock["now"])

    def return_salvaged_candidate(_instance, solution, **_kwargs):
        candidate = nils_module._empty_solution_clone(solution)
        candidate.objective = solution.objective - 1.0
        candidate.components = {"_strict_feasibility_checked": 1.0}
        clock["now"] = 2.0
        return candidate

    monkeypatch.setattr(nils_module, "_extract_best_displacement", return_salvaged_candidate)
    result = run_nils(
        instance,
        seed=24,
        max_iter=3,
        max_no_improve=2,
        time_limit=1,
        enable_local_search=False,
        enable_perturbation=False,
        require_feasible_incumbent=False,
    )

    assert result.objective < initial_objective

