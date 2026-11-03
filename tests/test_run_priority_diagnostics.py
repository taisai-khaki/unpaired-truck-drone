from __future__ import annotations

from types import SimpleNamespace

from src.experiments import run_priority_service_speed_by_method as runner


def test_run_one_preserves_search_diagnostics_and_refreshes_objective_components(monkeypatch) -> None:
    solution = SimpleNamespace(
        objective=10.0,
        components={
            "iterations": 17.0,
            "improving_moves_accepted": 4.0,
            "priority_tardiness_cost": 10.0,
        },
    )
    monkeypatch.setattr(runner, "_scenario_config", lambda config, scenario: config)
    monkeypatch.setattr(runner, "_run_method", lambda instance, config, method: solution)
    monkeypatch.setattr(runner, "_force_feasible_solution", lambda instance, sol, config, method: sol)
    monkeypatch.setattr(
        runner,
        "evaluate_solution",
        lambda instance, sol: (9.0, {"priority_tardiness_cost": 8.0, "truck_travel_cost": 1.0}),
    )
    monkeypatch.setattr(runner, "is_feasible", lambda instance, sol: True)

    result, _, feasible = runner._run_one(object(), {}, "nils", object())

    assert feasible is True
    assert result.objective == 9.0
    assert result.components["priority_tardiness_cost"] == 8.0
    assert result.components["truck_travel_cost"] == 1.0
    assert result.components["iterations"] == 17.0
    assert result.components["improving_moves_accepted"] == 4.0
