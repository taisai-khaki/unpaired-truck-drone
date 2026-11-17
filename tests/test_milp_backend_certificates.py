from __future__ import annotations

import math

import pytest

from src.milp.backend import _PulpGRB, _read_cbc_result, gp, set_active_backend


def test_cbc_log_reader_extracts_native_incumbent_and_bound(tmp_path) -> None:
    log = tmp_path / "cbc.log"
    log.write_text(
        "Result - Optimal solution found\n"
        "Objective value: 42.50000000\n"
        "Lower bound: 42.500\n"
        "Gap: 0.00\n",
        encoding="utf-8",
    )

    result = _read_cbc_result(str(log))

    assert result == {
        "result": "Optimal solution found",
        "objective": 42.5,
        "best_bound": 42.5,
        "has_feasible_solution": True,
    }


def test_cbc_log_reader_does_not_invent_a_bound_without_a_log() -> None:
    result = _read_cbc_result(None)

    assert result["result"] == ""
    assert result["objective"] is None
    assert result["best_bound"] is None
    assert result["has_feasible_solution"] is None


def test_cbc_log_does_not_treat_relaxation_values_as_a_feasible_incumbent(tmp_path) -> None:
    log = tmp_path / "cbc_no_incumbent.log"
    log.write_text(
        "Result - Stopped on time limit\n"
        "No feasible solution found\n"
        "Lower bound: 5.195\n",
        encoding="utf-8",
    )

    result = _read_cbc_result(str(log))

    assert result["result"] == "Stopped on time limit"
    assert result["objective"] is None
    assert result["best_bound"] == 5.195
    assert result["has_feasible_solution"] is False


def test_pulp_optimal_without_native_log_is_not_reported_as_certified() -> None:
    pulp = pytest.importorskip("pulp")
    cbc = pulp.PULP_CBC_CMD(msg=False)
    if not cbc.available():
        pytest.skip("CBC is unavailable")
    set_active_backend("pulp_cbc")
    model = gp.Model("no_synthetic_certificate")
    variable = model.addVar(vtype=gp.GRB.BINARY, name="decision")
    model.addConstr(variable == 1)
    model.setObjective(variable, gp.GRB.MINIMIZE)

    model.optimize()

    assert model.SolCount == 1
    assert model.Status == _PulpGRB.FEASIBLE
    assert math.isnan(model.BestBound)
    assert math.isnan(model.MIPGap)
