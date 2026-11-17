"""Solver entry-point for exact MILP optimization."""
from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, List

from ..parameters import SearchConfig
from ..data_models import InstanceData, SolutionData
from .extract_solution import extract_solution
from .model_builder import build_model
from .backend import gp


def solve_instance(
    instance: InstanceData,
    config: SearchConfig,
    progress_hook: Callable[[str], None] | None = None,
    solver_log_path: str | Path | None = None,
) -> SolutionData:
    """Build and solve one MILP instance."""
    if progress_hook is not None:
        progress_hook("build_model_start")
    artifacts = build_model(instance, config)
    if progress_hook is not None:
        progress_hook("build_model_done")
    model = artifacts.model
    if solver_log_path is not None:
        model.LogFile = str(solver_log_path)

    if config.milp.threads > 0:
        model.Params.Threads = config.milp.threads
    if config.milp.time_limit_seconds > 0:
        model.Params.TimeLimit = config.milp.time_limit_seconds
    if config.milp.mip_gap > 0:
        model.Params.MIPGap = config.milp.mip_gap

    if progress_hook is not None:
        progress_hook("optimize_start")
    t0 = time.time()
    model.optimize()
    runtime = time.time() - t0
    if progress_hook is not None:
        progress_hook("optimize_done")

    if model.SolCount == 0:
        return SolutionData(
            instance_name=instance.name,
            status="no_solution",
            objective=float("inf"),
            components={},
            run_time_seconds=runtime,
        )

    status = str(model.Status)
    if model.Status not in (gp.GRB.OPTIMAL, gp.GRB.TIME_LIMIT, gp.GRB.SUBOPTIMAL, gp.GRB.FEASIBLE, gp.GRB.INTERRUPTED):
        status = f"unsolved_{model.Status}"

    sol = extract_solution(instance, artifacts, status=status, runtime_seconds=runtime)
    sol.components["runtime_seconds"] = runtime
    sol.components["solver_status_code"] = float(model.Status)
    if hasattr(model, "MIPGap"):
        sol.components["optimality_gap"] = float(model.MIPGap)
    if hasattr(model, "BestBound"):
        sol.components["best_bound"] = float(model.BestBound)
    if hasattr(model, "ObjVal"):
        sol.components["incumbent_objective"] = float(model.ObjVal)
    if hasattr(model, "SolverLogStatus"):
        sol.__dict__["solver_log_status"] = str(model.SolverLogStatus)
    sol.__dict__["milp_native_snapshot"] = {
        name: {
            ",".join(str(part) for part in key)
            if isinstance(key, tuple)
            else str(key): _solver_value(variable)
            for key, variable in values.items()
        }
        for name, values in artifacts.variables.items()
        if name in {
            "a_truck",
            "a_drone",
            "l_truck",
            "l_drone",
            "ready_truck",
            "ready_drone",
            "tw_truck",
            "tw_drone",
            "a_bar_truck",
            "a_bar_drone",
            "tardiness",
            "lambda_loaded",
            "r",
            "r_departure",
        }
    }
    sol.objective = float(model.ObjVal) if hasattr(model, "ObjVal") else sol.objective
    return sol


def _solver_value(variable) -> float:
    if hasattr(variable, "x"):
        return float(variable.x)
    if hasattr(variable, "varValue"):
        return float(variable.varValue)
    if hasattr(variable, "value"):
        return float(variable.value())
    return float(variable)


def solve_multiple(instances: List[InstanceData], config: SearchConfig) -> List[SolutionData]:
    """Solve many instances one by one."""
    return [solve_instance(instance, config) for instance in instances]
