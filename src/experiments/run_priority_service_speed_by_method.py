"""Build customer-level diagnostics for the matched M1/M5/M8/M9 benchmark."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd

from ..distance_utils import euclidean_distance, travel_time_from_distance
from ..feasibility import is_feasible
from ..heuristics.nils import _repair_infeasible_solution, evaluate_solution
from ..heuristics.nils_r import compute_priority_metrics
from ..timing import active_rendezvous_events, compute_synchronization_waiting
from ..instance_generator import load_instance_json
from ..parameters import SearchConfig, load_and_build_config
from .heuristic_study import (
    _as_levels,
    _attach_relative_fields,
    _attach_reporting_fields,
    _collect_row,
    _generate_feasibility_disaggregation,
    _generate_figures,
    _generate_tables,
    _run_method,
    _scenario_config,
)
from .run_alns_comparison import _scenario_from_row, _with_alns_settings


MAIN_METHODS = [
    "truck_only",
    "nils",
]
ALNS_METHOD = "feasibility_aware_alns"
METHOD_ORDER = {
    "truck_only": ("M1 Truck-only", 1),
    "nils": ("M5 NILS", 5),
    ALNS_METHOD: ("M8 Feasibility-aware ALNS", 8),
    "nils_r": ("M9 NILS-R", 9),
}
ALL_METHODS = MAIN_METHODS + [ALNS_METHOD, "nils_r"]
OUTPUT_COLUMNS = [
    "method",
    "size",
    "feasible_runs",
    "mean_high_priority_delivery_time",
    "median_high_priority_delivery_time",
    "p90_high_priority_delivery_time",
    "mean_high_priority_tardiness",
    "median_high_priority_tardiness",
    "p90_high_priority_tardiness",
    "max_high_priority_tardiness",
    "high_priority_on_time_rate",
    "high_priority_late_rate",
    "mean_high_priority_minutes_early_when_on_time",
    "mean_high_priority_delivery_time_relative_to_deadline",
    "high_priority_served_by_drone_share",
]


def _log(path: Path, message: str) -> None:
    line = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {message}"
    print(line, flush=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def _finite_values(values: Iterable[object]) -> np.ndarray:
    out = []
    for value in values:
        if pd.notna(value):
            number = float(value)
            if math.isfinite(number):
                out.append(number)
    return np.asarray(out, dtype=float)


def _mean(values: Iterable[object]) -> float:
    arr = _finite_values(values)
    return float(np.mean(arr)) if arr.size else float("nan")


def _median(values: Iterable[object]) -> float:
    arr = _finite_values(values)
    return float(np.median(arr)) if arr.size else float("nan")


def _p90(values: Iterable[object]) -> float:
    arr = _finite_values(values)
    return float(np.percentile(arr, 90.0)) if arr.size else float("nan")


def _max(values: Iterable[object]) -> float:
    arr = _finite_values(values)
    return float(np.max(arr)) if arr.size else float("nan")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _scenario_key(row: pd.Series) -> str:
    return str(row["scenario_id"])


def _delivery_times(instance, solution) -> dict[int, float]:
    customer_ids = {customer.node_id for customer in instance.customers}
    delivered: dict[int, float] = {}
    for (truck_id, customer_id), arrival in solution.a_truck.items():
        if customer_id in customer_ids and solution.u_truck.get((truck_id, customer_id), 0) >= 0.5:
            delivered[customer_id] = float(arrival)
    for (drone_id, customer_id), arrival in solution.a_drone.items():
        if customer_id in customer_ids and solution.u_drone.get((drone_id, customer_id), 0) >= 0.5:
            delivered[customer_id] = float(arrival)
    return delivered


def _customer_rows(instance, solution, method_name: str) -> list[dict[str, object]]:
    delivered = _delivery_times(instance, solution)
    customer_map = instance.customer_map()
    drone_customers = solution.served_by_drone()
    high_customers = [customer for customer in instance.customers if customer.priority == "high"]
    missing = [customer.node_id for customer in high_customers if customer.node_id not in delivered]
    if missing:
        raise ValueError(f"Missing delivery times for high-priority customers {missing}")

    rows = []
    for customer in high_customers:
        delivery_time = float(delivered[customer.node_id])
        deadline = float(customer.ub)
        on_time = delivery_time <= deadline + 1e-9
        relative = delivery_time / deadline if deadline > 1e-12 else float("nan")
        rows.append(
            {
                "instance_id": instance.name,
                "scenario_id": str(instance.metadata.get("scenario_id", "")),
                "n": int(instance.num_customers),
                "method_name": method_name,
                "customer_id": int(customer.node_id),
                "delivery_time": delivery_time,
                "deadline": deadline,
                "tardiness": max(0.0, delivery_time - deadline),
                "on_time": bool(on_time),
                "minutes_early_when_on_time": deadline - delivery_time if on_time else float("nan"),
                "delivery_time_relative_to_deadline": relative,
                "served_by_drone": int(customer.node_id in drone_customers),
            }
        )
    return rows


def _event_nodes(solution, drone_id: int) -> tuple[object, object, object, object]:
    events = sorted(
        (node, truck)
        for node, drone, truck in active_rendezvous_events(solution)
        if drone == drone_id
    )
    launch_events = events
    recovery_events = events
    launch_node, launch_truck = launch_events[0] if launch_events else ("", "")
    recovery_node, recovery_truck = recovery_events[-1] if recovery_events else ("", "")
    return launch_node, recovery_node, launch_truck, recovery_truck


def _sortie_energy(instance, solution, drone_id: int, route: list[int]) -> float:
    if len(route) < 2:
        return float("nan")
    coordinates = instance.coordinate_map()
    drone = instance.drones[drone_id - 1]
    energy = 0.0
    for left, right in zip(route, route[1:]):
        duration = travel_time_from_distance(
            euclidean_distance(coordinates[left], coordinates[right]), drone.speed_kmph
        )
        burn = drone.energy_per_min_when_loaded if right != 0 else drone.energy_per_min_when_empty
        energy += duration * burn
    return float(energy)


def _customer_detail_rows(instance, solution, method_name: str, scenario_class_id: str, seed: int) -> list[dict[str, object]]:
    delivered = _delivery_times(instance, solution)
    customer_map = instance.customer_map()
    truck_assignments = {
        customer_id: truck_id
        for (truck_id, customer_id), value in solution.u_truck.items()
        if float(value) >= 0.5
    }
    drone_assignments = {
        customer_id: drone_id
        for (drone_id, customer_id), value in solution.u_drone.items()
        if float(value) >= 0.5
    }
    method_label = METHOD_ORDER[method_name][0]
    rows: list[dict[str, object]] = []
    for customer in instance.customers:
        customer_id = int(customer.node_id)
        delivery_time = float(delivered.get(customer_id, float("nan")))
        deadline = float(customer.ub)
        on_time = bool(np.isfinite(delivery_time) and delivery_time <= deadline + 1e-9)
        truck_id = truck_assignments.get(customer_id)
        drone_id = drone_assignments.get(customer_id)
        is_drone = drone_id is not None
        service_mode = "drone" if is_drone else "truck"
        serving_id = drone_id if is_drone else truck_id
        launch_node = ""
        recovery_node = ""
        launch_truck = ""
        recovery_truck = ""
        battery_before = float("nan")
        battery_after = float("nan")
        battery_slack = float("nan")
        sortie_energy = float("nan")
        sync_wait = 0.0
        recovery_type = ""
        if is_drone:
            route = list(solution.drone_routes.get(int(drone_id), []))
            launch_node = route[0] if route else ""
            recovery_node = route[-1] if route else ""
            event_launch, event_recovery, launch_truck, recovery_truck = _event_nodes(solution, int(drone_id))
            if event_launch != "":
                launch_node = event_launch
            if event_recovery != "":
                recovery_node = event_recovery
            drone = instance.drones[int(drone_id) - 1]
            sortie_energy = _sortie_energy(instance, solution, int(drone_id), route)
            battery_before = float(solution.battery_drone.get((int(drone_id), launch_node), drone.max_battery_wh))
            battery_after = float(solution.battery_drone.get((int(drone_id), recovery_node), drone.max_battery_wh - sortie_energy))
            reserve_wh = float(instance.metadata.get("battery_slack_ratio", 0.0)) * float(drone.max_battery_wh)
            battery_slack = battery_after - reserve_wh
            if recovery_node == 0:
                recovery_type = "depot"
            elif launch_truck != "" and recovery_truck != "" and launch_truck != recovery_truck:
                recovery_type = "cross_truck"
            else:
                recovery_type = "same_truck"
            sync_nodes = {customer_id, launch_node, recovery_node}
            sync_wait = float(
                sum(
                    float(event["total_wait"])
                    for event in compute_synchronization_waiting(instance, solution)["events"]
                    if int(event["drone_id"]) == int(drone_id)
                    and int(event["node"]) in {int(node) for node in sync_nodes if str(node) not in {"", "nan"}}
                )
            )
        tardiness = max(0.0, delivery_time - deadline) if np.isfinite(delivery_time) else float("nan")
        rows.append(
            {
                "instance_id": instance.name,
                "scenario_class_id": scenario_class_id,
                "seed": int(seed),
                "method": method_label,
                "size": int(instance.num_customers),
                "customer_id": customer_id,
                "priority_class": customer.priority,
                "priority_weight": float(instance.priority_penalties[customer.priority]),
                "service_mode": service_mode,
                "serving_vehicle_id": serving_id if serving_id is not None else "",
                "delivery_time_L_i": delivery_time,
                "window_start_e_i": float(customer.lb),
                "deadline_l_i": deadline,
                "tardiness": tardiness,
                "weighted_tardiness": tardiness * float(instance.priority_penalties[customer.priority])
                if np.isfinite(tardiness) else float("nan"),
                "on_time_indicator": int(on_time),
                "minutes_early_if_on_time": deadline - delivery_time if on_time else float("nan"),
                "is_high_priority": int(customer.priority == "high"),
                "is_drone_served": int(is_drone),
                "truck_route_id": truck_id if truck_id is not None else "",
                "drone_id": drone_id if drone_id is not None else "",
                "launch_node": launch_node,
                "recovery_node": recovery_node,
                "recovery_type": recovery_type,
                "sync_truck_id": recovery_truck if recovery_truck != "" else launch_truck,
                "battery_before_sortie": battery_before,
                "battery_after_sortie": battery_after,
                "battery_slack_after_sortie": battery_slack,
                "sortie_energy": sortie_energy,
                "sync_wait_time": sync_wait,
            }
        )
    return rows


def _force_feasible_solution(instance, solution, config: SearchConfig, method_name: str):
    if is_feasible(instance, solution):
        return solution
    experiment = dict(config.experiment or {})
    max_steps = max(1, int(experiment.get("study_output_repair_max_steps", 8)))
    repaired = _repair_infeasible_solution(instance, solution, max_steps=max(max_steps, instance.num_customers))
    if repaired is not None and is_feasible(instance, repaired):
        return repaired
    fallback = _run_method(instance, config, "truck_only")
    if is_feasible(instance, fallback):
        fallback.status = f"{method_name}|fallback_truck_only"
        return fallback
    return solution


def _run_one(
    instance,
    scenario: dict[str, object],
    method_name: str,
    config: SearchConfig,
    common_time_limit: float | None = None,
):
    scenario_config = _scenario_config(config, scenario)
    if common_time_limit is not None:
        limit = float(common_time_limit)
        if limit <= 0:
            raise ValueError("common_time_limit must be positive")
        experiment = dict(scenario_config.experiment or {})
        if method_name == ALNS_METHOD:
            experiment["alns_time_limit_seconds"] = limit
        scenario_config = replace(
            scenario_config,
            heuristics=replace(scenario_config.heuristics, time_limit_seconds=limit),
            experiment=experiment,
        )
    start = time.time()
    solution = _run_method(instance, scenario_config, method_name)
    solution = _force_feasible_solution(instance, solution, scenario_config, method_name)
    search_components = dict(solution.components)
    solution.objective, objective_components = evaluate_solution(instance, solution)
    search_diagnostics = {
        key: value for key, value in search_components.items() if key not in objective_components
    }
    solution.components = {**search_diagnostics, **objective_components}
    elapsed = time.time() - start
    return solution, elapsed, bool(is_feasible(instance, solution))


def _load_partial(path: Path, columns: list[str]) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame(columns=columns)
    frame = pd.read_csv(path)
    for column in columns:
        if column not in frame.columns:
            frame[column] = np.nan
    return frame[columns].copy()


def _write_partial(frame: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(path, index=False, float_format="%.10f")


def _aggregate(run_status: pd.DataFrame, customer_rows: pd.DataFrame) -> pd.DataFrame:
    summaries = []
    for method_name, (label, method_order) in METHOD_ORDER.items():
        method_status = run_status[run_status["method_name"].eq(method_name)].copy()
        for size in ["All", "15", "50", "75", "100"]:
            if size == "All":
                status_subset = method_status
                customer_subset = customer_rows[customer_rows["method_name"].eq(method_name)]
            else:
                status_subset = method_status[method_status["n"].eq(int(size))]
                customer_subset = customer_rows[
                    customer_rows["method_name"].eq(method_name) & customer_rows["n"].eq(int(size))
                ]
            feasible_status = status_subset[status_subset["feasible"].astype(bool)]
            feasible_runs = int(len(feasible_status))
            on_time = _finite_values(customer_subset["on_time"] if not customer_subset.empty else [])
            late = 1.0 - float(np.mean(on_time)) if on_time.size else float("nan")
            on_time_rate = float(np.mean(on_time)) if on_time.size else float("nan")
            summaries.append(
                {
                    "_method_order": method_order,
                    "_size_order": -1 if size == "All" else int(size),
                    "method": label,
                    "size": size,
                    "feasible_runs": feasible_runs,
                    "mean_high_priority_delivery_time": _mean(customer_subset["delivery_time"]),
                    "median_high_priority_delivery_time": _median(customer_subset["delivery_time"]),
                    "p90_high_priority_delivery_time": _p90(customer_subset["delivery_time"]),
                    "mean_high_priority_tardiness": _mean(customer_subset["tardiness"]),
                    "median_high_priority_tardiness": _median(customer_subset["tardiness"]),
                    "p90_high_priority_tardiness": _p90(customer_subset["tardiness"]),
                    "max_high_priority_tardiness": _max(customer_subset["tardiness"]),
                    "high_priority_on_time_rate": on_time_rate,
                    "high_priority_late_rate": late,
                    "mean_high_priority_minutes_early_when_on_time": _mean(
                        customer_subset["minutes_early_when_on_time"]
                    ),
                    "mean_high_priority_delivery_time_relative_to_deadline": _mean(
                        customer_subset["delivery_time_relative_to_deadline"]
                    ),
                    "high_priority_served_by_drone_share": _mean(customer_subset["served_by_drone"]),
                }
            )
    table = pd.DataFrame(summaries).sort_values(["_method_order", "_size_order"]).reset_index(drop=True)
    return table[OUTPUT_COLUMNS]


def _build_decision_quality_tables(
    paper_rows: pd.DataFrame,
    customer_detail_rows: pd.DataFrame,
    output_dir: Path,
) -> None:
    method_labels = {name: value[0] for name, value in METHOD_ORDER.items()}
    per_run = paper_rows.copy()
    per_run["method"] = per_run["method_name"].map(method_labels)
    per_run["size"] = per_run["n"].astype(int).astype(str)
    detail = customer_detail_rows.copy()
    reverse_labels = {label: name for name, label in method_labels.items()}
    if not detail.empty:
        detail["method_name"] = detail["method"].map(reverse_labels)
        detail["is_drone_served"] = pd.to_numeric(detail["is_drone_served"], errors="coerce").fillna(0)
        detail["is_high_priority"] = pd.to_numeric(detail["is_high_priority"], errors="coerce").fillna(0)
        detail["drone_id"] = detail["drone_id"].replace("", np.nan)
        active = (
            detail[detail["is_drone_served"].eq(1)]
            .groupby(["instance_id", "method_name"])["drone_id"]
            .nunique()
            .rename("active_drones_used")
        )
        priority_shares = (
            detail.groupby(["instance_id", "method_name", "priority_class"])["is_drone_served"]
            .mean()
            .unstack("priority_class")
            .rename(
                columns={
                    "high": "high_priority_drone_share",
                    "medium": "medium_priority_drone_share",
                    "low": "low_priority_drone_share",
                }
            )
        )
        for share_column in (
            "high_priority_drone_share",
            "medium_priority_drone_share",
            "low_priority_drone_share",
        ):
            if share_column not in priority_shares.columns:
                priority_shares[share_column] = np.nan
        hp_tail = (
            detail[detail["is_high_priority"].eq(1)]
            .groupby(["instance_id", "method_name"])["tardiness"]
            .quantile(0.90)
            .rename("p90_high_priority_tardiness")
        )
        per_run = per_run.merge(active.reset_index(), on=["instance_id", "method_name"], how="left")
        per_run = per_run.merge(priority_shares.reset_index(), on=["instance_id", "method_name"], how="left")
        per_run = per_run.merge(hp_tail.reset_index(), on=["instance_id", "method_name"], how="left")
    else:
        per_run["active_drones_used"] = np.nan
        per_run["high_priority_drone_share"] = np.nan
        per_run["medium_priority_drone_share"] = np.nan
        per_run["low_priority_drone_share"] = np.nan
        per_run["p90_high_priority_tardiness"] = np.nan
    per_run["active_drones_used"] = per_run["active_drones_used"].fillna(0)
    per_run["active_drone_ratio"] = per_run["active_drones_used"] / per_run["number_of_drones"].replace(0, np.nan)
    truck_reference = per_run[per_run["method_name"].eq("truck_only")][
        ["instance_id", "priority_tardiness_cost", "high_priority_tardiness_minutes", "final_solution_feasibility_flag"]
    ].rename(
        columns={
            "priority_tardiness_cost": "truck_reference_ptc",
            "high_priority_tardiness_minutes": "truck_reference_hpt",
            "final_solution_feasibility_flag": "truck_reference_feasible",
        }
    )
    per_run = per_run.merge(truck_reference, on="instance_id", how="left")
    valid_reference = per_run["truck_reference_feasible"].astype(str).str.lower().isin({"true", "1"})
    drone_customer_count = pd.to_numeric(per_run["customers_served_by_drone"], errors="coerce")
    drone_distance = pd.to_numeric(per_run["drone_flight_distance_km"], errors="coerce")
    per_run["service_value_per_drone_customer"] = np.where(
        valid_reference & (drone_customer_count > 0),
        (per_run["truck_reference_ptc"] - per_run["priority_tardiness_cost"]) / drone_customer_count,
        np.nan,
    )
    per_run["service_value_per_active_drone"] = np.where(
        valid_reference & (per_run["active_drones_used"] > 0),
        (per_run["truck_reference_ptc"] - per_run["priority_tardiness_cost"]) / per_run["active_drones_used"],
        np.nan,
    )
    per_run["service_value_per_drone_km"] = np.where(
        valid_reference & (drone_distance > 0),
        (per_run["truck_reference_ptc"] - per_run["priority_tardiness_cost"]) / drone_distance,
        np.nan,
    )
    per_run["high_priority_protection_ratio"] = np.where(
        valid_reference,
        (per_run["truck_reference_hpt"] - per_run["high_priority_tardiness_minutes"])
        / np.maximum(1e-6, per_run["truck_reference_hpt"]),
        np.nan,
    )
    if "total_sync_wait_readiness_minutes" not in per_run.columns:
        per_run["total_sync_wait_readiness_minutes"] = np.nan
    per_run["total_sync_wait_readiness_minutes"] = pd.to_numeric(
        per_run["total_sync_wait_readiness_minutes"], errors="coerce"
    )
    groups = [("All", per_run)] + [
        (str(size), per_run[per_run["n"].eq(size)]) for size in sorted(per_run["n"].dropna().unique())
    ]
    summary_rows = []
    for size, size_frame in groups:
        for method_name, label in method_labels.items():
            method_frame = size_frame[
                size_frame["method_name"].eq(method_name)
                & size_frame["final_solution_feasibility_flag"].astype(str).str.lower().isin({"true", "1"})
            ]
            if method_frame.empty:
                continue
            summary_rows.append(
                {
                    "method": label,
                    "size": size,
                    "feasible_runs": int(len(method_frame)),
                    "mean_objective": float(method_frame["objective_total"].mean()),
                    "mean_priority_tardiness_cost": float(method_frame["priority_tardiness_cost"].mean()),
                    "mean_high_priority_tardiness": float(method_frame["high_priority_tardiness_minutes"].mean()),
                    "mean_medium_priority_tardiness": float(method_frame["medium_priority_tardiness_minutes"].mean()),
                    "mean_low_priority_tardiness": float(method_frame["low_priority_tardiness_minutes"].mean()),
                    "mean_drone_share": float(method_frame["drone_service_share"].mean()),
                    "mean_drone_distance": float(method_frame["drone_flight_distance_km"].mean()),
                    "mean_truck_distance": float(method_frame["truck_route_distance_km"].mean()),
                    "mean_total_sync_wait": float(method_frame["total_sync_wait_readiness_minutes"].mean()),
                    "mean_cpu": float(method_frame["cpu_time_seconds"].mean()),
                    "service_value_per_drone_customer": float(method_frame["service_value_per_drone_customer"].mean()),
                    "service_value_per_drone_km": float(method_frame["service_value_per_drone_km"].mean()),
                    "high_priority_protection_ratio": float(method_frame["high_priority_protection_ratio"].mean()),
                }
            )
    decision_quality = pd.DataFrame(summary_rows)
    tables_dir = output_dir / "tables"
    tables_dir.mkdir(parents=True, exist_ok=True)
    decision_quality.to_csv(tables_dir / "table_decision_quality_existing_methods.csv", index=False, float_format="%.10f")
    use_columns = [
        "method", "size", "feasible_runs", "mean_available_drones", "mean_active_drones_used",
        "mean_active_drones_used_ratio", "mean_drone_served_customers", "mean_drone_share",
        "mean_high_priority_served_by_drone_share", "mean_medium_priority_served_by_drone_share",
        "mean_low_priority_served_by_drone_share", "mean_priority_tardiness_cost",
        "mean_high_priority_tardiness", "mean_p90_high_priority_tardiness", "mean_drone_distance",
        "mean_sync_wait", "mean_battery_slack_after_sortie", "service_value_per_drone_customer",
        "service_value_per_active_drone", "service_value_per_drone_km",
    ]
    use_rows = []
    for size, size_frame in groups:
        for method_name, label in method_labels.items():
            method_frame = size_frame[
                size_frame["method_name"].eq(method_name)
                & size_frame["final_solution_feasibility_flag"].astype(str).str.lower().isin({"true", "1"})
            ]
            if method_frame.empty:
                continue
            available = pd.to_numeric(method_frame["number_of_drones"], errors="coerce")
            hp_share = pd.to_numeric(method_frame.get("high_priority_drone_share", np.nan), errors="coerce")
            medium_share = pd.to_numeric(method_frame.get("medium_priority_drone_share", np.nan), errors="coerce")
            low_share = pd.to_numeric(method_frame.get("low_priority_drone_share", np.nan), errors="coerce")
            battery = pd.to_numeric(
                detail[
                    detail["method_name"].eq(method_name)
                    & detail["instance_id"].isin(method_frame["instance_id"])
                    & detail["is_drone_served"].eq(1)
                ]["battery_slack_after_sortie"],
                errors="coerce",
            ) if not detail.empty else pd.Series(dtype=float)
            use_rows.append(
                {
                    "method": label,
                    "size": size,
                    "feasible_runs": int(len(method_frame)),
                    "mean_available_drones": float(available.mean()),
                    "mean_active_drones_used": float(method_frame["active_drones_used"].mean()),
                    "mean_active_drones_used_ratio": float(method_frame["active_drone_ratio"].mean()),
                    "mean_drone_served_customers": float(method_frame["customers_served_by_drone"].mean()),
                    "mean_drone_share": float(method_frame["drone_service_share"].mean()),
                    "mean_high_priority_served_by_drone_share": float(hp_share.mean()),
                    "mean_medium_priority_served_by_drone_share": float(medium_share.mean()),
                    "mean_low_priority_served_by_drone_share": float(low_share.mean()),
                    "mean_priority_tardiness_cost": float(method_frame["priority_tardiness_cost"].mean()),
                    "mean_high_priority_tardiness": float(method_frame["high_priority_tardiness_minutes"].mean()),
                    "mean_p90_high_priority_tardiness": float(method_frame["p90_high_priority_tardiness"].mean()),
                    "mean_drone_distance": float(method_frame["drone_flight_distance_km"].mean()),
                    "mean_sync_wait": float(method_frame["total_sync_wait_readiness_minutes"].mean()),
                    "mean_battery_slack_after_sortie": float(battery.mean()) if not battery.empty else float("nan"),
                    "service_value_per_drone_customer": float(method_frame["service_value_per_drone_customer"].mean()),
                    "service_value_per_active_drone": float(method_frame["service_value_per_active_drone"].mean()),
                    "service_value_per_drone_km": float(method_frame["service_value_per_drone_km"].mean()),
                }
            )
    pd.DataFrame(use_rows, columns=use_columns).to_csv(
        tables_dir / "table_drone_use_decision_quality.csv", index=False, float_format="%.10f"
    )


def run_pipeline(
    *,
    main_config_path: Path,
    alns_config_path: Path,
    source_output_dir: Path,
    output_dir: Path,
    limit: int | None = None,
    start_index: int = 0,
    end_index: int | None = None,
    methods: list[str] | None = None,
    common_time_limit: float | None = None,
) -> pd.DataFrame:
    main_config = load_and_build_config(str(main_config_path))
    alns_config = _with_alns_settings(load_and_build_config(str(alns_config_path)))
    source_path = source_output_dir / "tables" / "heuristic_study_runs.csv"
    source = pd.read_csv(source_path)
    meta = (
        source.sort_values(["scenario_id", "instance_id"])
        .drop_duplicates(["scenario_id", "instance_id"])
        .reset_index(drop=True)
    )
    if limit is not None and limit > 0:
        meta = meta.head(int(limit)).copy()
    meta = meta.iloc[int(start_index):int(end_index) if end_index is not None else None].reset_index(drop=True)

    raw_dir = source_output_dir / "raw"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / "priority_service_speed_progress.log"
    run_path = output_dir / "run_status_partial.csv"
    customer_path = output_dir / "customer_delivery_observations_partial.csv"
    customer_detail_path = output_dir / "customer_service_detail.csv"
    reconstruction_path = output_dir / "nils_r_reconstruction_log.csv"
    decision_path = output_dir / "drone_decision_log.csv"
    paper_rows_path = output_dir / "paper_method_rows_partial.csv"
    validation_path = output_dir / "replay_objective_audit.csv"
    derived_columns = {
        "truck_only_objective",
        "truck_only_distance",
        "simple_drone_objective",
        "improvement_vs_truck_only_pct",
        "improvement_vs_simple_drone_pct",
        "truck_distance_reduction_pct",
        "is_feasible_run",
        "reporting_policy",
        "reporting_penalty_value",
        "objective_for_reporting",
        "reporting_included",
        "truck_only_objective_reported",
        "simple_drone_objective_reported",
        "improvement_vs_truck_only_reported_pct",
        "improvement_vs_simple_drone_reported_pct",
    }
    paper_columns = [column for column in source.columns if column not in derived_columns]
    if "total_sync_wait_readiness_minutes" not in paper_columns:
        paper_columns.append("total_sync_wait_readiness_minutes")
    paper_rows = _load_partial(paper_rows_path, paper_columns)
    validation_columns = [
        "instance_id", "method_name", "source_objective", "replay_objective", "absolute_objective_gap",
        "source_feasible", "replay_feasible", "feasibility_match", "objective_match", "cpu_time_seconds", "error",
    ]
    validation_rows = _load_partial(validation_path, validation_columns)
    source_lookup = {
        (str(row["instance_id"]), str(row["method_name"])): row
        for row in source.to_dict(orient="records")
    }
    instance_cache: dict[str, object] = {}
    solution_cache: dict[tuple[str, str], object] = {}
    run_columns = [
        "instance_id", "scenario_id", "n", "method_name", "feasible", "cpu_time_seconds", "error",
        "objective", "priority_tardiness_cost", "high_priority_tardiness",
        "max_high_priority_tardiness", "p90_high_priority_tardiness",
        "drone_served_customers", "drone_share", "truck_travel_cost", "drone_travel_cost",
        "waiting_sync_cost", "truck_route_distance_km", "drone_flight_distance_km",
        "total_truck_waiting_time", "total_drone_waiting_time", "average_synchronization_delay",
        "maximum_synchronization_delay", "average_battery_usage_ratio", "maximum_battery_usage_ratio",
        "sorties_near_endurance_limit", "eligible_not_assigned_due_battery", "makespan_minutes",
        "number_of_drone_launches", "number_of_drone_retrievals",
    ]
    customer_detail_columns = [
        "instance_id", "scenario_class_id", "seed", "method", "size", "customer_id",
        "priority_class", "priority_weight", "service_mode", "serving_vehicle_id",
        "delivery_time_L_i", "window_start_e_i", "deadline_l_i", "tardiness", "weighted_tardiness",
        "on_time_indicator", "minutes_early_if_on_time", "is_high_priority", "is_drone_served",
        "truck_route_id", "drone_id", "launch_node", "recovery_node", "recovery_type",
        "sync_truck_id", "battery_before_sortie", "battery_after_sortie",
        "battery_slack_after_sortie", "sortie_energy", "sync_wait_time",
    ]
    decision_columns = [
        "instance_id", "seed", "method", "iteration", "move_type", "customer_id",
        "priority_class", "action", "from_mode", "to_mode", "launch_node", "recovery_node",
        "recovery_type", "sync_truck_id", "delta_Z", "delta_priority_tardiness_cost",
        "delta_high_priority_tardiness", "delta_max_high_priority_tardiness", "delta_truck_cost",
        "delta_drone_cost", "delta_sync_wait", "battery_slack_after_move", "accepted", "rejection_reason",
    ]
    customer_columns = [
        "instance_id",
        "scenario_id",
        "n",
        "method_name",
        "customer_id",
        "delivery_time",
        "deadline",
        "tardiness",
        "on_time",
        "minutes_early_when_on_time",
        "delivery_time_relative_to_deadline",
        "served_by_drone",
    ]
    reconstruction_columns = [
        "instance_id", "seed", "size", "reconstruction_attempt", "trigger_reason",
        "num_drone_customers_before", "num_candidates_evaluated", "num_customers_returned_to_truck",
        "customer_ids_returned", "Z_before", "Z_after", "HPT_before", "HPT_after",
        "MaxHPT_before", "MaxHPT_after", "P90HPT_before", "P90HPT_after",
        "drone_share_before", "drone_share_after", "active_drones_before", "active_drones_after",
        "accepted", "rejection_reason", "cpu_reconstruction",
    ]
    run_status = _load_partial(run_path, run_columns)
    customer_rows = _load_partial(customer_path, customer_columns)
    customer_detail_rows = _load_partial(customer_detail_path, customer_detail_columns)
    decision_rows = _load_partial(decision_path, decision_columns)
    reconstruction_rows = _load_partial(reconstruction_path, reconstruction_columns)
    selected_methods = list(methods) if methods else list(ALL_METHODS)
    unknown_methods = sorted(set(selected_methods) - set(ALL_METHODS))
    if unknown_methods:
        raise ValueError(f"Unknown methods: {unknown_methods}")
    provenance_path = output_dir / "run_provenance.json"
    try:
        source_commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path.cwd(),
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        source_commit = "unavailable"
    if provenance_path.exists():
        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        if provenance.get("source_git_commit") != source_commit:
            raise RuntimeError("Cannot resume diagnostics with a different source commit")
    else:
        config_copy_dir = output_dir / "config"
        config_copy_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(main_config_path, config_copy_dir / "main_config.yaml")
        shutil.copy2(alns_config_path, config_copy_dir / "alns_config.yaml")
        source_code = Path(__file__).resolve()
        provenance = {
            "run_status": "running",
            "run_started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "source_git_commit": source_commit,
            "source_code_file": str(source_code),
            "source_code_sha256": _sha256(source_code),
            "main_config_sha256": _sha256(main_config_path),
            "alns_config_sha256": _sha256(alns_config_path),
            "source_results_file": str(source_path.resolve()),
            "source_results_sha256": _sha256(source_path),
            "input_instances": int(len(meta)),
            "selected_methods": selected_methods,
            "configured_nils_time_limit_seconds": float(main_config.heuristics.time_limit_seconds),
            "configured_nils_max_outer_iterations": int(main_config.heuristics.max_outer_iter),
            "configured_alns_time_limit_seconds": float(
                dict(main_config.experiment or {}).get("alns_time_limit_seconds", float("nan"))
            ),
            "configured_alns_max_iterations": int(
                dict(main_config.experiment or {}).get("alns_max_iter", 0)
            ),
            "resume_policy": "append per completed instance; skip successful instance-method pairs",
        }
        provenance_path.write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    successful_status = run_status[run_status["error"].fillna("").astype(str).eq("")]
    completed = set(zip(successful_status["instance_id"].astype(str), successful_status["method_name"].astype(str)))
    _log(log_path, f"Priority service-speed study start | instances={len(meta)} | methods={selected_methods} | completed_pairs={len(completed)}")

    for index, source_row in meta.iterrows():
        instance_id = str(source_row["instance_id"])
        instance = load_instance_json(raw_dir / f"{instance_id}.json")
        instance_cache[instance_id] = instance
        copied_raw_dir = output_dir / "raw"
        copied_raw_dir.mkdir(parents=True, exist_ok=True)
        for suffix in (".json", ".csv"):
            source_raw = raw_dir / f"{instance_id}{suffix}"
            if source_raw.exists():
                shutil.copy2(source_raw, copied_raw_dir / source_raw.name)
        scenario = _scenario_from_row(source_row)
        _log(log_path, f"[{index + 1}/{len(meta)}] {instance_id} | n={instance.num_customers} start")
        instance_run_rows = []
        instance_customer_rows = []
        instance_customer_detail_rows = []
        instance_decision_rows = []
        instance_paper_rows = []
        instance_validation_rows = []
        for method_name in selected_methods:
            key = (instance_id, method_name)
            if key in completed:
                continue
            config = alns_config if method_name == ALNS_METHOD else main_config
            try:
                solution, elapsed, feasible = _run_one(
                    instance,
                    scenario,
                    method_name,
                    config,
                    common_time_limit=common_time_limit,
                )
                priority_metrics = compute_priority_metrics(instance, solution) if feasible else {}
                drone_customers = solution.served_by_drone() if feasible else set()
                rich_metrics = _collect_row(instance, solution, scenario, method_name)
                rich_metrics["cpu_time_seconds"] = float(elapsed)
                rich_metrics["total_sync_wait_readiness_minutes"] = float(
                    compute_synchronization_waiting(instance, solution)["total_waiting_minutes"]
                )
                instance_paper_rows.append(rich_metrics)
                solution_cache[(instance_id, method_name)] = solution
                expected = source_lookup.get(key, {})
                source_objective = float(expected.get("objective_total", float("nan")))
                replay_objective = float(solution.objective)
                objective_gap = abs(source_objective - replay_objective)
                source_feasible = str(expected.get("final_solution_feasibility_flag", "")).strip().lower() in {"true", "1"}
                instance_validation_rows.append(
                    {
                        "instance_id": instance_id,
                        "method_name": method_name,
                        "source_objective": source_objective,
                        "replay_objective": replay_objective,
                        "absolute_objective_gap": objective_gap,
                        "source_feasible": source_feasible,
                        "replay_feasible": bool(feasible),
                        "feasibility_match": source_feasible == bool(feasible),
                        "objective_match": bool(np.isfinite(objective_gap) and objective_gap <= 1e-6),
                        "cpu_time_seconds": float(elapsed),
                        "error": "",
                    }
                )
                instance_run_rows.append(
                    {
                        "instance_id": instance_id,
                        "scenario_id": _scenario_key(source_row),
                        "n": int(instance.num_customers),
                        "method_name": method_name,
                        "feasible": bool(feasible),
                        "cpu_time_seconds": float(elapsed),
                        "error": "",
                        "objective": float(solution.objective) if feasible else float("nan"),
                        "priority_tardiness_cost": float(priority_metrics.get("priority_tardiness_cost", float("nan"))),
                        "high_priority_tardiness": float(priority_metrics.get("high_priority_tardiness", float("nan"))),
                        "max_high_priority_tardiness": float(priority_metrics.get("max_high_priority_tardiness", float("nan"))),
                        "p90_high_priority_tardiness": float(priority_metrics.get("p90_high_priority_tardiness", float("nan"))),
                        "drone_served_customers": int(len(drone_customers)) if feasible else 0,
                        "drone_share": float(len(drone_customers) / max(1, instance.num_customers)) if feasible else float("nan"),
                        "truck_travel_cost": float(rich_metrics.get("truck_travel_cost", float("nan"))),
                        "drone_travel_cost": float(rich_metrics.get("drone_travel_cost", float("nan"))),
                        "waiting_sync_cost": float(rich_metrics.get("waiting_sync_cost", float("nan"))),
                        "truck_route_distance_km": float(rich_metrics.get("truck_route_distance_km", float("nan"))),
                        "drone_flight_distance_km": float(rich_metrics.get("drone_flight_distance_km", float("nan"))),
                        "total_truck_waiting_time": float(rich_metrics.get("total_truck_waiting_time", float("nan"))),
                        "total_drone_waiting_time": float(rich_metrics.get("total_drone_waiting_time", float("nan"))),
                        "average_synchronization_delay": float(rich_metrics.get("average_synchronization_delay", float("nan"))),
                        "maximum_synchronization_delay": float(rich_metrics.get("maximum_synchronization_delay", float("nan"))),
                        "average_battery_usage_ratio": float(rich_metrics.get("average_battery_usage_ratio", float("nan"))),
                        "maximum_battery_usage_ratio": float(rich_metrics.get("maximum_battery_usage_ratio", float("nan"))),
                        "sorties_near_endurance_limit": float(rich_metrics.get("sorties_near_endurance_limit", float("nan"))),
                        "eligible_not_assigned_due_battery": float(rich_metrics.get("eligible_not_assigned_due_battery", float("nan"))),
                        "makespan_minutes": float(rich_metrics.get("makespan_minutes", float("nan"))),
                        "number_of_drone_launches": float(rich_metrics.get("number_of_drone_launches", float("nan"))),
                        "number_of_drone_retrievals": float(rich_metrics.get("number_of_drone_retrievals", float("nan"))),
                    }
                )
                if feasible:
                    instance_customer_rows.extend(_customer_rows(instance, solution, method_name))
                    instance_customer_detail_rows.extend(
                        _customer_detail_rows(instance, solution, method_name, _scenario_key(source_row), instance.seed)
                    )
                if method_name == "nils_r":
                    for decision_log in solution.__dict__.get("nils_r_drone_decision_logs", []):
                        decision_row = dict(decision_log)
                        decision_row["instance_id"] = instance_id
                        decision_row["seed"] = int(instance.seed)
                        decision_row["method"] = METHOD_ORDER[method_name][0]
                        customer = instance.customer_map().get(int(decision_row["customer_id"]))
                        decision_row["priority_class"] = customer.priority if customer is not None else ""
                        instance_decision_rows.append(decision_row)
                if method_name == "nils_r":
                    for reconstruction_log in solution.__dict__.get("nils_r_reconstruction_logs", []):
                        reconstruction_row = dict(reconstruction_log)
                        reconstruction_row["instance_id"] = instance_id
                        reconstruction_row["seed"] = int(instance.seed)
                        reconstruction_rows = pd.concat(
                            [reconstruction_rows, pd.DataFrame([reconstruction_row])], ignore_index=True
                        )
                _log(log_path, f"  {instance_id} | {method_name} | feasible={feasible} time={elapsed:.2f}s")
            except Exception as error:
                instance_run_rows.append(
                    {
                        "instance_id": instance_id,
                        "scenario_id": _scenario_key(source_row),
                        "n": int(instance.num_customers),
                        "method_name": method_name,
                        "feasible": False,
                        "cpu_time_seconds": float("nan"),
                        "error": repr(error),
                        "objective": float("nan"),
                        "priority_tardiness_cost": float("nan"),
                        "high_priority_tardiness": float("nan"),
                        "max_high_priority_tardiness": float("nan"),
                        "p90_high_priority_tardiness": float("nan"),
                        "drone_served_customers": 0,
                        "drone_share": float("nan"),
                        "truck_travel_cost": float("nan"),
                        "drone_travel_cost": float("nan"),
                        "waiting_sync_cost": float("nan"),
                        "truck_route_distance_km": float("nan"),
                        "drone_flight_distance_km": float("nan"),
                        "total_truck_waiting_time": float("nan"),
                        "total_drone_waiting_time": float("nan"),
                        "average_synchronization_delay": float("nan"),
                        "maximum_synchronization_delay": float("nan"),
                        "average_battery_usage_ratio": float("nan"),
                        "maximum_battery_usage_ratio": float("nan"),
                        "sorties_near_endurance_limit": float("nan"),
                        "eligible_not_assigned_due_battery": float("nan"),
                        "makespan_minutes": float("nan"),
                        "number_of_drone_launches": float("nan"),
                        "number_of_drone_retrievals": float("nan"),
                    }
                )
                instance_validation_rows.append(
                    {
                        "instance_id": instance_id,
                        "method_name": method_name,
                        "source_objective": float("nan"),
                        "replay_objective": float("nan"),
                        "absolute_objective_gap": float("nan"),
                        "source_feasible": False,
                        "replay_feasible": False,
                        "feasibility_match": False,
                        "objective_match": False,
                        "cpu_time_seconds": float("nan"),
                        "error": repr(error),
                    }
                )
                _log(log_path, f"  {instance_id} | {method_name} | failed error={error}")

        if instance_run_rows:
            run_status = pd.concat([run_status, pd.DataFrame(instance_run_rows)], ignore_index=True)
            customer_rows = pd.concat([customer_rows, pd.DataFrame(instance_customer_rows)], ignore_index=True)
            customer_detail_rows = pd.concat(
                [customer_detail_rows, pd.DataFrame(instance_customer_detail_rows)], ignore_index=True
            )
            decision_rows = pd.concat([decision_rows, pd.DataFrame(instance_decision_rows)], ignore_index=True)
            run_status = run_status.drop_duplicates(["instance_id", "method_name"], keep="last")
            customer_rows = customer_rows.drop_duplicates(
                ["instance_id", "method_name", "customer_id"], keep="last"
            )
            customer_detail_rows = customer_detail_rows.drop_duplicates(
                ["instance_id", "method", "customer_id"], keep="last"
            )
            reconstruction_rows = reconstruction_rows.drop_duplicates(
                ["instance_id", "reconstruction_attempt"], keep="last"
            )
            if instance_paper_rows:
                paper_rows = pd.concat([paper_rows, pd.DataFrame(instance_paper_rows)], ignore_index=True)
                paper_rows = paper_rows.drop_duplicates(["instance_id", "method_name"], keep="last")
            if instance_validation_rows:
                validation_rows = pd.concat([validation_rows, pd.DataFrame(instance_validation_rows)], ignore_index=True)
                validation_rows = validation_rows.drop_duplicates(["instance_id", "method_name"], keep="last")
            _write_partial(run_status, run_path)
            _write_partial(customer_rows, customer_path)
            _write_partial(customer_detail_rows, customer_detail_path)
            _write_partial(decision_rows, decision_path)
            _write_partial(reconstruction_rows, reconstruction_path)
            _write_partial(paper_rows, paper_rows_path)
            _write_partial(validation_rows, validation_path)

    table = _aggregate(run_status, customer_rows)
    output_path = output_dir / "table_priority_service_speed_by_method.csv"
    table.to_csv(output_path, index=False, float_format="%.6f")
    reconstruction_rows.to_csv(reconstruction_path, index=False, float_format="%.10f")
    customer_detail_rows.to_csv(customer_detail_path, index=False, float_format="%.10f")
    decision_rows.to_csv(decision_path, index=False, float_format="%.10f")
    validation_rows.to_csv(validation_path, index=False, float_format="%.10f")
    paper_rows = paper_rows.drop_duplicates(["instance_id", "method_name"], keep="last")
    paper_rows["final_solution_feasibility_flag"] = (
        paper_rows["final_solution_feasibility_flag"]
        .astype(str)
        .str.strip()
        .str.lower()
        .map({"true": True, "false": False, "1": True, "0": False})
        .fillna(False)
        .astype(bool)
    )
    paper_rows = _attach_relative_fields(paper_rows)
    paper_rows = _attach_reporting_fields(paper_rows, main_config)
    tables_dir = output_dir / "tables"
    tables_dir.mkdir(parents=True, exist_ok=True)
    paper_rows.to_csv(tables_dir / "heuristic_study_runs.csv", index=False)
    _generate_tables(paper_rows, _as_levels(main_config), output_dir)
    _generate_feasibility_disaggregation(paper_rows, output_dir)
    _generate_figures(paper_rows, output_dir, instance_cache, solution_cache)
    _build_decision_quality_tables(paper_rows, customer_detail_rows, output_dir)
    table.to_csv(tables_dir / "table_priority_service_speed_by_method.csv", index=False, float_format="%.6f")
    successful_pairs = run_status[run_status["error"].fillna("").astype(str).eq("")][
        ["instance_id", "method_name"]
    ].drop_duplicates()
    expected_pairs = len(meta) * len(selected_methods)
    provenance.update(
        {
            "run_status": "complete" if len(successful_pairs) == expected_pairs else "incomplete",
            "run_finished_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "method_rows": int(len(run_status)),
            "paper_table_rows": int(len(paper_rows)),
            "customer_detail_rows": int(len(customer_detail_rows)),
            "decision_log_rows": int(len(decision_rows)),
            "reconstruction_log_rows": int(len(reconstruction_rows)),
            "objective_replay_matches": int(validation_rows["objective_match"].fillna(False).astype(bool).sum()),
            "objective_replay_mismatches": int((~validation_rows["objective_match"].fillna(False).astype(bool)).sum()),
        }
    )
    provenance_path.write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    _log(log_path, f"Priority service-speed study complete | rows={len(table)} | output={output_path}")
    return table


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--main-config",
        type=Path,
        default=Path("config/experiment_doptimal_beta010_tables5_14.yaml"),
    )
    parser.add_argument(
        "--alns-config",
        type=Path,
        default=Path("config/experiment_alns_comparison.yaml"),
    )
    parser.add_argument(
        "--source-output-dir",
        type=Path,
        default=Path("outputs/doptimal_full_beta010_tables5_14_rerun2_20260601_144538"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/priority_service_speed_by_method"),
    )
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--end-index", type=int, default=None)
    parser.add_argument("--methods", nargs="+", choices=ALL_METHODS, default=None)
    parser.add_argument(
        "--common-time-limit",
        type=float,
        default=None,
        help="Use the same positive wall-clock limit in seconds for every selected method and instance.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_pipeline(
        main_config_path=args.main_config,
        alns_config_path=args.alns_config,
        source_output_dir=args.source_output_dir,
        output_dir=args.output_dir,
        limit=args.limit,
        start_index=args.start_index,
        end_index=args.end_index,
        methods=args.methods,
        common_time_limit=args.common_time_limit,
    )


if __name__ == "__main__":
    main()


