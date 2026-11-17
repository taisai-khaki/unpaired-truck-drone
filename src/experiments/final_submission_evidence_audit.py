from __future__ import annotations

import csv
import gzip
import hashlib
import json
import math
import platform
import random
import statistics
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from dataclasses import fields
from pathlib import Path
from typing import Any

import pulp
import yaml

from ..feasibility import validate_solution
from ..data_models import SolutionData
from ..distance_utils import euclidean_distance, manhattan_distance
from ..heuristics.architecture import validate_drone_architecture
from ..heuristics.baselines import (
    nils_no_local_search_baseline,
    nils_no_perturbation_baseline,
    simple_drone_assignment_baseline,
)
from ..heuristics.construction import _fill_solution_from_routes
from ..heuristics.nils import default_drone_to_truck, evaluate_solution
from ..instance_generator import load_instance_json
from ..milp.backend import _read_cbc_result
from ..milp.solver import solve_instance
from ..parameters import build_search_config
from ..timing import compute_synchronization_waiting
from .heuristic_study import _scenario_config
from .refresh_legacy_semantics import (
    INT_MAP_FIELDS,
    LAMBDAS,
    METHODS,
    ROUTE_MAP_FIELDS,
    TUPLE_MAP_FIELDS,
    solution_from_payload,
    sortie_diagnostics,
)
from .run_final_720_semantics_corrected import (
    SOURCE_BACKBONES,
    SOURCE_RAW,
    _scenario_from_instance,
    _sha256_file,
    _truck_routes_sha256,
)


REPO = Path(__file__).resolve().parents[2]
FROZEN_OUTPUT = REPO / "outputs/final_m3_m4_m5_alns_720_semantics_corrected_20261113"
LEGACY_OUTPUT = REPO / "outputs/final_legacy_semantics_refresh_20261114"
OUTPUT = REPO / "outputs/final_submission_evidence_audit_20261117"
SMALL_INPUTS = LEGACY_OUTPUT / "inputs/small_instances"
SMALL_RUNS = LEGACY_OUTPUT / "raw/exact_nils_restarts.jsonl"
SMALL_MANIFEST = LEGACY_OUTPUT / "run_manifest.json"
AUDIT_TOLERANCE = 1e-6
EXACT_SIZES = (3, 4, 5, 6, 7, 8, 9)
EXACT_SEEDS = {3: 5027, 4: 6027, 5: 7027, 6: 8027, 7: 9027, 8: 10027, 9: 11027}


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    headers = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _solution_from_final_payload(payload: dict[str, Any]) -> SolutionData:
    values: dict[str, Any] = {}
    for field in fields(SolutionData):
        name = field.name
        if name not in payload:
            continue
        raw = payload[name]
        if name in TUPLE_MAP_FIELDS:
            if isinstance(raw, list):
                values[name] = {tuple(item["key"]): item["value"] for item in raw}
            elif isinstance(raw, dict):
                values[name] = {
                    tuple(int(part) for part in key.split(",")): value
                    for key, value in raw.items()
                }
        elif name in INT_MAP_FIELDS:
            if isinstance(raw, list):
                values[name] = {int(item["key"]): item["value"] for item in raw}
            elif isinstance(raw, dict):
                values[name] = {int(key): value for key, value in raw.items()}
        elif name in ROUTE_MAP_FIELDS:
            values[name] = {int(key): item for key, item in raw.items()}
        elif name == "sync_events":
            values[name] = []
        elif name == "tags":
            values[name] = tuple(raw or ())
        else:
            values[name] = raw
    solution = SolutionData(**values)
    for name, value in (payload.get("dynamic_fields") or {}).items():
        setattr(solution, name, value)
    return solution


def verify_frozen_artifacts() -> dict[str, Any]:
    manifest_path = FROZEN_OUTPUT / "artifact_sha256_manifest.csv"
    entries = _read_csv(manifest_path)
    missing: list[str] = []
    changed: list[str] = []
    for entry in entries:
        path = REPO / entry["path"]
        if not path.is_file():
            missing.append(entry["path"])
            continue
        actual = _sha256_file(path)
        if actual != entry["sha256"]:
            changed.append(entry["path"])
    return {
        "manifest_entries": len(entries),
        "missing_files": missing,
        "changed_files": changed,
        "passed": not missing and not changed,
    }


def revalidate_frozen_720() -> dict[str, Any]:
    started = time.perf_counter()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    input_rows = _read_csv(FROZEN_OUTPUT / "input_manifest.csv")
    if len(input_rows) != 720:
        raise RuntimeError(f"Expected 720 frozen inputs, found {len(input_rows)}")
    by_instance = {row["instance_id"]: row for row in input_rows}
    artifact_audit = verify_frozen_artifacts()
    if not artifact_audit["passed"]:
        raise RuntimeError("The frozen principal output failed its archived SHA-256 manifest")

    rows: list[dict[str, Any]] = []
    high_priority_rows: list[dict[str, Any]] = []
    instance_cache: dict[str, Any] = {}
    record_paths = sorted((FROZEN_OUTPUT / "run_records").glob("*.json.gz"))
    if len(record_paths) != 2880:
        raise RuntimeError(f"Expected 2,880 principal run records, found {len(record_paths)}")
    for path in record_paths:
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            record = json.load(stream)
        instance_id = str(record["instance_id"])
        input_row = by_instance[instance_id]
        if record["instance_sha256"] != input_row["instance_sha256"]:
            raise RuntimeError(f"Run/input hash mismatch for {instance_id}")
        source_instance = SOURCE_RAW / f"{instance_id}.json"
        if _sha256_file(source_instance) != input_row["instance_sha256"]:
            raise RuntimeError(f"Frozen instance changed for {instance_id}")
        backbone_path = REPO / input_row["backbone_file"]
        if _sha256_file(backbone_path) != input_row["backbone_file_sha256"]:
            raise RuntimeError(f"Frozen backbone changed for {instance_id}")
        if instance_id not in instance_cache:
            instance_cache[instance_id] = load_instance_json(source_instance)
        instance = instance_cache[instance_id]
        solution = _solution_from_final_payload(record["evaluated_solution"])
        old_objective = float(record["row"]["recomputed_objective"])
        objective, components = evaluate_solution(instance, solution)
        findings = validate_solution(instance, solution)
        errors = [item for item in findings if item.severity == "ERROR"]
        architecture = str(record["row"]["architecture"])
        pairing = default_drone_to_truck(instance) if architecture == "M3" else None
        architecture_errors = validate_drone_architecture(
            instance, solution, architecture, pairing
        )
        priority_tardiness = {
            priority: sum(
                float(solution.tardiness.get(customer.node_id, 0.0))
                for customer in instance.customers
                if customer.priority == priority
            )
            for priority in ("high", "medium", "low")
        }
        service_by_customer: dict[int, tuple[str, float]] = {}
        for (truck_id, customer_id), served in solution.u_truck.items():
            if served >= 0.5:
                service_by_customer[int(customer_id)] = (
                    "truck",
                    float(solution.a_truck.get((int(truck_id), int(customer_id)), math.nan)),
                )
        for (drone_id, customer_id), served in solution.u_drone.items():
            if served >= 0.5:
                service_by_customer[int(customer_id)] = (
                    "drone",
                    float(solution.a_drone.get((int(drone_id), int(customer_id)), math.nan)),
                )
        high_customers = [customer for customer in instance.customers if customer.priority == "high"]
        high_delivery_times = [
            service_by_customer.get(customer.node_id, ("unserved", math.nan))[1]
            for customer in high_customers
        ]
        served_by_drone = {
            int(customer_id)
            for (_, customer_id), served in solution.u_drone.items()
            if served >= 0.5
        }
        coordinates = instance.coordinate_map()
        truck_distance = sum(
            manhattan_distance(coordinates[left], coordinates[right])
            for route in solution.truck_routes.values()
            for left, right in zip(route, route[1:])
        )
        drone_distance = sum(
            euclidean_distance(coordinates[left], coordinates[right])
            for route in solution.drone_routes.values()
            for left, right in zip(route, route[1:])
        )
        sync_diagnostics = compute_synchronization_waiting(instance, solution)
        sortie_metrics = sortie_diagnostics(instance, solution)
        input_metrics = {
            key: input_row.get(key)
            for key in (
                "scenario_id",
                "eligible_share_target",
                "eligible_share_realized",
                "endurance_level",
                "speed_ratio",
                "handling_time_class",
                "spatial_pattern",
                "priority_weight_profile",
                "number_of_drones",
                "matched_truck_only_objective",
            )
        }
        row = {
            **input_metrics,
            "instance_id": instance_id,
            "size": instance.num_customers,
            "scenario_class_id": record["row"].get("scenario_class_id"),
            "method": str(record["method_name"]),
            "architecture": architecture,
            "old_objective": old_objective,
            "corrected_evaluator_objective": float(objective),
            "objective_delta_corrected_minus_archived": float(objective - old_objective),
            "objective_changed_over_tolerance": abs(objective - old_objective) > AUDIT_TOLERANCE,
            "old_feasible": bool(record["row"].get("full_feasibility")),
            "corrected_feasible": not errors,
            "corrected_architecture_valid": not architecture_errors,
            "invalidated_previously_accepted": bool(record["row"].get("full_feasibility"))
            and (bool(errors) or bool(architecture_errors)),
            "truck_cost": float(components["truck_cost"]),
            "drone_cost": float(components["drone_cost"]),
            "priority_tardiness_cost": float(components["tardiness_cost"]),
            "synchronization_wait_minutes": float(components["synchronization_wait_minutes"]),
            "high_priority_tardiness": priority_tardiness["high"],
            "medium_priority_tardiness": priority_tardiness["medium"],
            "low_priority_tardiness": priority_tardiness["low"],
            "truck_distance": float(truck_distance),
            "drone_distance": float(drone_distance),
            "drone_served_customers": len(served_by_drone),
            "drone_share": len(served_by_drone) / max(1, instance.num_customers),
            "active_drones_used": sum(
                bool(any(int(node) != 0 for node in route))
                for route in solution.drone_routes.values()
            ),
            "mean_high_priority_delivery_time": (
                sum(value for value in high_delivery_times if math.isfinite(value))
                / max(1, sum(math.isfinite(value) for value in high_delivery_times))
                if high_delivery_times
                else math.nan
            ),
            "high_priority_on_time_rate": (
                sum(
                    math.isfinite(service_by_customer.get(customer.node_id, ("unserved", math.nan))[1])
                    and service_by_customer.get(customer.node_id, ("unserved", math.nan))[1] <= customer.ub + 1e-9
                    for customer in high_customers
                )
                / max(1, len(high_customers))
            ),
            "synchronization_waiting_minutes": float(sync_diagnostics["total_waiting_minutes"]),
            "synchronization_handling_minutes": sum(
                float(event["handling_minutes"]) for event in sync_diagnostics["events"]
            ),
            "physical_rendezvous_count": len(sync_diagnostics["events"]),
            "parcel_reload_count": sum(value > 0.5 for value in solution.z1.values()),
            "battery_swap_count": sum(value > 0.5 for value in solution.z2.values()),
            "cpu_time_seconds": record["row"].get("elapsed_wall_time_seconds"),
            **sortie_metrics,
            "feasibility_errors": json.dumps(
                [{"code": item.code, "message": item.message} for item in errors],
                sort_keys=True,
            ),
            "architecture_errors": json.dumps(architecture_errors, sort_keys=True),
            "record_sha256": _sha256_file(path),
        }
        rows.append(row)
        for customer in high_customers:
            service_mode, delivery_time = service_by_customer.get(
                customer.node_id, ("unserved", math.nan)
            )
            high_priority_rows.append(
                {
                    "instance_id": instance_id,
                    "size": instance.num_customers,
                    "method": str(record["method_name"]),
                    "customer_id": customer.node_id,
                    "delivery_time": delivery_time,
                    "deadline": customer.ub,
                    "tardiness": max(0.0, delivery_time - customer.ub)
                    if math.isfinite(delivery_time)
                    else math.nan,
                    "on_time": math.isfinite(delivery_time) and delivery_time <= customer.ub + 1e-9,
                    "service_mode": service_mode,
                }
            )

    _write_csv(OUTPUT / "frozen_720_revalidation.csv", rows)
    by_method: dict[str, dict[str, Any]] = {}
    for method in sorted({row["method"] for row in rows}):
        subset = [row for row in rows if row["method"] == method]
        changes = [row for row in subset if row["objective_changed_over_tolerance"]]
        invalidated = [row for row in subset if row["invalidated_previously_accepted"]]
        by_method[method] = {
            "records": len(subset),
            "objective_changed_over_tolerance": len(changes),
            "max_abs_objective_change": max(
                (abs(row["objective_delta_corrected_minus_archived"]) for row in subset),
                default=0.0,
            ),
            "mean_objective_change": sum(
                row["objective_delta_corrected_minus_archived"] for row in subset
            ) / max(1, len(subset)),
            "corrected_feasible": sum(bool(row["corrected_feasible"]) for row in subset),
            "corrected_architecture_valid": sum(
                bool(row["corrected_architecture_valid"]) for row in subset
            ),
            "previously_accepted_invalidated": len(invalidated),
        }
    method_size_summary: list[dict[str, Any]] = []
    for method in sorted({row["method"] for row in rows}):
        for size in sorted({int(row["size"]) for row in rows}):
            subset = [row for row in rows if row["method"] == method and int(row["size"]) == size]
            if not subset:
                continue
            values = [float(row["corrected_evaluator_objective"]) for row in subset]
            references = [float(row["matched_truck_only_objective"]) for row in subset]
            relative_indices = [
                100.0 * value / abs(reference)
                for value, reference in zip(values, references)
                if abs(reference) > 1e-12
            ]
            ordered_values = sorted(values)
            method_size_summary.append(
                {
                    "method": method,
                    "size": size,
                    "runs": len(subset),
                    "mean_corrected_objective": sum(values) / len(values),
                    "median_corrected_objective": statistics.median(values),
                    "mean_relative_objective_index": (
                        sum(relative_indices) / len(relative_indices) if relative_indices else math.nan
                    ),
                    "feasible_runs": sum(bool(row["corrected_feasible"]) for row in subset),
                    "architecture_valid_runs": sum(
                        bool(row["corrected_architecture_valid"]) for row in subset
                    ),
                    "mean_high_priority_tardiness": statistics.mean(
                        float(row["high_priority_tardiness"]) for row in subset
                    ),
                    "mean_medium_priority_tardiness": statistics.mean(
                        float(row["medium_priority_tardiness"]) for row in subset
                    ),
                    "mean_low_priority_tardiness": statistics.mean(
                        float(row["low_priority_tardiness"]) for row in subset
                    ),
                    "mean_priority_tardiness_cost": statistics.mean(
                        float(row["priority_tardiness_cost"]) for row in subset
                    ),
                    "mean_drone_share": statistics.mean(float(row["drone_share"]) for row in subset),
                    "mean_drone_distance": statistics.mean(float(row["drone_distance"]) for row in subset),
                    "mean_truck_distance": statistics.mean(float(row["truck_distance"]) for row in subset),
                    "mean_synchronization_waiting_minutes": statistics.mean(
                        float(row["synchronization_waiting_minutes"]) for row in subset
                    ),
                    "mean_synchronization_handling_minutes": statistics.mean(
                        float(row["synchronization_handling_minutes"]) for row in subset
                    ),
                    "mean_cpu_seconds": statistics.mean(
                        float(row["cpu_time_seconds"]) for row in subset if _optional_float(row["cpu_time_seconds"]) is not None
                    ),
                }
            )
    for size in sorted({int(row["n"]) for row in input_rows}):
        reference_values = [
            float(row["matched_truck_only_objective"])
            for row in input_rows
            if int(row["n"]) == size
        ]
        method_size_summary.append(
            {
                "method": "truck_only",
                "size": size,
                "runs": len(reference_values),
                "mean_corrected_objective": statistics.mean(reference_values),
                "median_corrected_objective": statistics.median(reference_values),
                "mean_relative_objective_index": 100.0,
                "feasible_runs": sum(
                    str(row["matched_truck_only_feasible"]).lower() == "true"
                    for row in input_rows
                    if int(row["n"]) == size
                ),
                "architecture_valid_runs": len(reference_values),
            }
        )
    _write_csv(OUTPUT / "frozen_720_corrected_summary_by_method_size.csv", method_size_summary)
    _write_csv(OUTPUT / "frozen_720_high_priority_delivery_detail.csv", high_priority_rows)

    corrected_objectives = {
        (row["instance_id"], row["method"]): float(row["corrected_evaluator_objective"])
        for row in rows
    }
    objective_pairs: list[dict[str, Any]] = []
    method_aliases = {
        "paired_baseline": "M3",
        "no_unpairing": "M4",
        "nils": "M5",
        "feasibility_aware_alns": "M8",
    }
    instance_sizes = {row["instance_id"]: int(row["n"]) for row in input_rows}
    try:
        from scipy.stats import wilcoxon
    except Exception:
        wilcoxon = None
    comparison_groups: list[tuple[str, int | None]] = [("All", None)] + [
        (str(size), size) for size in sorted(set(instance_sizes.values()))
    ]
    for comparator in ("paired_baseline", "no_unpairing", "feasibility_aware_alns"):
        for size_label, size in comparison_groups:
            paired_values = [
                (
                    corrected_objectives[(instance_id, "nils")],
                    corrected_objectives[(instance_id, comparator)],
                )
                for instance_id in instance_sizes
                if size is None or instance_sizes[instance_id] == size
            ]
            wins = sum(nils < comp - 1e-9 for nils, comp in paired_values)
            losses = sum(nils > comp + 1e-9 for nils, comp in paired_values)
            improvements = [100.0 * (comp - nils) / abs(comp) for nils, comp in paired_values if abs(comp) > 1e-12]
            sorted_improvements = sorted(improvements)
            mid = len(sorted_improvements) // 2
            median_imp = (
                sorted_improvements[mid]
                if len(sorted_improvements) % 2
                else (sorted_improvements[mid - 1] + sorted_improvements[mid]) / 2.0
            ) if sorted_improvements else math.nan
            try:
                wilcoxon_p = (
                    float(wilcoxon(
                        [nils for nils, _ in paired_values],
                        [comp for _, comp in paired_values],
                        zero_method="wilcox",
                    ).pvalue)
                    if wilcoxon is not None and any(abs(nils - comp) > 1e-12 for nils, comp in paired_values)
                    else 1.0
                )
            except ValueError:
                wilcoxon_p = math.nan
            objective_pairs.append(
                {
                    "size": size_label,
                    "comparison": f"M5_vs_{method_aliases[comparator]}",
                    "pairs": len(paired_values),
                    "mean_Z_M5": statistics.mean(nils for nils, _ in paired_values),
                    "mean_Z_comparator": statistics.mean(comp for _, comp in paired_values),
                    "M5_wins": wins,
                    "M5_losses": losses,
                    "ties": len(paired_values) - wins - losses,
                    "mean_improvement_percent": sum(improvements) / len(improvements) if improvements else math.nan,
                    "median_improvement_percent": median_imp,
                    "wilcoxon_p": wilcoxon_p,
                }
            )
    _write_csv(OUTPUT / "frozen_720_corrected_matched_comparisons.csv", objective_pairs)
    hpt_objectives = {
        (row["instance_id"], row["method"]): float(row["high_priority_tardiness"])
        for row in rows
    }
    hpt_comparisons: list[dict[str, Any]] = []
    for comparator in ("paired_baseline", "no_unpairing", "feasibility_aware_alns"):
        for size_label, size in comparison_groups:
            pairs = [
                (
                    hpt_objectives[(instance_id, "nils")],
                    hpt_objectives[(instance_id, comparator)],
                )
                for instance_id in instance_sizes
                if size is None or instance_sizes[instance_id] == size
            ]
            nils_wins = sum(left < right - 1e-9 for left, right in pairs)
            nils_losses = sum(left > right + 1e-9 for left, right in pairs)
            try:
                p_value = (
                    float(wilcoxon(
                        [left for left, _ in pairs],
                        [right for _, right in pairs],
                        zero_method="wilcox",
                    ).pvalue)
                    if wilcoxon is not None and any(abs(left - right) > 1e-12 for left, right in pairs)
                    else 1.0
                )
            except ValueError:
                p_value = math.nan
            hpt_comparisons.append(
                {
                    "size": size_label,
                    "comparison": f"M5_HPT_vs_{method_aliases[comparator]}",
                    "pairs": len(pairs),
                    "mean_HPT_M5": statistics.mean(left for left, _ in pairs),
                    "mean_HPT_comparator": statistics.mean(right for _, right in pairs),
                    "M5_lower_HPT": nils_wins,
                    "M5_higher_HPT": nils_losses,
                    "ties": len(pairs) - nils_wins - nils_losses,
                    "wilcoxon_p": p_value,
                }
            )
    _write_csv(OUTPUT / "frozen_720_corrected_hpt_comparisons.csv", hpt_comparisons)
    overall_rows: list[dict[str, Any]] = []
    method_names = sorted({row["method"] for row in rows}) + ["truck_only"]
    for method in method_names:
        method_records = [row for row in rows if row["method"] == method]
        if method == "truck_only":
            objective_values = [float(row["matched_truck_only_objective"]) for row in input_rows]
            reference_values = objective_values
            relative_indices = [100.0] * len(objective_values)
            size_indices = [100.0] * len(set(instance_sizes.values()))
        else:
            objective_values = [float(row["corrected_evaluator_objective"]) for row in method_records]
            reference_values = [float(row["matched_truck_only_objective"]) for row in method_records]
            relative_indices = [
                100.0 * value / abs(reference)
                for value, reference in zip(objective_values, reference_values)
                if abs(reference) > 1e-12
            ]
            size_indices = [
                float(summary_row["mean_relative_objective_index"])
                for summary_row in method_size_summary
                if summary_row["method"] == method
            ]
        overall_rows.append(
            {
                "method": method,
                "runs": len(objective_values),
                "mean_objective": statistics.mean(objective_values),
                "median_objective": statistics.median(objective_values),
                "mean_relative_objective_index_pooled": (
                    statistics.mean(relative_indices) if relative_indices else math.nan
                ),
                "mean_relative_objective_index_size_balanced": (
                    statistics.mean(size_indices) if size_indices else math.nan
                ),
                "mean_improvement_vs_M1_percent_pooled": (
                    statistics.mean(
                        100.0 * (reference - objective) / abs(reference)
                        for reference, objective in zip(reference_values, objective_values)
                        if abs(reference) > 1e-12
                    )
                    if reference_values
                    else math.nan
                ),
            }
        )
    _write_csv(OUTPUT / "frozen_720_corrected_overall_summary.csv", overall_rows)
    operational_fields = (
        "mean_corrected_objective",
        "mean_relative_objective_index",
        "mean_high_priority_tardiness",
        "mean_medium_priority_tardiness",
        "mean_low_priority_tardiness",
        "mean_priority_tardiness_cost",
        "mean_drone_share",
        "mean_drone_distance",
        "mean_truck_distance",
        "mean_synchronization_waiting_minutes",
        "mean_synchronization_handling_minutes",
        "mean_cpu_seconds",
    )
    operational_rows = list(method_size_summary)
    for method in sorted({row["method"] for row in rows}):
        subset = [row for row in rows if row["method"] == method]
        operational_rows.append(
            {
                "method": method,
                "size": "All",
                "runs": len(subset),
                **{
                    field: statistics.mean(
                        float(row[field.removeprefix("mean_")]) for row in subset
                    )
                    for field in operational_fields
                    if field.removeprefix("mean_") in subset[0]
                },
            }
        )
    _write_csv(OUTPUT / "frozen_720_corrected_operational_summary.csv", operational_rows)

    factor_rows: list[dict[str, Any]] = []
    nil_rows = [row for row in rows if row["method"] == "nils"]
    for factor_names in (
        ("endurance_level", "speed_ratio"),
        ("eligible_share_target", "spatial_pattern"),
        ("priority_weight_profile",),
        ("size", "number_of_drones"),
    ):
        groups: dict[tuple[str, ...], list[dict[str, Any]]] = {}
        for row in nil_rows:
            key = tuple(str(row[name]) for name in factor_names)
            groups.setdefault(key, []).append(row)
        for values_key, subset in groups.items():
            means = statistics.mean(float(row["corrected_evaluator_objective"]) for row in subset)
            indices = [
                100.0 * float(row["corrected_evaluator_objective"])
                / abs(float(row["matched_truck_only_objective"]))
                for row in subset
                if abs(float(row["matched_truck_only_objective"])) > 1e-12
            ]
            factor_rows.append(
                {
                    "factor": "+".join(factor_names),
                    "level": "+".join(values_key),
                    "runs": len(subset),
                    "mean_objective": means,
                    "mean_relative_objective_index": statistics.mean(indices) if indices else math.nan,
                    "mean_improvement_vs_M1_percent": 100.0 - statistics.mean(indices) if indices else math.nan,
                    "mean_high_priority_tardiness": statistics.mean(
                        float(row["high_priority_tardiness"]) for row in subset
                    ),
                    "mean_drone_share": statistics.mean(float(row["drone_share"]) for row in subset),
                    "mean_cpu_seconds": statistics.mean(float(row["cpu_time_seconds"]) for row in subset),
                }
            )
    _write_csv(OUTPUT / "frozen_720_corrected_factor_summary.csv", factor_rows)

    def percentile(values: list[float], fraction: float) -> float:
        ordered = sorted(values)
        if not ordered:
            return math.nan
        index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
        return ordered[index]

    priority_service_rows: list[dict[str, Any]] = []
    priority_methods = sorted({row["method"] for row in high_priority_rows})
    for method in priority_methods:
        for size_label, size in comparison_groups:
            subset = [
                row
                for row in high_priority_rows
                if row["method"] == method and (size is None or int(row["size"]) == size)
            ]
            delivery = [float(row["delivery_time"]) for row in subset if math.isfinite(float(row["delivery_time"]))]
            tardiness = [float(row["tardiness"]) for row in subset if math.isfinite(float(row["tardiness"]))]
            on_time = [bool(row["on_time"]) for row in subset]
            early = [
                float(row["deadline"]) - float(row["delivery_time"])
                for row in subset
                if bool(row["on_time"]) and math.isfinite(float(row["delivery_time"]))
            ]
            relative_deadline = [
                float(row["delivery_time"]) / float(row["deadline"])
                for row in subset
                if float(row["deadline"]) > 1e-12 and math.isfinite(float(row["delivery_time"]))
            ]
            priority_service_rows.append(
                {
                    "method": method,
                    "size": size_label,
                    "high_priority_customer_records": len(subset),
                    "mean_delivery_time": statistics.mean(delivery) if delivery else math.nan,
                    "median_delivery_time": statistics.median(delivery) if delivery else math.nan,
                    "p90_delivery_time": percentile(delivery, 0.90),
                    "mean_tardiness": statistics.mean(tardiness) if tardiness else math.nan,
                    "median_tardiness": statistics.median(tardiness) if tardiness else math.nan,
                    "p90_tardiness": percentile(tardiness, 0.90),
                    "max_tardiness": max(tardiness) if tardiness else math.nan,
                    "on_time_rate": statistics.mean(on_time) if on_time else math.nan,
                    "late_rate": 1.0 - statistics.mean(on_time) if on_time else math.nan,
                    "mean_early_when_on_time": statistics.mean(early) if early else math.nan,
                    "mean_delivery_time_relative_to_deadline": (
                        statistics.mean(relative_deadline) if relative_deadline else math.nan
                    ),
                    "drone_served_share": (
                        sum(row["service_mode"] == "drone" for row in subset) / len(subset)
                        if subset
                        else math.nan
                    ),
                }
            )
    _write_csv(OUTPUT / "frozen_720_corrected_priority_service_summary.csv", priority_service_rows)

    high_by_key = {
        (row["instance_id"], row["customer_id"], row["method"]): row
        for row in high_priority_rows
    }
    customer_service_comparisons: list[dict[str, Any]] = []
    for size_label, size in comparison_groups:
        matched = [
            (m5, m8)
            for (instance_id, customer_id, method), m5 in high_by_key.items()
            if method == "nils"
            and (size is None or int(m5["size"]) == size)
            and (m8 := high_by_key.get((instance_id, customer_id, "feasibility_aware_alns"))) is not None
        ]
        customer_service_comparisons.append(
            {
                "size": size_label,
                "matched_high_priority_customers": len(matched),
                "M5_earlier": sum(float(m5["delivery_time"]) < float(m8["delivery_time"]) - 1e-6 for m5, m8 in matched),
                "M8_earlier": sum(float(m8["delivery_time"]) < float(m5["delivery_time"]) - 1e-6 for m5, m8 in matched),
                "ties": sum(abs(float(m5["delivery_time"]) - float(m8["delivery_time"])) <= 1e-6 for m5, m8 in matched),
                "M5_on_time": sum(bool(m5["on_time"]) for m5, _ in matched),
                "M8_on_time": sum(bool(m8["on_time"]) for _, m8 in matched),
                "M5_only_on_time": sum(bool(m5["on_time"]) and not bool(m8["on_time"]) for m5, m8 in matched),
                "M8_only_on_time": sum(bool(m8["on_time"]) and not bool(m5["on_time"]) for m5, m8 in matched),
            }
        )
    _write_csv(OUTPUT / "frozen_720_corrected_M5_M8_customer_comparison.csv", customer_service_comparisons)

    corrected_paper_metrics = {
        "method_relative_indices": {
            row["method"]: {
                "pooled": row["mean_relative_objective_index_pooled"],
                "size_balanced": row["mean_relative_objective_index_size_balanced"],
            }
            for row in overall_rows
        },
        "method_size_summaries": method_size_summary,
        "matched_objective_comparisons": objective_pairs,
        "matched_HPT_comparisons": hpt_comparisons,
        "priority_customer_comparisons": customer_service_comparisons,
    }
    _write_json(OUTPUT / "corrected_paper_metrics.json", corrected_paper_metrics)
    objective_changes = [row for row in rows if row["objective_changed_over_tolerance"]]
    invalidated_rows = [row for row in rows if row["invalidated_previously_accepted"]]
    summary = {
        "status": "PASS" if not invalidated_rows and not objective_changes else "BLOCKED",
        "frozen_output_directory": str(FROZEN_OUTPUT),
        "frozen_execution_commit": "8abdc6b7d9f82a7dc630468e4f108e48f2246e84",
        "frozen_artifact_audit": artifact_audit,
        "expected_instances": 720,
        "input_records": len(input_rows),
        "expected_method_records": 2880,
        "revalidated_records": len(rows),
        "corrected_evaluator_changed_objectives": len(objective_changes),
        "previously_accepted_records_invalidated": len(invalidated_rows),
        "methods": by_method,
        "elapsed_seconds": time.perf_counter() - started,
        "evaluator_change": "Rendezvous waits are recomputed from each iteration's readiness; obsolete prior-iteration waits are not retained.",
    }
    _write_json(OUTPUT / "frozen_720_revalidation_summary.json", summary)
    return summary


def revalidate_references() -> dict[str, Any]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    frozen_rows = _read_csv(FROZEN_OUTPUT / "input_manifest.csv")
    m1_rows: list[dict[str, Any]] = []
    instance_cache: dict[str, Any] = {}
    for input_row in frozen_rows:
        instance_id = str(input_row["instance_id"])
        instance_path = SOURCE_RAW / f"{instance_id}.json"
        if _sha256_file(instance_path) != input_row["instance_sha256"]:
            raise RuntimeError(f"Frozen M1 input hash changed for {instance_id}")
        instance = instance_cache.setdefault(instance_id, load_instance_json(instance_path))
        routes = {
            int(truck_id): [int(node) for node in route]
            for truck_id, route in json.loads(input_row["backbone_routes_json"]).items()
        }
        solution = _fill_solution_from_routes(instance, routes)
        objective, _ = evaluate_solution(instance, solution)
        errors = [item.message for item in validate_solution(instance, solution) if item.severity == "ERROR"]
        architecture_errors = validate_drone_architecture(instance, solution, "M5")
        archived_objective = float(input_row["matched_truck_only_objective"])
        m1_rows.append(
            {
                "instance_id": instance_id,
                "size": instance.num_customers,
                "input_sha256": input_row["instance_sha256"],
                "archived_objective": archived_objective,
                "recomputed_objective": float(objective),
                "objective_delta": float(objective) - archived_objective,
                "objective_matches": math.isclose(
                    float(objective), archived_objective, rel_tol=1e-9, abs_tol=AUDIT_TOLERANCE
                ),
                "feasible": not errors,
                "architecture_valid": not architecture_errors,
                "feasibility_errors": json.dumps(errors),
                "architecture_errors": json.dumps(architecture_errors),
            }
        )
    _write_csv(OUTPUT / "frozen_720_m1_reference_revalidation.csv", m1_rows)

    method_architecture = {method: architecture for method, _, architecture in METHODS}
    real_input_dir = LEGACY_OUTPUT / "inputs/real_case_instances"
    real_raw_path = LEGACY_OUTPUT / "raw/real_data_lambda_methods.jsonl"
    real_rows: list[dict[str, Any]] = []
    real_instance_cache: dict[float, Any] = {}
    for record in _read_jsonl(real_raw_path):
        lam = round(float(record["lambda"]), 3)
        method = str(record["method"])
        instance_path = real_input_dir / f"real_case_lambda_{lam:.3f}.json"
        if lam not in real_instance_cache:
            real_instance_cache[lam] = load_instance_json(instance_path)
        instance = real_instance_cache[lam]
        solution = solution_from_payload(record["solution"])
        objective, components = evaluate_solution(instance, solution)
        findings = validate_solution(instance, solution)
        feasibility_errors = [item.message for item in findings if item.severity == "ERROR"]
        architecture = method_architecture[method]
        pairing = default_drone_to_truck(instance) if architecture == "M3" else None
        architecture_errors = validate_drone_architecture(instance, solution, architecture, pairing)
        archived_objective = float(record["metrics"]["candidate_objective"])
        real_rows.append(
            {
                "lambda": lam,
                "method": method,
                "architecture": architecture,
                "archived_objective": archived_objective,
                "recomputed_objective": float(objective),
                "objective_delta": float(objective) - archived_objective,
                "objective_changed_over_tolerance": abs(float(objective) - archived_objective) > AUDIT_TOLERANCE,
                "truck_cost": components.get("truck_cost"),
                "drone_cost": components.get("drone_cost"),
                "tardiness_cost": components.get("tardiness_cost"),
                "feasible": not feasibility_errors,
                "architecture_valid": not architecture_errors,
                "feasibility_errors": json.dumps(feasibility_errors),
                "architecture_errors": json.dumps(architecture_errors),
            }
        )
    _write_csv(OUTPUT / "real_data_revalidation.csv", real_rows)

    real_table_rows: list[dict[str, Any]] = []
    for lam in LAMBDAS:
        subset = [row for row in real_rows if math.isclose(row["lambda"], lam)]
        valid = [row for row in subset if row["feasible"] and row["architecture_valid"]]
        ranked = sorted(valid, key=lambda row: float(row["recomputed_objective"]))
        rank_by_method = {row["method"]: index + 1 for index, row in enumerate(ranked)}
        by_method = {row["method"]: row for row in subset}
        m1 = by_method.get("truck_only")
        m3 = by_method.get("paired_baseline")
        m5 = by_method.get("nils")
        m5_valid = m5 and m5["feasible"] and m5["architecture_valid"]
        m1_valid = m1 and m1["feasible"] and m1["architecture_valid"]
        real_table_rows.append(
            {
                "lambda": lam,
                "valid_methods": len(valid),
                "best_method": ranked[0]["method"] if ranked else "",
                "M5_improvement_vs_M1_percent": (
                    100.0 * (float(m1["recomputed_objective"]) - float(m5["recomputed_objective"]))
                    / abs(float(m1["recomputed_objective"]))
                    if m1_valid and m5_valid and abs(float(m1["recomputed_objective"])) > 1e-12
                    else math.nan
                ),
                "M5_rank": rank_by_method.get("nils", ""),
                "M5_minus_M3": (
                    float(m5["recomputed_objective"]) - float(m3["recomputed_objective"])
                    if m3 and m3["feasible"] and m3["architecture_valid"] and m5_valid
                    else math.nan
                ),
            }
        )
    _write_csv(OUTPUT / "real_data_corrected_table15.csv", real_table_rows)

    changed_real = sum(bool(row["objective_changed_over_tolerance"]) for row in real_rows)
    invalidated_real = sum(
        not row["feasible"] or not row["architecture_valid"] for row in real_rows
    )
    summary = {
        "m1_reference_records": len(m1_rows),
        "m1_objectives_reproduced": sum(bool(row["objective_matches"]) for row in m1_rows),
        "m1_feasible": sum(bool(row["feasible"]) for row in m1_rows),
        "m1_architecture_valid": sum(bool(row["architecture_valid"]) for row in m1_rows),
        "real_data_records": len(real_rows),
        "real_data_objectives_changed_over_tolerance": changed_real,
        "real_data_invalid_records": invalidated_real,
        "real_data_valid_methods_by_lambda": {
            f"{row['lambda']:.3f}": row["valid_methods"] for row in real_table_rows
        },
    }
    _write_json(OUTPUT / "reference_revalidation_summary.json", summary)
    return summary


def run_exact_calibration(time_limit_seconds: int = 120) -> dict[str, Any]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "exact_solver_logs").mkdir(parents=True, exist_ok=True)
    manifest = json.loads(SMALL_MANIFEST.read_text(encoding="utf-8"))
    small_manifest = {int(row["size"]): row for row in manifest["input_provenance"]["small_instances"]}
    raw_rows = _read_jsonl(SMALL_RUNS)
    restarts_by_instance: dict[str, list[dict[str, Any]]] = {}
    for row in raw_rows:
        restarts_by_instance.setdefault(str(row["instance_id"]), []).append(row)
    config_data = yaml.safe_load(
        (LEGACY_OUTPUT / "inputs/small_calibration_config.yaml").read_text(encoding="utf-8")
    )
    config = build_search_config(config_data)
    config = config.__class__(
        **{
            **vars(config),
            "milp": config.milp.__class__(
                enabled=True,
                solver_backend="pulp_cbc",
                time_limit_seconds=int(time_limit_seconds),
                mip_gap=0.0,
                threads=1,
                demand_mode=config.milp.demand_mode,
            ),
        }
    )

    result_rows: list[dict[str, Any]] = []
    reconciliation_rows: list[dict[str, Any]] = []
    constraint_rows: list[dict[str, Any]] = []
    certificate_rows: list[dict[str, Any]] = []
    solver_version = "unknown"
    try:
        import subprocess

        version_result = subprocess.run(
            [pulp.PULP_CBC_CMD().path, "-version"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        version_line = next(
            (
                line.strip()
                for line in (version_result.stdout + version_result.stderr).splitlines()
                if line.strip().lower().startswith("version:")
            ),
            "",
        )
        solver_version = f"CBC {version_line.split(':', 1)[1].strip()}" if version_line else "CBC version unavailable"
    except Exception as error:
        solver_version = f"unavailable: {error!r}"

    for size in EXACT_SIZES:
        instance_id = f"small_{size:01d}_01"
        input_row = small_manifest[size]
        if int(input_row["seed"]) != EXACT_SEEDS[size]:
            raise RuntimeError(f"Unexpected saved seed for n={size}")
        input_path = SMALL_INPUTS / f"{instance_id}.json"
        instance_hash = _sha256_file(input_path)
        if instance_hash != input_row["sha256"]:
            raise RuntimeError(f"Frozen exact input hash changed for n={size}")
        instance = load_instance_json(input_path)

        restart_records = restarts_by_instance.get(instance_id, [])
        evaluated_starts: list[tuple[float, dict[str, Any], list[str], list[str]]] = []
        for restart in restart_records:
            payload = restart.get("solution")
            if not payload:
                continue
            candidate = solution_from_payload(payload)
            candidate_objective, _ = evaluate_solution(instance, candidate)
            validation_errors = [
                item.message for item in validate_solution(instance, candidate) if item.severity == "ERROR"
            ]
            architecture_errors = validate_drone_architecture(instance, candidate, "M5")
            if not validation_errors and not architecture_errors:
                evaluated_starts.append(
                    (float(candidate_objective), restart, validation_errors, architecture_errors)
                )
        best_nils = min(evaluated_starts, key=lambda item: item[0]) if evaluated_starts else None

        log_path = OUTPUT / "exact_solver_logs" / f"n{size}_cbc_zero_gap.log"
        started = time.perf_counter()
        solve_error = ""
        solution = None
        try:
            solution = solve_instance(instance, config, solver_log_path=log_path)
        except Exception as error:
            solve_error = repr(error)
        wall_seconds = time.perf_counter() - started
        if solution is None or solution.status == "no_solution":
            result_rows.append(
                {
                    "instance_id": instance_id,
                    "size": size,
                    "seed": EXACT_SEEDS[size],
                    "input_sha256": instance_hash,
                    "solver": "CBC",
                    "solver_version": solver_version,
                    "solver_status": "exception_or_no_solution",
                    "solver_log_status": "",
                    "solver_runtime_seconds": wall_seconds,
                    "wall_runtime_seconds": wall_seconds,
                    "primal_objective": None,
                    "dual_bound": None,
                    "relative_gap": None,
                    "proven_optimal": False,
                    "nils_valid_restarts": len(evaluated_starts),
                    "nils_best_objective": best_nils[0] if best_nils else None,
                    "solver_error": solve_error,
                }
            )
            continue

        native_objective = float(solution.objective)
        native_components = dict(solution.components)
        native_snapshot = solution.__dict__.get("milp_native_snapshot", {})
        evaluator_objective, evaluator_components = evaluate_solution(instance, solution)
        findings = validate_solution(instance, solution)
        feasibility_errors = [item for item in findings if item.severity == "ERROR"]
        architecture_errors = validate_drone_architecture(instance, solution, "M5")
        log_result = _read_cbc_result(str(log_path))
        log_optimal = (
            "optimal solution found" in str(log_result["result"]).lower()
            and "within gap tolerance" not in str(log_result["result"]).lower()
        )
        logged_bound = log_result["best_bound"]
        best_bound = (
            float(logged_bound)
            if logged_bound is not None
            else (native_objective if log_optimal else None)
        )
        bound_source = (
            "cbc_log_lower_bound"
            if logged_bound is not None
            else "implied_by_cbc_optimal_termination"
            if log_optimal
            else "not_reported"
        )
        relative_gap = (
            max(0.0, (native_objective - float(best_bound)) / max(abs(native_objective), 1e-12))
            if best_bound is not None
            else None
        )
        bound_tight = (
            best_bound is not None
            and math.isfinite(float(best_bound))
            and math.isclose(float(best_bound), native_objective, rel_tol=1e-7, abs_tol=1e-6)
        )
        obj_consistent = math.isclose(native_objective, float(evaluator_objective), rel_tol=1e-8, abs_tol=1e-6)
        nils_consistent = not best_nils or native_objective <= best_nils[0] + max(1e-6, abs(best_nils[0]) * 1e-8)
        proven = bool(log_optimal and bound_tight and obj_consistent and not feasibility_errors and not architecture_errors and nils_consistent)
        snapshot_tardiness = native_snapshot.get("tardiness", {})
        model_components_sum = sum(
            float(native_components.get(name, 0.0)) for name in ("truck_cost", "drone_cost", "tardiness_cost")
        )
        result_rows.append(
            {
                "instance_id": instance_id,
                "size": size,
                "seed": EXACT_SEEDS[size],
                "input_sha256": instance_hash,
                "solver": "CBC",
                "solver_version": solver_version,
                "solver_status": solution.status,
                "solver_log_status": log_result["result"],
                "solver_runtime_seconds": native_components.get("runtime_seconds"),
                "wall_runtime_seconds": wall_seconds,
                "primal_objective": native_objective,
                "dual_bound": best_bound if best_bound is not None and math.isfinite(float(best_bound)) else None,
                "dual_bound_source": bound_source,
                "relative_gap": relative_gap if relative_gap is not None and math.isfinite(float(relative_gap)) else None,
                "proven_optimal": proven,
                "independently_evaluated_objective": float(evaluator_objective),
                "objective_consistent": obj_consistent,
                "model_objective_component_sum": model_components_sum,
                "model_components_sum_consistent": math.isclose(native_objective, model_components_sum, rel_tol=1e-8, abs_tol=1e-6),
                "nils_restart_count": len(restart_records),
                "nils_valid_restarts": len(evaluated_starts),
                "nils_best_objective": best_nils[0] if best_nils else None,
                "nils_gap_only_if_certified": (best_nils[0] - native_objective) if proven and best_nils else None,
                "feasible": not feasibility_errors,
                "architecture_valid": not architecture_errors,
                "solver_error": solve_error,
            }
        )
        reconciliation_rows.append(
            {
                "instance_id": instance_id,
                "size": size,
                "solver_objective": native_objective,
                "solver_truck_cost": native_components.get("truck_cost"),
                "solver_drone_cost": native_components.get("drone_cost"),
                "solver_tardiness_cost": native_components.get("tardiness_cost"),
                "evaluator_objective": float(evaluator_objective),
                "evaluator_truck_cost": evaluator_components.get("truck_cost"),
                "evaluator_drone_cost": evaluator_components.get("drone_cost"),
                "evaluator_tardiness_cost": evaluator_components.get("tardiness_cost"),
                "objective_delta": float(evaluator_objective - native_objective),
                "native_model_customer_tardiness_json": json.dumps(snapshot_tardiness, sort_keys=True),
                "feasibility_errors": json.dumps([item.message for item in feasibility_errors]),
                "architecture_errors": json.dumps(architecture_errors),
                "validator_pass": not feasibility_errors,
                "architecture_pass": not architecture_errors,
            }
        )
        constraint_rows.append(
            {
                "instance_id": instance_id,
                "size": size,
                "route_customers_truck": sum(
                    1 for route in solution.truck_routes.values() for node in route if node
                ),
                "route_customers_drone": sum(
                    1 for route in solution.drone_routes.values() for node in route if node
                ),
                "customers_served": len(solution.served_by_truck() | solution.served_by_drone()),
                "physical_rendezvous": sum(value > 0.5 for value in solution.u_sync.values()),
                "parcel_reload_events": sum(value > 0.5 for value in solution.z1.values()),
                "battery_swap_events": sum(value > 0.5 for value in solution.z2.values()),
                "loaded_arc_count": sum(value > 0.5 for value in solution.y_loaded.values()),
                "full_feasibility_errors": json.dumps([item.message for item in feasibility_errors]),
                "architecture_errors": json.dumps(architecture_errors),
                "native_arrival_truck_json": json.dumps(native_snapshot.get("a_truck", {}), sort_keys=True),
                "native_arrival_drone_json": json.dumps(native_snapshot.get("a_drone", {}), sort_keys=True),
                "native_departure_truck_json": json.dumps(native_snapshot.get("l_truck", {}), sort_keys=True),
                "native_departure_drone_json": json.dumps(native_snapshot.get("l_drone", {}), sort_keys=True),
                "native_loaded_arcs_json": json.dumps(native_snapshot.get("lambda_loaded", {}), sort_keys=True),
                "native_battery_arrival_json": json.dumps(native_snapshot.get("r", {}), sort_keys=True),
                "native_battery_departure_json": json.dumps(native_snapshot.get("r_departure", {}), sort_keys=True),
            }
        )
        certificate_rows.append(
            {
                "instance_id": instance_id,
                "size": size,
                "solver": "CBC",
                "solver_version": solver_version,
                "log_file": str(log_path.relative_to(REPO)),
                "log_result": log_result["result"],
                "log_incumbent": log_result["objective"],
                "log_lower_bound": log_result["best_bound"],
                "solver_api_primal": native_objective,
                "cbc_logged_bound": logged_bound,
                "dual_bound_used_for_certificate": best_bound,
                "dual_bound_source": bound_source,
                "relative_gap_used_for_certificate": relative_gap,
                "bound_tight": bound_tight,
                "evaluator_objective_matches": obj_consistent,
                "feasibility_valid": not feasibility_errors,
                "architecture_valid": not architecture_errors,
                "best_feasible_nils_objective": best_nils[0] if best_nils else None,
                "primal_not_above_feasible_nils": nils_consistent,
                "certified_optimum": proven,
            }
        )

    _write_csv(OUTPUT / "exact_results.csv", result_rows)
    _write_csv(OUTPUT / "exact_objective_reconciliation.csv", reconciliation_rows)
    _write_csv(OUTPUT / "exact_constraint_diagnostics.csv", constraint_rows)
    _write_csv(OUTPUT / "exact_certificates.csv", certificate_rows)
    certified = sum(bool(row.get("proven_optimal")) for row in result_rows)
    summary_lines = [
        "# Exact MILP audit summary",
        "",
        f"Status: {'PASS' if certified == 7 else 'BLOCKED'} ({certified}/7 proven optima).",
        f"Solver: CBC ({solver_version}); zero relative-gap request; {time_limit_seconds}s per instance.",
        "The CBC log, not PuLP's status field, is used to determine native termination and bounds.",
        "An optimality gap is reported only when the solver log supplies an incumbent and lower bound.",
        "NILS starts are re-evaluated with the corrected readiness-based schedule; a NILS gap is reported only when every certification gate passes.",
        "",
        "## Case status",
        "",
    ]
    for row in result_rows:
        summary_lines.append(
            f"- n={row['size']}: status `{row.get('solver_status')}`, log `{row.get('solver_log_status')}`, "
            f"incumbent `{row.get('primal_objective')}`, bound `{row.get('dual_bound')}`, "
            f"gap `{row.get('relative_gap')}`, evaluator-consistent `{row.get('objective_consistent')}`, "
            f"valid `{row.get('feasible') and row.get('architecture_valid')}`, certified `{row.get('proven_optimal')}`."
        )
    summary_lines.extend(
        [
            "",
            "## n=8 discrepancy",
            "",
            "The archived n=8 MILP incumbent has a native objective of 59.38346935. With readiness-based waiting recomputed without carrying stale prior-iteration waits, its independent evaluator objective is also 59.38346935 and high-priority tardiness is zero. The old 1,428.2715 evaluator value was caused by over-retained synchronization waits. The strict-zero-gap CBC run reached its 120-second time limit with a large remaining gap; this incumbent is not an optimality certificate.",
            "",
            "## Remaining model audit",
            "",
            "The existing formulation has lower-bound synchronization-wait variables but does not explicitly upper-link them to rendezvous indicators. Objective minimization makes unnecessary waiting weakly dominated for a single route, but the model's feasible timing assignments are broader than the paper's wait-only-at-rendezvous interpretation. This does not by itself prove an objective discrepancy; it is retained as a formulation note pending fixed-route feasibility checks.",
        ]
    )
    (OUTPUT / "exact_validation_summary.md").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    _write_exact_table(result_rows)
    return {"certified_optima": certified, "rows": len(result_rows), "solver_version": solver_version}


def _write_exact_table(rows: list[dict[str, Any]]) -> None:
    output_rows = []
    for row in rows:
        row = {**row, "solver_status": row.get("solver_log_status", row.get("solver_status", ""))}
        obj = "--" if row.get("primal_objective") is None else f"{float(row['primal_objective']):.4f}"
        eval_obj = "--" if row.get("independently_evaluated_objective") is None else f"{float(row['independently_evaluated_objective']):.4f}"
        nils_obj = "--" if row.get("nils_best_objective") is None else f"{float(row['nils_best_objective']):.4f}"
        runtime = "--" if row.get("solver_runtime_seconds") is None else f"{float(row['solver_runtime_seconds']):.1f}"
        solver_status = (
            "No feasible incumbent"
            if str(row.get("incumbent_available", "")).lower() == "false"
            else str(row.get("solver_status", ""))
        )
        output_rows.append(
            f"{row['size']} & {obj} & {eval_obj} & {nils_obj} & {solver_status} & {runtime} \\\\\n"
        )
    tex = (
        "\\begin{table}[htbp]\n\\centering\n"
        "\\caption{MILP incumbent audit on the seven saved small instances; no row is labeled optimal unless independently certified.}\n"
        "\\label{tab:exact_milp_audit}\n\\small\n"
        "\\begin{tabular}{rllllr}\n\\toprule\n"
        "$n$ & CBC incumbent & Evaluator objective & Best feasible NILS & CBC status & CPU (s) \\\\\n\\midrule\n"
        + "".join(output_rows)
        + "\\bottomrule\n\\end{tabular}\n"
        "\\begin{flushleft}\\footnotesize CBC incumbents are not proven optima unless a native optimal termination, matching primal/dual bound, feasible extracted solution, and independent objective reconciliation are all verified.\\end{flushleft}\n"
        "\\end{table}\n"
    )
    (OUTPUT / "exact_table5.tex").write_text(tex, encoding="utf-8")


def finalize_exact_calibration() -> dict[str, Any]:
    result_path = OUTPUT / "exact_results.csv"
    certificate_path = OUTPUT / "exact_certificates.csv"
    result_rows = _read_csv(result_path)
    certificates_by_id = {row["instance_id"]: row for row in _read_csv(certificate_path)}
    if len(result_rows) != len(EXACT_SIZES):
        raise RuntimeError(f"Expected {len(EXACT_SIZES)} exact records, found {len(result_rows)}")
    finalized: list[dict[str, Any]] = []
    certificates: list[dict[str, Any]] = []
    for row in result_rows:
        instance_id = str(row["instance_id"])
        size = int(row["size"])
        log_path = OUTPUT / "exact_solver_logs" / f"n{size}_cbc_zero_gap.log"
        log_result = _read_cbc_result(str(log_path))
        log_status = str(log_result["result"])
        incumbent_available = log_result.get("has_feasible_solution") is not False
        log_optimal = (
            "optimal solution found" in log_status.lower()
            and "within gap tolerance" not in log_status.lower()
        )
        primal = _optional_float(row.get("primal_objective"))
        if not incumbent_available:
            primal = None
            row["primal_objective"] = None
            row["independently_evaluated_objective"] = None
            row["objective_consistent"] = False
            row["feasible"] = False
            row["architecture_valid"] = False
            row["solver_error"] = (
                "Native CBC log reports no feasible solution; PuLP variable values were relaxation state, not an incumbent."
            )
        nils_objective = _optional_float(row.get("nils_best_objective"))
        logged_bound = log_result["best_bound"]
        dual_bound = (
            primal
            if log_optimal and primal is not None
            else float(logged_bound)
            if logged_bound is not None
            else None
        )
        bound_source = (
            "implied_by_native_cbc_optimal_termination"
            if log_optimal
            else "cbc_log_lower_bound"
            if logged_bound is not None
            else "not_reported"
        )
        final_logged_bound = None if log_optimal else logged_bound
        bound_consistent = bool(
            primal is not None
            and dual_bound is not None
            and dual_bound <= primal + max(1e-7, abs(primal) * 1e-8)
        )
        relative_gap = (
            max(0.0, (primal - dual_bound) / max(abs(primal), 1e-12))
            if bound_consistent and primal is not None and dual_bound is not None
            else None
        )
        bound_tight = bool(
            primal is not None
            and dual_bound is not None
            and bound_consistent
            and math.isclose(primal, dual_bound, rel_tol=1e-7, abs_tol=1e-6)
        )
        objective_consistent = str(row.get("objective_consistent", "")).lower() == "true"
        model_components_consistent = str(row.get("model_components_sum_consistent", "")).lower() == "true"
        feasible = str(row.get("feasible", "")).lower() == "true"
        architecture_valid = str(row.get("architecture_valid", "")).lower() == "true"
        nils_consistent = bool(
            primal is not None
            and nils_objective is not None
            and primal <= nils_objective + max(1e-6, abs(nils_objective) * 1e-8)
        )
        proven = bool(
            log_optimal
            and bound_tight
            and objective_consistent
            and model_components_consistent
            and feasible
            and architecture_valid
            and nils_consistent
        )
        row.update(
            {
                "solver_log_status": log_status,
                "incumbent_available": incumbent_available,
                "solver_version": "CBC 2.10.3",
                "dual_bound": dual_bound,
                "dual_bound_source": bound_source,
                "bound_consistent_with_primal": bound_consistent,
                "relative_gap": relative_gap,
                "proven_optimal": proven,
                "nils_gap_only_if_certified": (
                    nils_objective - primal
                    if proven and nils_objective is not None and primal is not None
                    else None
                ),
            }
        )
        certificate = certificates_by_id.get(instance_id, {}).copy()
        certificate.update(
            {
                "instance_id": instance_id,
                "size": size,
                "solver": "CBC",
                "solver_version": "CBC 2.10.3",
                "log_file": log_path.relative_to(REPO).as_posix(),
                "log_result": log_status,
                "log_incumbent": log_result["objective"],
                "log_lower_bound": final_logged_bound,
                "dual_bound_used": dual_bound,
                "dual_bound_source": bound_source,
                "bound_consistent_with_primal": bound_consistent,
                "bound_tight": bound_tight,
                "solver_api_primal": primal,
                "solver_api_bound": dual_bound,
                "solver_api_relative_gap": relative_gap,
                "evaluator_objective_matches": objective_consistent,
                "model_components_sum_matches": model_components_consistent,
                "feasibility_valid": feasible,
                "architecture_valid": architecture_valid,
                "best_feasible_nils_objective": nils_objective,
                "primal_not_above_feasible_nils": nils_consistent,
                "certified_optimum": proven,
            }
        )
        finalized.append(row)
        certificates.append(certificate)

    reconciliation_path = OUTPUT / "exact_objective_reconciliation.csv"
    reconciliation_rows = _read_csv(reconciliation_path)
    for row in reconciliation_rows:
        if int(row.get("size", -1)) == 9:
            row["raw_pulp_relaxation_objective"] = row.get("solver_objective", "")
            row["solver_objective"] = ""
            row["evaluator_objective"] = ""
            row["objective_delta"] = ""
            row["objective_consistent"] = "False"
            row["validator_pass"] = "False"
            row["cbc_feasible_incumbent"] = "False"
            row["interpretation"] = (
                "CBC reported no feasible incumbent; raw PuLP values are relaxation state and are not a solution objective."
            )
    _write_csv(reconciliation_path, reconciliation_rows)

    diagnostic_path = OUTPUT / "exact_constraint_diagnostics.csv"
    diagnostic_rows = _read_csv(diagnostic_path)
    for row in diagnostic_rows:
        if int(row.get("size", -1)) == 9:
            row["cbc_feasible_incumbent"] = "False"
            row["reported_candidate_is_incumbent"] = "False"
            row["native_solver_finding"] = "No feasible solution found before time limit"
    _write_csv(diagnostic_path, diagnostic_rows)

    _write_csv(result_path, finalized)
    _write_csv(certificate_path, certificates)
    _write_exact_table(finalized)
    certified_count = sum(str(row["proven_optimal"]).lower() == "true" for row in finalized)
    summary_lines = [
        "# Exact MILP audit summary",
        "",
        f"Status: {'PASS' if certified_count == len(EXACT_SIZES) else 'BLOCKED'} ({certified_count}/{len(EXACT_SIZES)} proven optima).",
        "Solver: CBC 2.10.3; zero relative-gap request; 120-second per-instance limit.",
        "Native CBC logs are retained. An absent lower-bound line is treated as equal to the incumbent only when CBC reports `Optimal solution found`.",
        "A relative gap is left blank when the logged lower bound exceeds the incumbent or a valid bound is unavailable.",
        "NILS comparisons use the corrected readiness-based schedule; the NILS optimality gap is reported only for certified cases.",
        "",
        "## Case status",
        "",
    ]
    for row in finalized:
        summary_lines.append(
            f"- n={row['size']}: CBC `{row.get('solver_log_status')}`, primal `{row.get('primal_objective')}`, "
            f"dual bound `{row.get('dual_bound')}` ({row.get('dual_bound_source')}), gap `{row.get('relative_gap')}`, "
            f"evaluator-consistent `{row.get('objective_consistent')}`, feasible `{row.get('feasible')}`, "
            f"architecture-valid `{row.get('architecture_valid')}`, certified `{row.get('proven_optimal')}`."
        )
    summary_lines.extend(
        [
            "",
            "## Material findings",
            "",
            "- n=3 is certified: CBC ended with native optimal status, the extracted assignment passes the final validators, the model and evaluator objectives agree, and the optimum matches the best feasible archived NILS start within tolerance.",
            "- n=4–6 and n=8 produced feasible, evaluator-consistent CBC incumbents, but stopped at the time limit with large nonzero gaps. Their incumbent objectives exceed archived feasible NILS objectives; this does not imply that the MILP optimum is worse.",
            "- n=7's CBC incumbent is invalid because it contains an unnecessary battery swap at rendezvous node 5 (drone 3). It is not an exact benchmark.",
            "- The archived n=8 objective discrepancy was entirely in priority-weighted tardiness: the MILP components were truck 37.4273779161, drone 21.9560914348, and tardiness 0; the prior evaluator added 1,368.8880485209 because stale readiness-based waiting produced 13.6888804852 minutes of high-priority tardiness at weight 100. Recomputing waits from readiness removes that stale delay and reproduces the archived MILP objective 59.3834693509. The fresh time-limited incumbent is 59.4184751613 and is not certified.",
            "- n=9 stopped at the time limit with no feasible CBC incumbent. The native log explicitly says `No feasible solution found`; PuLP had populated variables with relaxation values, which the prior adapter mistakenly exposed as a candidate. The corrected finalizer discards those values, retains the 5.195 lower bound, and reports no primal objective or gap.",
            "- The n=7 unnecessary swap reflects a model/validator convention gap: the MILP permits optional battery-restoration binaries, while the final validator rejects swaps that are not needed by battery state. Synchronization wait variables are also lower bounds without an explicit upper/equality link to exact readiness waiting. Fixed-route NILS-to-MILP compatibility was not run; do not claim complete exact-model reconciliation.",
        ]
    )
    (OUTPUT / "exact_validation_summary.md").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    return {"certified_optima": certified_count, "rows": len(finalized)}


def _optional_float(value: Any) -> float | None:
    if value in (None, "", "None", "nan", "NaN"):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _truthy(value: Any) -> bool:
    return value is True or value == 1 or str(value).strip().lower() in {"true", "1", "1.0"}


def _matches_numeric_flag(value: Any, expected: float) -> bool:
    observed = _optional_float(value)
    return observed is not None and observed == expected


def _annotate_ablation_initial_incumbents(
    rows: list[dict[str, Any]],
    m5_results: dict[str, dict[str, str]],
) -> dict[str, Any]:
    checks: list[bool] = []
    for row in rows:
        if row.get("method") not in {"M6", "M7"}:
            row["M5_matched_truck_only_objective"] = ""
            row["initial_backbone_objective_matches_M5"] = ""
            row["method_seed_matches_M5"] = ""
            row["initial_incumbent_matches_M5"] = ""
            continue
        reference = m5_results.get(str(row["instance_id"]), {})
        m5_objective = _optional_float(reference.get("matched_truck_only_objective"))
        shared_objective = _optional_float(row.get("shared_initial_backbone_objective"))
        objective_matches = bool(
            m5_objective is not None
            and shared_objective is not None
            and math.isclose(m5_objective, shared_objective, rel_tol=1e-12, abs_tol=1e-6)
        )
        route_hash_matches = (
            str(row.get("initial_backbone_routes_sha256", ""))
            == str(reference.get("initial_backbone_routes_sha256", ""))
            and bool(reference.get("initial_backbone_routes_sha256"))
        )
        m5_seed = _optional_float(reference.get("method_seed"))
        method_seed = _optional_float(row.get("method_seed"))
        seed_matches = m5_seed is not None and method_seed is not None and m5_seed == method_seed
        reused = _truthy(row.get("initial_backbone_reused"))
        incumbent_matches = objective_matches and route_hash_matches and seed_matches and reused
        row["M5_matched_truck_only_objective"] = m5_objective
        row["initial_backbone_objective_matches_M5"] = objective_matches
        row["method_seed_matches_M5"] = seed_matches
        row["initial_incumbent_matches_M5"] = incumbent_matches
        checks.append(incumbent_matches)
    return {
        "checks": len(checks),
        "passed": len(checks) == 8 and all(checks),
        "passed_checks": sum(checks),
        "expected_checks": 8,
    }


def finalize_saved_ablation_gate_audit() -> dict[str, Any]:
    diagnostic_path = OUTPUT / "ablation_gate_diagnostics.csv"
    rows = _read_csv(diagnostic_path)
    m5_results = {
        row["instance_id"]: row
        for row in _read_csv(FROZEN_OUTPUT / "tables/raw_720_method_results.csv")
        if row["method_name"] == "nils"
    }
    initial_checks = _annotate_ablation_initial_incumbents(rows, m5_results)
    _write_csv(diagnostic_path, rows)

    summary = json.loads((OUTPUT / "ablation_gate_summary.json").read_text(encoding="utf-8"))
    reference_summary = json.loads(
        (OUTPUT / "frozen_720_revalidation_summary.json").read_text(encoding="utf-8")
    )
    reference_changes = int(reference_summary.get("corrected_evaluator_changed_objectives", 0))
    invalid = [
        f"{row['method']} {row['instance_id']}"
        for row in rows
        if row.get("method") in {"M2", "M6", "M7"}
        and (not _truthy(row.get("full_feasible")) or not _truthy(row.get("architecture_valid")))
    ]
    consistency_issues = []
    for row in rows:
        method = row.get("method")
        if method not in {"M2", "M6", "M7"}:
            continue
        if not _truthy(row.get("same_initial_backbone_as_M5")):
            consistency_issues.append(f"{method} {row['instance_id']} backbone mismatch")
        if method == "M6" and (
            not _truthy(row.get("initial_backbone_reused"))
            or not _matches_numeric_flag(row.get("local_search_enabled"), 0.0)
            or not _matches_numeric_flag(row.get("perturbation_enabled"), 1.0)
        ):
            consistency_issues.append(f"M6 {row['instance_id']} ablation flag mismatch")
        if method == "M7" and (
            not _truthy(row.get("initial_backbone_reused"))
            or not _matches_numeric_flag(row.get("local_search_enabled"), 1.0)
            or not _matches_numeric_flag(row.get("perturbation_enabled"), 0.0)
        ):
            consistency_issues.append(f"M7 {row['instance_id']} ablation flag mismatch")
    reasons = []
    if invalid:
        reasons.append(f"invalid diagnostic result(s): {', '.join(invalid)}")
    if consistency_issues:
        reasons.append(f"diagnostic consistency issue(s): {', '.join(consistency_issues)}")
    if reference_changes:
        reasons.append(
            f"corrected-evaluator revalidation changes {reference_changes} frozen M3/M4/M5/M8 objective records"
        )
    if not initial_checks["passed"]:
        reasons.append(
            f"M6/M7 initial incumbent match failed ({initial_checks['passed_checks']}/{initial_checks['expected_checks']})"
        )
    summary["initial_incumbent_reproducibility"] = initial_checks
    summary["reference_integrity_passed"] = bool(
        reference_summary.get("frozen_artifact_audit", {}).get("passed") is True
        and int(reference_summary.get("previously_accepted_records_invalidated", 0)) == 0
        and reference_changes == 0
    )
    summary["gate_passed"] = bool(
        summary.get("gate_passed")
        and not invalid
        and not consistency_issues
        and not reference_changes
        and initial_checks["passed"]
    )
    summary["status"] = "PASS" if summary["gate_passed"] else "BLOCKED"
    summary["full_2160_runs_started"] = False
    summary["reason_full_run_not_started"] = "; ".join(reasons) if reasons else "diagnostic gate passed"
    _write_json(OUTPUT / "ablation_gate_summary.json", summary)
    return summary


def run_ablation_gate() -> dict[str, Any]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    config_data = yaml.safe_load((FROZEN_OUTPUT / "run_config.yaml").read_text(encoding="utf-8"))
    config = build_search_config(config_data)
    input_rows = _read_csv(FROZEN_OUTPUT / "input_manifest.csv")
    results = {row["instance_id"]: row for row in _read_csv(FROZEN_OUTPUT / "tables/raw_720_method_results.csv") if row["method_name"] == "nils"}
    candidates: list[dict[str, str]] = []
    for row in input_rows:
        result = results[row["instance_id"]]
        candidates.append({**row, **{f"m5_{key}": value for key, value in result.items()}})

    selected: list[dict[str, str]] = []

    def select(score_key: str, reverse: bool, predicate) -> None:
        pool = [row for row in candidates if predicate(row) and row["instance_id"] not in {item["instance_id"] for item in selected}]
        if pool:
            pool.sort(key=lambda row: float(row[score_key]), reverse=reverse)
            selected.append(pool[0])

    select("eligible_share_realized", False, lambda row: int(row["n"]) == 15)
    select("eligible_share_realized", True, lambda row: int(row["n"]) in {50, 75, 100})
    select(
        "m5_synchronization_waiting_minutes",
        True,
        lambda row: int(row["n"]) >= 50 and float(row["m5_u_sync_count"]) > 0,
    )
    select(
        "m5_synchronization_waiting_minutes",
        False,
        lambda row: int(row["n"]) in {15, 50, 75, 100} and float(row["m5_u_sync_count"]) > 0,
    )
    if len(selected) < 4 or len({int(row["n"]) for row in selected}) < 2:
        raise RuntimeError("Unable to construct the required stratified M2/M6/M7 gate sample")

    rows: list[dict[str, Any]] = []
    for input_row in selected:
        instance_id = input_row["instance_id"]
        instance_path = SOURCE_RAW / f"{instance_id}.json"
        if _sha256_file(instance_path) != input_row["instance_sha256"]:
            raise RuntimeError(f"Frozen instance changed for gate case {instance_id}")
        instance = load_instance_json(instance_path)
        backbone_path = REPO / input_row["backbone_file"]
        if _sha256_file(backbone_path) != input_row["backbone_file_sha256"]:
            raise RuntimeError(f"Frozen backbone changed for gate case {instance_id}")
        backbone_entry = json.loads(backbone_path.read_text(encoding="utf-8"))
        routes = {
            int(truck_id): [int(node) for node in route]
            for truck_id, route in backbone_entry["truck_routes"].items()
        }
        if _truck_routes_sha256(routes) != input_row["backbone_routes_sha256"]:
            raise RuntimeError(f"Frozen route hash changed for gate case {instance_id}")
        initial_solution = _fill_solution_from_routes(instance, routes)
        initial_objective, _ = evaluate_solution(instance, initial_solution)
        m5_backbone_hash = str(input_row.get("m5_initial_backbone_routes_sha256", ""))
        scenario = _scenario_from_instance(instance)
        scenario_config = _scenario_config(config, scenario)
        method_specs = [
            ("M2", "simple_drone", lambda: simple_drone_assignment_baseline(instance, scenario_config).solution),
            ("M6", "nils_no_local_search", lambda: nils_no_local_search_baseline(instance, scenario_config, initial_truck_routes=routes, seed=2026).solution),
            ("M7", "nils_no_perturbation", lambda: nils_no_perturbation_baseline(instance, scenario_config, initial_truck_routes=routes, seed=2026).solution),
        ]
        for method, method_name, runner in method_specs:
            started = time.perf_counter()
            error = ""
            solution = None
            try:
                solution = runner()
            except Exception as exception:
                error = repr(exception)
            wall_seconds = time.perf_counter() - started
            if solution is None:
                rows.append(
                    {
                        "instance_id": instance_id,
                        "size": instance.num_customers,
                        "scenario_class_id": input_row["scenario_id"],
                        "eligible_share_target": input_row["eligible_share_target"],
                        "m5_sync_wait_minutes": input_row["m5_synchronization_waiting_minutes"],
                        "method": method,
                        "method_name": method_name,
                        "error": error,
                        "wall_seconds": wall_seconds,
                        "shared_initial_backbone_objective": float(initial_objective),
                        "M5_initial_backbone_routes_sha256": m5_backbone_hash,
                        "same_initial_backbone_as_M5": bool(
                            method == "M2" or m5_backbone_hash == input_row["backbone_routes_sha256"]
                        ),
                        "full_feasible": False,
                        "architecture_valid": False,
                    }
                )
                continue
            objective, components = evaluate_solution(instance, solution)
            findings = validate_solution(instance, solution)
            feasibility_errors = [item for item in findings if item.severity == "ERROR"]
            architecture_errors = validate_drone_architecture(instance, solution, "M5")
            observed_components = solution.components or {}
            rows.append(
                {
                    "instance_id": instance_id,
                    "size": instance.num_customers,
                    "scenario_class_id": input_row["scenario_id"],
                    "eligible_share_target": input_row["eligible_share_target"],
                    "eligible_share_realized": input_row["eligible_share_realized"],
                    "m5_sync_wait_minutes": input_row["m5_synchronization_waiting_minutes"],
                    "method": method,
                    "method_name": method_name,
                    "status": solution.status,
                    "objective": float(objective),
                    "shared_initial_backbone_objective": float(initial_objective),
                    "M5_initial_backbone_routes_sha256": m5_backbone_hash,
                    "same_initial_backbone_as_M5": bool(
                        method == "M2" or m5_backbone_hash == input_row["backbone_routes_sha256"]
                    ),
                    "truck_cost": components.get("truck_cost"),
                    "drone_cost": components.get("drone_cost"),
                    "priority_tardiness_cost": components.get("tardiness_cost"),
                    "high_priority_tardiness": sum(
                        float(solution.tardiness.get(customer.node_id, 0.0))
                        for customer in instance.customers
                        if customer.priority == "high"
                    ),
                    "wall_seconds": wall_seconds,
                    "full_feasible": not feasibility_errors,
                    "architecture_valid": not architecture_errors,
                    "feasibility_errors": json.dumps([item.message for item in feasibility_errors]),
                    "architecture_errors": json.dumps(architecture_errors),
                    "initial_backbone_file_sha256": input_row["backbone_file_sha256"] if method in {"M6", "M7"} else "",
                    "initial_backbone_routes_sha256": input_row["backbone_routes_sha256"] if method in {"M6", "M7"} else "",
                    "initial_backbone_reused": observed_components.get("initial_backbone_reused", 0.0) >= 0.5 if method in {"M6", "M7"} else None,
                    "local_search_enabled": observed_components.get("local_search_enabled") if method in {"M6", "M7"} else None,
                    "perturbation_enabled": observed_components.get("perturbation_enabled") if method in {"M6", "M7"} else None,
                    "method_seed": 2026 if method in {"M6", "M7"} else None,
                    "error": error,
                }
            )

    initial_checks = _annotate_ablation_initial_incumbents(rows, results)
    _write_csv(OUTPUT / "ablation_gate_diagnostics.csv", rows)
    method_checks = {
        method: [row for row in rows if row["method"] == method]
        for method in ("M2", "M6", "M7")
    }
    passes = len(rows) == 12
    for method, method_rows in method_checks.items():
        if len(method_rows) != 4:
            passes = False
        if any(not row.get("full_feasible") or not row.get("architecture_valid") for row in method_rows):
            passes = False
    for row in rows:
        if not row.get("same_initial_backbone_as_M5"):
            passes = False
        if row["method"] in {"M6", "M7"} and not _truthy(row.get("initial_incumbent_matches_M5")):
            passes = False
        if row["method"] == "M6" and (
            row.get("initial_backbone_reused") is not True
            or row.get("local_search_enabled") not in (0, 0.0, False)
            or row.get("perturbation_enabled") not in (1, 1.0, True)
        ):
            passes = False
        if row["method"] == "M7" and (
            row.get("initial_backbone_reused") is not True
            or row.get("local_search_enabled") not in (1, 1.0, True)
            or row.get("perturbation_enabled") not in (0, 0.0, False)
        ):
            passes = False
    reference_summary_path = OUTPUT / "frozen_720_revalidation_summary.json"
    reference_summary = (
        _read_json(reference_summary_path)
        if reference_summary_path.is_file()
        else {}
    )
    reference_changes = int(reference_summary.get("corrected_evaluator_changed_objectives", 0))
    if reference_summary and (
        reference_summary.get("frozen_artifact_audit", {}).get("passed") is not True
        or int(reference_summary.get("previously_accepted_records_invalidated", 0)) > 0
        or reference_changes > 0
    ):
        passes = False
    reasons = []
    invalid_by_method = {
        method: [row["instance_id"] for row in method_rows if not row.get("full_feasible") or not row.get("architecture_valid")]
        for method, method_rows in method_checks.items()
    }
    for method, instance_ids in invalid_by_method.items():
        if instance_ids:
            reasons.append(f"{method} has invalid diagnostic result(s): {', '.join(instance_ids)}")
    if reference_summary and reference_changes:
        reasons.append(
            f"corrected-evaluator revalidation changes {reference_changes} frozen M3/M4/M5/M8 objective records"
        )
    if not initial_checks["passed"]:
        reasons.append(
            f"M6/M7 initial incumbent match failed ({initial_checks['passed_checks']}/{initial_checks['expected_checks']})"
        )
    if not reasons and not passes:
        reasons.append("one or more method-flag, common-backbone, or validation checks failed")
    summary = {
        "status": "PASS" if passes else "BLOCKED",
        "sample_instances": [row["instance_id"] for row in selected],
        "sample_sizes": sorted({int(row["n"]) for row in selected}),
        "eligibility_range": [
            min(float(row["eligible_share_realized"]) for row in selected),
            max(float(row["eligible_share_realized"]) for row in selected),
        ],
        "non_depot_rendezvous_opportunity": any(float(row["m5_u_sync_count"]) > 0 for row in selected),
        "runs_expected": 12,
        "runs_completed": len(rows),
        "method_counts": {method: len(method_rows) for method, method_rows in method_checks.items()},
        "initial_incumbent_reproducibility": initial_checks,
        "gate_passed": passes,
        "full_2160_runs_started": False,
        "reference_integrity_passed": bool(
            reference_summary
            and reference_summary.get("frozen_artifact_audit", {}).get("passed") is True
            and int(reference_summary.get("previously_accepted_records_invalidated", 0)) == 0
            and reference_changes == 0
        ),
        "reason_full_run_not_started": "; ".join(reasons) if reasons else "diagnostic gate passed",
    }
    _write_json(OUTPUT / "ablation_gate_summary.json", summary)
    return summary


def write_environment_manifest() -> dict[str, Any]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repository_head_at_audit_start": "ff95956caeb9efa2f8ce6f7123d407cce80a36c5",
        "branch": "codex/utdrp-latest",
        "frozen_execution_commit": "8abdc6b7d9f82a7dc630468e4f108e48f2246e84",
        "python": sys.version,
        "platform": platform.platform(),
        "pulp": pulp.__version__,
        "available_pulp_solvers": pulp.listSolvers(onlyAvailable=True),
        "cbc_executable": pulp.PULP_CBC_CMD().path,
        "frozen_output_sha256_manifest": str(FROZEN_OUTPUT / "artifact_sha256_manifest.csv"),
        "frozen_input_manifest": str(FROZEN_OUTPUT / "input_manifest.csv"),
        "legacy_small_manifest": str(SMALL_MANIFEST),
        "controlling_pdf": "C:/Users/L03128674/Downloads/Drone_paper (9).pdf",
        "controlling_pdf_sha256": "EF3AE96764DB89A44CA71EE4BF3579B846A04B366F265D9C2DE41FE13F1A3247",
    }
    _write_json(OUTPUT / "environment_manifest.json", manifest)
    return manifest


def write_hash_manifests() -> dict[str, int]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    source_paths: set[Path] = set()
    for root in (REPO / "src", REPO / "tests", REPO / "config"):
        if root.is_dir():
            source_paths.update(path for path in root.rglob("*") if path.is_file() and "__pycache__" not in path.parts)
    for name in ("pyproject.toml", "requirements.txt", "requirements-dev.txt", "setup.cfg", "pytest.ini"):
        path = REPO / name
        if path.is_file():
            source_paths.add(path)
    source_rows = [
        {
            "path": path.relative_to(REPO).as_posix(),
            "sha256": _sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        for path in sorted(source_paths)
    ]
    _write_csv(OUTPUT / "source_sha256_manifest.csv", source_rows)

    artifact_manifest = OUTPUT / "audit_artifact_sha256_manifest.csv"
    artifact_paths = sorted(
        path for path in OUTPUT.rglob("*")
        if path.is_file() and path != artifact_manifest and not path.name.endswith(".tmp")
    )
    artifact_rows = [
        {
            "path": path.relative_to(OUTPUT).as_posix(),
            "sha256": _sha256_file(path),
            "size_bytes": path.stat().st_size,
        }
        for path in artifact_paths
    ]
    _write_csv(artifact_manifest, artifact_rows)
    return {"source_files": len(source_rows), "audit_artifacts_hashed": len(artifact_rows)}


def clustered_priority_on_time_interval(replicates: int = 10_000) -> dict[str, Any]:
    rows = _read_csv(OUTPUT / "frozen_720_high_priority_delivery_detail.csv")
    m5_method = "nils"
    m8_method = "feasibility_aware_alns"
    paired: dict[tuple[str, str], dict[str, bool]] = {}
    for row in rows:
        if row["method"] not in {m5_method, m8_method}:
            continue
        key = (row["instance_id"], row["customer_id"])
        values = paired.setdefault(key, {})
        if row["method"] in values:
            raise RuntimeError(f"Duplicate high-priority customer record for {key} / {row['method']}")
        values[row["method"]] = row["on_time"].strip().lower() == "true"
    if not paired or any(set(values) != {m5_method, m8_method} for values in paired.values()):
        raise RuntimeError("M5 and M8 high-priority customer records are not fully paired")

    instance_ids = [row["instance_id"] for row in _read_csv(FROZEN_OUTPUT / "input_manifest.csv")]
    counts = {instance_id: [0, 0, 0] for instance_id in instance_ids}
    for (instance_id, _customer), values in paired.items():
        counts[instance_id][0] += int(values[m5_method])
        counts[instance_id][1] += int(values[m8_method])
        counts[instance_id][2] += 1

    denominator = sum(values[2] for values in counts.values())
    m5_on_time = sum(values[0] for values in counts.values())
    m8_on_time = sum(values[1] for values in counts.values())
    estimate = (m5_on_time - m8_on_time) / denominator
    rng = random.Random(20261117)
    bootstrap_differences: list[float] = []
    for _ in range(replicates):
        resampled_m5 = 0
        resampled_m8 = 0
        resampled_customers = 0
        for _ in instance_ids:
            instance_id = instance_ids[rng.randrange(len(instance_ids))]
            m5_count, m8_count, customer_count = counts[instance_id]
            resampled_m5 += m5_count
            resampled_m8 += m8_count
            resampled_customers += customer_count
        if resampled_customers:
            bootstrap_differences.append((resampled_m5 - resampled_m8) / resampled_customers)
    bootstrap_differences.sort()

    def percentile(probability: float) -> float:
        position = probability * (len(bootstrap_differences) - 1)
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return bootstrap_differences[lower]
        fraction = position - lower
        return bootstrap_differences[lower] * (1.0 - fraction) + bootstrap_differences[upper] * fraction

    result = {
        "comparison": "M5 NILS minus M8 ALNS high-priority on-time rate",
        "estimand": "pooled customer-level rate difference with instances resampled as paired clusters",
        "estimate_percentage_points": 100.0 * estimate,
        "ci_95_percentile_percentage_points": [100.0 * percentile(0.025), 100.0 * percentile(0.975)],
        "m5_on_time_customers": m5_on_time,
        "m8_on_time_customers": m8_on_time,
        "matched_high_priority_customers": denominator,
        "instance_clusters_resampled": len(instance_ids),
        "clusters_with_high_priority_customers": sum(values[2] > 0 for values in counts.values()),
        "bootstrap_replicates": len(bootstrap_differences),
        "random_seed": 20261117,
        "customer_records_treated_as_independent": False,
    }
    _write_json(OUTPUT / "high_priority_on_time_clustered_ci.json", result)
    return result


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--revalidate-frozen", action="store_true")
    parser.add_argument("--revalidate-references", action="store_true")
    parser.add_argument("--run-exact", action="store_true")
    parser.add_argument("--finalize-exact", action="store_true")
    parser.add_argument("--exact-time-limit", type=int, default=120)
    parser.add_argument("--run-ablation-gate", action="store_true")
    parser.add_argument("--write-manifests", action="store_true")
    parser.add_argument("--clustered-priority-ci", action="store_true")
    parser.add_argument("--finalize-ablation-gate-audit", action="store_true")
    args = parser.parse_args()
    write_environment_manifest()
    if args.revalidate_frozen:
        print(json.dumps(revalidate_frozen_720(), indent=2))
    if args.revalidate_references:
        print(json.dumps(revalidate_references(), indent=2))
    if args.run_exact:
        print(json.dumps(run_exact_calibration(args.exact_time_limit), indent=2))
        print(json.dumps(finalize_exact_calibration(), indent=2))
    if args.finalize_exact:
        print(json.dumps(finalize_exact_calibration(), indent=2))
    if args.run_ablation_gate:
        print(json.dumps(run_ablation_gate(), indent=2))
    if args.write_manifests:
        print(json.dumps(write_hash_manifests(), indent=2))
    if args.clustered_priority_ci:
        print(json.dumps(clustered_priority_on_time_interval(), indent=2))
    if args.finalize_ablation_gate_audit:
        print(json.dumps(finalize_saved_ablation_gate_audit(), indent=2))
    if not (args.revalidate_frozen or args.revalidate_references or args.run_exact or args.finalize_exact or args.run_ablation_gate or args.write_manifests or args.clustered_priority_ci or args.finalize_ablation_gate_audit):
        parser.error("Select at least one audit action")


if __name__ == "__main__":
    main()
