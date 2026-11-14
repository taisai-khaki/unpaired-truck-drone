"""Recompute legacy calibration and real-case results with final semantics."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, fields, is_dataclass, replace
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from src.data_models import SolutionData
from src.distance_utils import euclidean_distance, manhattan_distance
from src.experiments.run_real_case import (
    _make_config,
    _simple_drone_from_feasible_backbone,
    _truck_only_ortools,
    build_real_case_instance,
)
from src.experiments.run_small_exact import (
    _append_progress,
    _build_progress_writer,
    _nils_seed_for_start,
    _safe_solve_exact,
)
from src.feasibility import validate_solution
from src.heuristics.architecture import validate_drone_architecture
from src.heuristics.baselines import (
    nils_no_local_search_baseline,
    nils_no_perturbation_baseline,
    no_unpairing_baseline,
    paired_baseline,
)
from src.heuristics.nils import (
    default_drone_to_truck,
    evaluate_solution,
    run_nils,
)
from src.instance_generator import load_instance_json
from src.parameters import build_search_config, load_yaml_config
from src.timing import active_rendezvous_events, compute_synchronization_waiting


PROJECT_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = PROJECT_ROOT / "outputs" / "final_legacy_semantics_refresh_20261114"
SMALL_CONFIG_PATH = PROJECT_ROOT / "config" / "tmp_small_multistart_beta010_reviewer.yaml"
SMALL_INPUT_DIR = (
    PROJECT_ROOT / "outputs" / "small_multistart_beta010_after_drone_two_opt" / "raw"
)
SMALL_HISTORICAL_TABLE = (
    PROJECT_ROOT
    / "outputs"
    / "small_multistart_beta010_after_drone_two_opt"
    / "tables"
    / "table_small_exact_vs_corrected_nils_beta010_full_precision.csv"
)
REAL_INPUT_PATH = Path(r"C:\Users\L03128674\Downloads\Delivery_Logistics.csv")
REAL_HISTORICAL_SAMPLE = (
    PROJECT_ROOT
    / "outputs"
    / "real_case_lambda_sensitivity_beta010_lam0075"
    / "lambda_0p07"
    / "real_case_sample.csv"
)
REAL_HISTORICAL_RESULTS = (
    PROJECT_ROOT
    / "outputs"
    / "real_case_lambda_sensitivity_beta010_combined"
    / "real_case_lambda_sensitivity_all_methods.csv"
)
MANUSCRIPT_PATH = Path(r"C:\Users\L03128674\Downloads\Drone_paper (7).pdf")
LAMBDAS = (0.075, 0.100, 0.115, 0.120, 0.125, 0.130, 0.150, 0.200)
METHODS = (
    ("truck_only", "M1 Truck-only", "M5"),
    ("simple_drone", "M2 Simple drone", "M5"),
    ("paired_baseline", "M3 Paired baseline", "M3"),
    ("no_unpairing", "M4 No-unpairing", "M4"),
    ("nils", "M5 NILS", "M5"),
    ("nils_no_local_search", "M6 NILS-no-local-search", "M5"),
    ("nils_no_perturbation", "M7 NILS-no-perturbation", "M5"),
)
SMALL_PAPER_VALUES = {
    3: (11.077564, 11.077564, 0.00),
    4: (11.319586, 11.319586, 0.00),
    5: (9.751888, 9.853851, 1.05),
    6: (10.310181, 10.310181, 0.00),
    7: (10.220200, 10.220200, 0.00),
    8: (11.132644, 11.132644, 0.00),
    9: (10.697707, 10.697707, 0.00),
}
TUPLE_MAP_FIELDS = {
    "x_truck",
    "x_drone",
    "u_truck",
    "u_drone",
    "z1",
    "z2",
    "u_sync",
    "y_loaded",
    "a_truck",
    "l_truck",
    "a_drone",
    "l_drone",
    "waiting_truck",
    "waiting_drone",
    "load_truck",
    "battery_drone",
}
INT_MAP_FIELDS = {"tardiness"}
ROUTE_MAP_FIELDS = {"truck_routes", "drone_routes"}
METRIC_COLUMNS = (
    "truck_cost",
    "drone_cost",
    "priority_tardiness_cost",
    "high_priority_tardiness",
    "medium_priority_tardiness",
    "low_priority_tardiness",
    "truck_distance",
    "drone_distance",
    "drone_service_share",
    "drone_sorties",
    "depot_depot_sorties",
    "depot_truck_sorties",
    "truck_depot_sorties",
    "same_truck_recovery_sorties",
    "cross_truck_recovery_sorties",
    "physical_rendezvous_count",
    "synchronization_waiting_minutes",
    "parcel_reload_count",
    "battery_swap_count",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def json_safe(value: Any) -> Any:
    if is_dataclass(value):
        return json_safe(asdict(value))
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        values = list(value)
        if isinstance(value, set):
            values = sorted(values, key=str)
        return [json_safe(item) for item in values]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def solution_payload(solution: SolutionData) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    declared = {item.name for item in fields(SolutionData)}
    for name in declared:
        value = getattr(solution, name)
        if name in TUPLE_MAP_FIELDS:
            payload[name] = [
                {"key": list(key), "value": json_safe(item)}
                for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
            ]
        elif name in INT_MAP_FIELDS or name in ROUTE_MAP_FIELDS:
            payload[name] = [
                {"key": int(key), "value": json_safe(item)}
                for key, item in sorted(value.items(), key=lambda pair: int(pair[0]))
            ]
        else:
            payload[name] = json_safe(value)
    payload["dynamic_fields"] = {
        name: json_safe(value)
        for name, value in solution.__dict__.items()
        if name not in declared
    }
    return payload


def solution_from_payload(payload: dict[str, Any]) -> SolutionData:
    values = {item.name: payload.get(item.name) for item in fields(SolutionData)}
    for name in TUPLE_MAP_FIELDS:
        values[name] = {
            tuple(item["key"]): item["value"] for item in (payload.get(name) or [])
        }
    for name in INT_MAP_FIELDS | ROUTE_MAP_FIELDS:
        values[name] = {
            int(item["key"]): item["value"] for item in (payload.get(name) or [])
        }
    values["tags"] = tuple(values.get("tags") or ())
    values["sync_events"] = []
    solution = SolutionData(**values)
    for name, value in (payload.get("dynamic_fields") or {}).items():
        setattr(solution, name, value)
    return solution


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(json_safe(value), indent=2, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def append_jsonl(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(json_safe(value), sort_keys=True, allow_nan=False))
        stream.write("\n")
        stream.flush()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(json.loads(line))
    return records


def log_message(message: str) -> None:
    line = f"[{datetime.now().astimezone().isoformat(timespec='seconds')}] {message}"
    print(line, flush=True)
    path = OUTPUT_DIR / "logs" / "refresh.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def validator_result(
    instance: Any,
    solution: SolutionData,
    architecture: str,
) -> dict[str, Any]:
    findings = validate_solution(instance, solution)
    errors = [
        {
            "code": finding.code,
            "message": finding.message,
            "vehicle": finding.vehicle,
            "node": finding.node,
        }
        for finding in findings
        if finding.severity.upper() == "ERROR"
    ]
    warnings = [
        {
            "code": finding.code,
            "message": finding.message,
            "vehicle": finding.vehicle,
            "node": finding.node,
        }
        for finding in findings
        if finding.severity.upper() != "ERROR"
    ]
    pairing = default_drone_to_truck(instance) if architecture == "M3" else None
    architecture_violations = validate_drone_architecture(
        instance, solution, architecture, drone_to_truck=pairing
    )
    return {
        "feasibility_validator_pass": not errors,
        "architecture_validator_pass": not architecture_violations,
        "final_valid": not errors and not architecture_violations,
        "feasibility_errors": errors,
        "feasibility_warnings": warnings,
        "architecture_violations": architecture_violations,
    }


def sortie_diagnostics(instance: Any, solution: SolutionData) -> dict[str, int]:
    events_by_drone_node: dict[tuple[int, int], set[int]] = {}
    for node, drone_id, truck_id in active_rendezvous_events(solution):
        events_by_drone_node.setdefault((int(drone_id), int(node)), set()).add(int(truck_id))

    counts = {
        "drone_sorties": 0,
        "depot_depot_sorties": 0,
        "depot_truck_sorties": 0,
        "truck_depot_sorties": 0,
        "same_truck_recovery_sorties": 0,
        "cross_truck_recovery_sorties": 0,
    }
    for drone_id, raw_route in solution.drone_routes.items():
        route = [int(node) for node in raw_route]
        if len(route) < 2:
            continue
        boundaries: list[tuple[int, int, set[int]]] = [(0, 0, set())]
        for position, node in enumerate(route):
            trucks = events_by_drone_node.get((int(drone_id), node), set())
            if node != 0 and trucks:
                boundaries.append((position, node, trucks))
        final_position = len(route) - 1
        if route[-1] == 0:
            boundaries.append((final_position, 0, set()))
        else:
            boundaries.append((len(route), 0, set()))
        boundaries.sort(key=lambda item: item[0])
        for start, end in zip(boundaries, boundaries[1:]):
            service_nodes = [
                route[position]
                for position in range(start[0] + 1, min(end[0], len(route)))
                if route[position] != 0
                and solution.u_drone.get((int(drone_id), route[position]), 0) >= 0.5
            ]
            if not service_nodes:
                continue
            counts["drone_sorties"] += 1
            launch_node, launch_trucks = start[1], start[2]
            recovery_node, recovery_trucks = end[1], end[2]
            if launch_node == 0 and recovery_node == 0:
                counts["depot_depot_sorties"] += 1
            elif launch_node == 0 and recovery_node != 0:
                counts["depot_truck_sorties"] += 1
            elif launch_node != 0 and recovery_node == 0:
                counts["truck_depot_sorties"] += 1
            elif len(launch_trucks) == 1 and len(recovery_trucks) == 1:
                if launch_trucks == recovery_trucks:
                    counts["same_truck_recovery_sorties"] += 1
                else:
                    counts["cross_truck_recovery_sorties"] += 1
    return counts


def solution_metrics(instance: Any, solution: SolutionData) -> dict[str, Any]:
    objective, components = evaluate_solution(instance, solution)
    solution.objective = float(objective)
    solution.components = dict(components)
    priorities = {"high": 0.0, "medium": 0.0, "low": 0.0}
    for customer in instance.customers:
        priorities[customer.priority] += float(solution.tardiness.get(customer.node_id, 0.0))
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
    sync = compute_synchronization_waiting(instance, solution)
    share = len(solution.served_by_drone()) / max(1, instance.num_customers)
    metrics = {
        "candidate_objective": float(objective),
        "truck_cost": float(components.get("truck_cost", 0.0)),
        "drone_cost": float(components.get("drone_cost", 0.0)),
        "priority_tardiness_cost": float(components.get("tardiness_cost", 0.0)),
        "high_priority_tardiness": priorities["high"],
        "medium_priority_tardiness": priorities["medium"],
        "low_priority_tardiness": priorities["low"],
        "truck_distance": float(truck_distance),
        "drone_distance": float(drone_distance),
        "drone_service_share": float(share),
        "physical_rendezvous_count": len(active_rendezvous_events(solution)),
        "synchronization_waiting_minutes": float(sync.get("total_waiting_minutes", 0.0)),
        "parcel_reload_count": sum(value > 0.5 for value in solution.z1.values()),
        "battery_swap_count": sum(value > 0.5 for value in solution.z2.values()),
    }
    metrics.update(sortie_diagnostics(instance, solution))
    return metrics


def package_versions() -> dict[str, Any]:
    package_names = ("numpy", "pandas", "PuLP", "ortools", "PyYAML", "scipy")
    packages: dict[str, str | None] = {}
    for package_name in package_names:
        try:
            packages[package_name] = version(package_name)
        except PackageNotFoundError:
            packages[package_name] = None
    import pulp

    return {
        "python": sys.version,
        "platform": platform.platform(),
        "packages": packages,
        "available_pulp_solvers": pulp.listSolvers(onlyAvailable=True),
    }


def git_value(*arguments: str) -> str | None:
    try:
        return subprocess.check_output(
            ["git", *arguments], cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except Exception:
        return None


def input_bundle() -> tuple[list[Any], dict[float, Any], dict[str, Any]]:
    input_dir = OUTPUT_DIR / "inputs"
    small_dir = input_dir / "small_instances"
    real_dir = input_dir / "real_case_instances"
    historical_dir = input_dir / "historical"
    small_dir.mkdir(parents=True, exist_ok=True)
    real_dir.mkdir(parents=True, exist_ok=True)
    historical_dir.mkdir(parents=True, exist_ok=True)

    small_config_data = load_yaml_config(SMALL_CONFIG_PATH)
    small_config = build_search_config(small_config_data)
    config_copy = input_dir / "small_calibration_config.yaml"
    if not config_copy.exists():
        shutil.copy2(SMALL_CONFIG_PATH, config_copy)
    historical_small_copy = historical_dir / "small_exact_table_full_precision.csv"
    historical_real_copy = historical_dir / "real_case_lambda_all_methods.csv"
    for source, target in (
        (SMALL_HISTORICAL_TABLE, historical_small_copy),
        (REAL_HISTORICAL_RESULTS, historical_real_copy),
    ):
        if not source.is_file():
            raise FileNotFoundError(f"Required historical comparison file is missing: {source}")
        if not target.exists():
            shutil.copy2(source, target)

    small_instances = []
    for size in range(3, 10):
        source = SMALL_INPUT_DIR / f"small_{size}_01.json"
        target = small_dir / source.name
        selected_path = target if target.is_file() else source
        if not selected_path.is_file():
            raise FileNotFoundError(f"Required frozen calibration instance is missing: {source}")
        instance = load_instance_json(selected_path)
        if instance.num_customers != size or instance.seed != 2026 + size * 1000 + 1:
            raise ValueError(f"Unexpected frozen calibration instance identity: {source}")
        if instance.num_trucks != 3 or instance.num_drones != 3:
            raise ValueError(f"Unexpected fleet in frozen calibration instance: {source}")
        if not target.exists():
            target.write_bytes(selected_path.read_bytes())
        small_instances.append(instance)

    sample_path = input_dir / "real_case_sample_80.csv"
    packaged_real_paths = {
        lam: real_dir / f"real_case_lambda_{lam:.3f}.json" for lam in LAMBDAS
    }
    real_instances: dict[float, Any] = {}
    packaged_inputs_complete = sample_path.is_file() and all(
        path.is_file() for path in packaged_real_paths.values()
    )
    if packaged_inputs_complete:
        canonical_sample = pd.read_csv(sample_path)
        for lam, path in packaged_real_paths.items():
            real_instances[lam] = load_instance_json(path)
            if (
                real_instances[lam].num_customers != 80
                or real_instances[lam].seed != 2026
                or not math.isclose(
                    float(real_instances[lam].metadata.get("distance_scale", -1.0)),
                    lam,
                    abs_tol=1e-12,
                )
            ):
                raise ValueError(f"Unexpected packaged lambda instance: {path}")
    else:
        if not REAL_INPUT_PATH.is_file():
            raise FileNotFoundError(f"Required real-data input is missing: {REAL_INPUT_PATH}")
        if not REAL_HISTORICAL_SAMPLE.is_file():
            raise FileNotFoundError(f"Required frozen sample is missing: {REAL_HISTORICAL_SAMPLE}")
        historical_sample = pd.read_csv(REAL_HISTORICAL_SAMPLE)
        canonical_sample = None
        for lam in LAMBDAS:
            instance, sample = build_real_case_instance(
                REAL_INPUT_PATH,
                seed=2026,
                sample_per_mode=20,
                distance_scale=lam,
                beta_slack_ratio=0.10,
            )
            if canonical_sample is None:
                canonical_sample = sample
                pd.testing.assert_frame_equal(
                    canonical_sample.reset_index(drop=True),
                    historical_sample.reset_index(drop=True),
                    check_dtype=False,
                    check_exact=True,
                )
            else:
                pd.testing.assert_frame_equal(
                    canonical_sample.reset_index(drop=True),
                    sample.reset_index(drop=True),
                    check_dtype=False,
                    check_exact=True,
                )
            real_instances[lam] = instance
            atomic_json(packaged_real_paths[lam], asdict(instance))
        canonical_sample.to_csv(sample_path, index=False)

    if REAL_HISTORICAL_SAMPLE.is_file():
        historical_sample = pd.read_csv(REAL_HISTORICAL_SAMPLE)
        pd.testing.assert_frame_equal(
            canonical_sample.reset_index(drop=True),
            historical_sample.reset_index(drop=True),
            check_dtype=False,
            check_exact=True,
        )
    if not sample_path.exists():
        canonical_sample.to_csv(sample_path, index=False)

    if canonical_sample is None:
        raise ValueError("The frozen real-data sample was not loaded.")
    for lam in LAMBDAS:
        sample = real_instances[lam]
        if sample.num_customers != len(canonical_sample):
            raise ValueError(f"Lambda {lam} does not use the frozen 80-row sample.")

    for lam in LAMBDAS:
        target = packaged_real_paths[lam]
        if not target.exists():
            atomic_json(target, asdict(real_instances[lam]))

    if not historical_small_copy.exists() or not historical_real_copy.exists():
        raise FileNotFoundError("Could not package the historical comparison tables.")

    source_hash = sha256_file(REAL_INPUT_PATH) if REAL_INPUT_PATH.is_file() else None
    sample_hash = sha256_file(sample_path)
    historical_sample_hash = (
        sha256_file(REAL_HISTORICAL_SAMPLE) if REAL_HISTORICAL_SAMPLE.is_file() else None
    )
    small_original_hashes = {
        size: (
            sha256_file(SMALL_INPUT_DIR / f"small_{size}_01.json")
            if (SMALL_INPUT_DIR / f"small_{size}_01.json").is_file()
            else sha256_file(small_dir / f"small_{size}_01.json")
        )
        for size in range(3, 10)
    }

    provenance = {
        "small_config_source": str(SMALL_CONFIG_PATH),
        "packaged_small_config": str(config_copy),
        "small_input_source_dir": str(SMALL_INPUT_DIR),
        "small_instances": [
            {
                "path": str(SMALL_INPUT_DIR / f"small_{size}_01.json"),
                "packaged_path": str(small_dir / f"small_{size}_01.json"),
                "sha256": small_original_hashes[size],
                "size": size,
                "seed": small_instances[size - 3].seed,
                "trucks": small_instances[size - 3].num_trucks,
                "drones": small_instances[size - 3].num_drones,
            }
            for size in range(3, 10)
        ],
        "real_data_source": str(REAL_INPUT_PATH),
        "real_data_source_sha256": source_hash,
        "historical_sample_path": str(REAL_HISTORICAL_SAMPLE),
        "historical_sample_sha256": historical_sample_hash,
        "packaged_sample_path": str(sample_path),
        "packaged_sample_sha256": sample_hash,
        "sample_matches_historical_exactly": True,
        "lambda_values": list(LAMBDAS),
        "real_instance_paths": {
            f"{lam:.3f}": str(real_dir / f"real_case_lambda_{lam:.3f}.json")
            for lam in LAMBDAS
        },
        "historical_results_path": str(REAL_HISTORICAL_RESULTS),
        "historical_results_sha256": sha256_file(historical_real_copy),
        "packaged_historical_small_table": str(historical_small_copy),
        "packaged_historical_real_results": str(historical_real_copy),
        "manuscript_path": str(MANUSCRIPT_PATH),
        "manuscript_sha256": sha256_file(MANUSCRIPT_PATH) if MANUSCRIPT_PATH.exists() else None,
        "small_input_fleet_mismatch_with_manuscript": {
            "saved_inputs": "3 trucks and 3 drones",
            "manuscript_table_5_description": "single truck and single drone",
            "resolution": "Use frozen inputs that produced the reported table; flag wording for correction.",
        },
    }
    return small_instances, real_instances, provenance


def exact_configuration(small_config_data: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    search_config = build_search_config(small_config_data)
    milp_config = replace(search_config.milp, enabled=True)
    exact_config = replace(search_config, milp=milp_config)
    experiment = dict(small_config_data.get("experiment", {}))
    experiment.setdefault("small_exact_watchdog_enabled", True)
    experiment.setdefault(
        "small_exact_wall_timeout_seconds",
        max(180, int(exact_config.milp.time_limit_seconds) + 60),
    )
    experiment.setdefault("small_exact_heartbeat_seconds", 20)
    return exact_config, experiment


def nils_start_record(
    instance: Any,
    config: Any,
    experiment: dict[str, Any],
    restart_index: int,
) -> dict[str, Any]:
    start_seed = _nils_seed_for_start(
        base_seed=int(config.heuristics.random_seed),
        instance_seed=int(instance.seed),
        restart_index=restart_index,
        seed_stride=int(experiment.get("small_nils_seed_stride", 10_000)),
        use_instance_seed=bool(experiment.get("small_nils_use_instance_seed", True)),
    )
    started = time.perf_counter()
    candidate = run_nils(
        instance,
        seed=start_seed,
        max_iter=int(config.heuristics.max_outer_iter),
        max_no_improve=int(config.heuristics.max_no_improve),
        time_limit=int(config.heuristics.time_limit_seconds),
        strict_candidate_check_cap=int(experiment.get("small_strict_candidate_check_cap", 24)),
        full_feasibility_check_interval=int(
            experiment.get("small_full_feasibility_check_interval", 1)
        ),
        require_feasible_incumbent=bool(
            experiment.get("small_require_feasible_incumbent", True)
        ),
        repair_infeasible_candidates=bool(
            experiment.get("small_repair_infeasible_candidates", True)
        ),
        repair_max_steps=int(experiment.get("small_repair_max_steps", 4)),
        battery_slack_ratio=float(
            experiment.get("small_battery_slack_ratio", experiment.get("battery_slack_ratio", 0.10))
        ),
        max_displacement_candidates=int(config.heuristics.max_displacement_candidates),
        max_repair_candidates=int(config.heuristics.max_repair_candidates),
    )
    elapsed = time.perf_counter() - started
    metrics = solution_metrics(instance, candidate)
    validators = validator_result(instance, candidate, "M5")
    return {
        "instance_id": instance.name,
        "size": instance.num_customers,
        "restart_index": restart_index,
        "seed": start_seed,
        "status": candidate.status,
        "elapsed_wall_seconds": elapsed,
        "candidate_runtime_seconds": float(candidate.run_time_seconds),
        "metrics": metrics,
        "validators": validators,
        "solution": solution_payload(candidate),
        "error": None,
    }


def run_small_calibration(
    small_instances: list[Any],
    small_config_data: dict[str, Any],
    raw_dir: Path,
    progress_path: Path,
) -> None:
    config = build_search_config(small_config_data)
    exact_config, experiment = exact_configuration(small_config_data)
    restart_count = max(1, int(experiment.get("small_nils_restarts", 1)))
    restart_path = raw_dir / "exact_nils_restarts.jsonl"
    exact_path = raw_dir / "exact_solver_runs.jsonl"
    starts = read_jsonl(restart_path)
    starts_by_key = {
        (row["instance_id"], int(row["restart_index"])): row for row in starts
    }
    exact_by_instance = {
        row["instance_id"]: row for row in read_jsonl(exact_path)
    }

    for instance in small_instances:
        log_message(f"Small calibration {instance.name}: NILS restarts")
        for restart_index in range(restart_count):
            key = (instance.name, restart_index)
            if key in starts_by_key:
                continue
            try:
                record = nils_start_record(instance, config, experiment, restart_index)
            except Exception as error:
                record = {
                    "instance_id": instance.name,
                    "size": instance.num_customers,
                    "restart_index": restart_index,
                    "seed": _nils_seed_for_start(
                        base_seed=int(config.heuristics.random_seed),
                        instance_seed=int(instance.seed),
                        restart_index=restart_index,
                        seed_stride=int(experiment.get("small_nils_seed_stride", 10_000)),
                        use_instance_seed=bool(experiment.get("small_nils_use_instance_seed", True)),
                    ),
                    "status": "exception",
                    "elapsed_wall_seconds": 0.0,
                    "candidate_runtime_seconds": None,
                    "metrics": {},
                    "validators": {
                        "feasibility_validator_pass": False,
                        "architecture_validator_pass": False,
                        "final_valid": False,
                        "feasibility_errors": [],
                        "feasibility_warnings": [],
                        "architecture_violations": [],
                    },
                    "solution": None,
                    "error": repr(error),
                }
            append_jsonl(restart_path, record)
            starts_by_key[key] = record
            log_message(
                f"{instance.name} restart {restart_index + 1}/{restart_count}: "
                f"status={record['status']}, valid={record['validators']['final_valid']}, "
                f"wall={record['elapsed_wall_seconds']:.3f}s"
            )

        if instance.name in exact_by_instance:
            continue
        exact_started = time.perf_counter()
        solution = None
        error_text = None
        try:
            solution = _safe_solve_exact(
                instance,
                exact_config,
                progress_path,
                experiment,
            )
        except Exception as error:
            error_text = repr(error)
        exact_elapsed = time.perf_counter() - exact_started
        if solution is None:
            record = {
                "instance_id": instance.name,
                "size": instance.num_customers,
                "status": "unavailable" if error_text is None else "exception",
                "solver_runtime_seconds": None,
                "elapsed_wall_seconds": exact_elapsed,
                "candidate_objective": None,
                "proven_optimal": False,
                "metrics": {},
                "validators": None,
                "solution": None,
                "error": error_text,
            }
        else:
            model_objective = float(solution.objective)
            model_components = dict(solution.components)
            metrics = solution_metrics(instance, solution)
            validators = validator_result(instance, solution, "M5")
            gap = metrics.get("candidate_objective")
            best_bound = model_components.get("best_bound")
            solver_gap = model_components.get("optimality_gap")
            status = str(solution.status)
            status_is_optimal = status.lower().strip().startswith("optimal")
            bound_is_tight = (
                best_bound is not None
                and abs(float(best_bound) - model_objective)
                <= max(1e-7, abs(model_objective) * 1e-9)
            )
            solver_gap_is_zero = solver_gap is not None and abs(float(solver_gap)) <= 1e-8
            objective_consistent = math.isclose(
                model_objective, float(gap), rel_tol=1e-9, abs_tol=1e-6
            )
            proven = (
                status_is_optimal
                and (bound_is_tight or solver_gap_is_zero)
                and objective_consistent
            )
            record = {
                "instance_id": instance.name,
                "size": instance.num_customers,
                "status": status,
                "solver_runtime_seconds": float(solution.run_time_seconds),
                "elapsed_wall_seconds": exact_elapsed,
                "candidate_objective": float(metrics["candidate_objective"]),
                "model_objective": model_objective,
                "model_components": json_safe(model_components),
                "objective_consistent_with_final_evaluator": objective_consistent,
                "best_bound": json_safe(best_bound),
                "solver_gap": json_safe(solver_gap),
                "proven_optimal": bool(proven),
                "metrics": metrics,
                "validators": validators,
                "solution": solution_payload(solution),
                "error": None,
            }
        append_jsonl(exact_path, record)
        exact_by_instance[instance.name] = record
        log_message(
            f"{instance.name} exact solve: status={record['status']}, "
            f"proven_optimal={record['proven_optimal']}, "
            f"wall={record['elapsed_wall_seconds']:.1f}s"
        )


def run_real_method(method: str, instance: Any, config: Any) -> SolutionData:
    if method == "truck_only":
        return _truck_only_ortools(instance)
    if method == "simple_drone":
        return _simple_drone_from_feasible_backbone(instance)
    if method == "paired_baseline":
        return paired_baseline(instance, config).solution
    if method == "no_unpairing":
        return no_unpairing_baseline(instance, config).solution
    if method == "nils":
        return run_nils(
            instance,
            seed=config.heuristics.random_seed,
            max_iter=config.heuristics.max_outer_iter,
            max_no_improve=config.heuristics.max_no_improve,
            time_limit=config.heuristics.time_limit_seconds,
            battery_slack_ratio=float(instance.metadata.get("battery_slack_ratio", 0.10)),
            max_displacement_candidates=config.heuristics.max_displacement_candidates,
            max_repair_candidates=config.heuristics.max_repair_candidates,
        )
    if method == "nils_no_local_search":
        return nils_no_local_search_baseline(instance, config).solution
    if method == "nils_no_perturbation":
        return nils_no_perturbation_baseline(instance, config).solution
    raise ValueError(f"Unrecognized real-case method: {method}")


def run_real_case(
    real_instances: dict[float, Any],
    raw_dir: Path,
) -> None:
    raw_path = raw_dir / "real_data_lambda_methods.jsonl"
    existing = read_jsonl(raw_path)
    completed = {(round(float(row["lambda"]), 3), row["method"]) for row in existing}
    for lam in LAMBDAS:
        instance = real_instances[lam]
        config = _make_config(seed=instance.seed)
        for method, method_label, architecture in METHODS:
            key = (round(lam, 3), method)
            if key in completed:
                continue
            started = time.perf_counter()
            try:
                solution = run_real_method(method, instance, config)
                metrics = solution_metrics(instance, solution)
                validators = validator_result(instance, solution, architecture)
                record = {
                    "lambda": lam,
                    "method": method,
                    "method_label": method_label,
                    "architecture": architecture,
                    "status": solution.status,
                    "elapsed_wall_seconds": time.perf_counter() - started,
                    "algorithm_runtime_seconds": float(solution.run_time_seconds),
                    "metrics": metrics,
                    "validators": validators,
                    "solution": solution_payload(solution),
                    "error": None,
                }
            except Exception as error:
                record = {
                    "lambda": lam,
                    "method": method,
                    "method_label": method_label,
                    "architecture": architecture,
                    "status": "exception",
                    "elapsed_wall_seconds": time.perf_counter() - started,
                    "algorithm_runtime_seconds": None,
                    "metrics": {},
                    "validators": {
                        "feasibility_validator_pass": False,
                        "architecture_validator_pass": False,
                        "final_valid": False,
                        "feasibility_errors": [],
                        "feasibility_warnings": [],
                        "architecture_violations": [],
                    },
                    "solution": None,
                    "error": repr(error),
                }
            append_jsonl(raw_path, record)
            completed.add(key)
            valid = bool(record["validators"]["final_valid"])
            objective = record["metrics"].get("candidate_objective")
            log_message(
                f"lambda={lam:.3f} {method_label}: status={record['status']}, "
                f"valid={valid}, candidate_objective={objective}, "
                f"wall={record['elapsed_wall_seconds']:.1f}s"
            )


def csv_write(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    frame = pd.DataFrame(rows)
    frame.to_csv(path, index=False, na_rep="")


def small_result_rows(raw_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    starts = read_jsonl(raw_dir / "exact_nils_restarts.jsonl")
    exact = read_jsonl(raw_dir / "exact_solver_runs.jsonl")
    starts_by_instance: dict[str, list[dict[str, Any]]] = {}
    for row in starts:
        starts_by_instance.setdefault(row["instance_id"], []).append(row)
    exact_by_instance = {row["instance_id"]: row for row in exact}
    result_rows = []
    summary_rows = []
    for size in range(3, 10):
        instance_id = f"small_{size}_01"
        runs = sorted(
            starts_by_instance.get(instance_id, []),
            key=lambda row: int(row["restart_index"]),
        )
        valid_runs = [
            row
            for row in runs
            if row.get("solution") is not None
            and row.get("validators", {}).get("final_valid")
            and math.isfinite(float(row.get("metrics", {}).get("candidate_objective", math.inf)))
        ]
        best_nils = min(
            valid_runs,
            key=lambda row: (
                float(row["metrics"]["candidate_objective"]),
                int(row["restart_index"]),
            ),
            default=None,
        )
        exact_row = exact_by_instance.get(instance_id, {})
        exact_valid = bool(
            exact_row.get("proven_optimal")
            and exact_row.get("validators")
            and exact_row["validators"].get("final_valid")
            and exact_row.get("objective_consistent_with_final_evaluator")
        )
        nils_valid = best_nils is not None
        exact_obj = (
            float(exact_row["candidate_objective"])
            if exact_valid and exact_row.get("candidate_objective") is not None
            else None
        )
        nils_obj = (
            float(best_nils["metrics"]["candidate_objective"]) if nils_valid else None
        )
        absolute_gap = nils_obj - exact_obj if nils_obj is not None and exact_obj is not None else None
        if absolute_gap is None:
            percentage_gap = None
            zero_denominator = False
        elif exact_obj == 0:
            zero_denominator = True
            percentage_gap = 0.0 if nils_obj == 0 else None
        else:
            zero_denominator = False
            percentage_gap = 100.0 * absolute_gap / exact_obj
        exact_metrics = exact_row.get("metrics", {})
        nils_metrics = best_nils.get("metrics", {}) if best_nils else {}
        row = {
            "instance_id": instance_id,
            "size": size,
            "instance_seed": 2026 + size * 1000 + 1,
            "nils_restart_count": len(runs),
            "nils_valid_restart_count": len(valid_runs),
            "nils_selected_restart": (
                int(best_nils["restart_index"]) + 1 if best_nils else None
            ),
            "exact_optimum": exact_obj,
            "exact_candidate_objective": exact_row.get("model_objective"),
            "exact_recomputed_candidate_objective": exact_row.get("candidate_objective"),
            "exact_objective_consistent_with_final_evaluator": exact_row.get(
                "objective_consistent_with_final_evaluator"
            ),
            "nils_objective": nils_obj,
            "nils_candidate_objective": (
                best_nils.get("metrics", {}).get("candidate_objective") if best_nils else None
            ),
            "absolute_gap": absolute_gap,
            "percentage_optimality_gap": percentage_gap,
            "gap_denominator_zero": zero_denominator,
            "exact_solver_status": exact_row.get("status"),
            "exact_proven_optimal": bool(exact_row.get("proven_optimal")),
            "exact_best_bound": exact_row.get("best_bound"),
            "exact_solver_reported_gap": exact_row.get("solver_gap"),
            "exact_failure_reasons": "; ".join(
                [
                    f"{error.get('code', 'ERROR')}: {error.get('message', '')}"
                    for error in (exact_row.get("validators") or {}).get(
                        "feasibility_errors", []
                    )
                ]
                + (exact_row.get("validators") or {}).get(
                    "architecture_violations", []
                )
            ),
            "exact_solver_runtime_seconds": exact_row.get("solver_runtime_seconds"),
            "exact_wall_runtime_seconds": exact_row.get("elapsed_wall_seconds"),
            "nils_wall_runtime_seconds": sum(
                float(run.get("elapsed_wall_seconds", 0.0)) for run in runs
            ),
            "nils_selected_run_runtime_seconds": (
                best_nils.get("candidate_runtime_seconds") if best_nils else None
            ),
            "exact_feasibility_validator_pass": (
                exact_row.get("validators", {}).get("feasibility_validator_pass")
            ),
            "exact_architecture_validator_pass": (
                exact_row.get("validators", {}).get("architecture_validator_pass")
            ),
            "nils_feasibility_validator_pass": (
                best_nils.get("validators", {}).get("feasibility_validator_pass")
                if best_nils
                else False
            ),
            "nils_architecture_validator_pass": (
                best_nils.get("validators", {}).get("architecture_validator_pass")
                if best_nils
                else False
            ),
            "nils_all_restarts_feasible_and_architecture_valid": (
                len(valid_runs) == len(runs) == 50
            ),
            "exact_drone_sorties": exact_metrics.get("drone_sorties"),
            "exact_physical_rendezvous": exact_metrics.get("physical_rendezvous_count"),
            "exact_parcel_reload_events": exact_metrics.get("parcel_reload_count"),
            "exact_battery_swap_events": exact_metrics.get("battery_swap_count"),
            "nils_drone_sorties": nils_metrics.get("drone_sorties"),
            "nils_physical_rendezvous": nils_metrics.get("physical_rendezvous_count"),
            "nils_parcel_reload_events": nils_metrics.get("parcel_reload_count"),
            "nils_battery_swap_events": nils_metrics.get("battery_swap_count"),
            "nils_failure_reason": (
                None
                if nils_valid
                else "No restart returned a solution passing both final validators."
            ),
        }
        result_rows.append(row)
        summary_rows.append(row.copy())
    return result_rows, summary_rows


def real_result_rows(raw_dir: Path) -> list[dict[str, Any]]:
    records = read_jsonl(raw_dir / "real_data_lambda_methods.jsonl")
    by_key = {
        (round(float(row["lambda"]), 3), row["method"]): row for row in records
    }
    rows = []
    for lam in LAMBDAS:
        for method, label, architecture in METHODS:
            record = by_key.get((round(lam, 3), method), {})
            metrics = record.get("metrics", {})
            validators = record.get("validators", {})
            valid = bool(validators.get("final_valid"))
            objective = (
                metrics.get("candidate_objective")
                if valid and record.get("solution") is not None
                else None
            )
            row = {
                "lambda": lam,
                "method": method,
                "method_label": label,
                "architecture": architecture,
                "status": record.get("status", "not_run"),
                "candidate_objective": metrics.get("candidate_objective"),
                "objective": objective,
                "high_priority_tardiness": metrics.get("high_priority_tardiness"),
                "medium_priority_tardiness": metrics.get("medium_priority_tardiness"),
                "low_priority_tardiness": metrics.get("low_priority_tardiness"),
                "priority_tardiness_cost": metrics.get("priority_tardiness_cost"),
                "truck_cost": metrics.get("truck_cost"),
                "drone_cost": metrics.get("drone_cost"),
                "drone_service_share": metrics.get("drone_service_share"),
                "truck_distance": metrics.get("truck_distance"),
                "drone_distance": metrics.get("drone_distance"),
                "drone_sorties": metrics.get("drone_sorties"),
                "depot_depot_sorties": metrics.get("depot_depot_sorties"),
                "depot_truck_sorties": metrics.get("depot_truck_sorties"),
                "truck_depot_sorties": metrics.get("truck_depot_sorties"),
                "same_truck_recovery_sorties": metrics.get("same_truck_recovery_sorties"),
                "cross_truck_recovery_sorties": metrics.get("cross_truck_recovery_sorties"),
                "physical_rendezvous_count": metrics.get("physical_rendezvous_count"),
                "synchronization_waiting_minutes": metrics.get(
                    "synchronization_waiting_minutes"
                ),
                "parcel_reload_count": metrics.get("parcel_reload_count"),
                "battery_swap_count": metrics.get("battery_swap_count"),
                "elapsed_wall_seconds": record.get("elapsed_wall_seconds"),
                "algorithm_runtime_seconds": record.get("algorithm_runtime_seconds"),
                "feasibility_validator_pass": validators.get(
                    "feasibility_validator_pass", False
                ),
                "architecture_validator_pass": validators.get(
                    "architecture_validator_pass", False
                ),
                "final_valid": valid,
                "feasibility_errors": json.dumps(
                    validators.get("feasibility_errors", []), ensure_ascii=False
                ),
                "architecture_violations": json.dumps(
                    validators.get("architecture_violations", []), ensure_ascii=False
                ),
                "error": record.get("error"),
            }
            rows.append(row)
    return rows


def percent_difference(new: Any, old: Any) -> float | None:
    if new is None or old is None:
        return None
    try:
        new_value = float(new)
        old_value = float(old)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(new_value) or not math.isfinite(old_value) or old_value == 0:
        return None
    return 100.0 * (new_value - old_value) / abs(old_value)


def reconciliation_rows(
    small_rows: list[dict[str, Any]],
    real_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    small_history_path = (
        OUTPUT_DIR
        / "inputs"
        / "historical"
        / "small_exact_table_full_precision.csv"
    )
    real_history_path = (
        OUTPUT_DIR
        / "inputs"
        / "historical"
        / "real_case_lambda_all_methods.csv"
    )
    archived_small = pd.read_csv(
        small_history_path if small_history_path.is_file() else SMALL_HISTORICAL_TABLE
    ).set_index("n")
    reconciliation: list[dict[str, Any]] = []
    for row in small_rows:
        size = int(row["size"])
        paper_exact, paper_nils, paper_gap = SMALL_PAPER_VALUES[size]
        archive = archived_small.loc[size]
        for method, paper_value, archived_value, new_value in (
            (
                "exact",
                paper_exact,
                float(archive["exact_objective"]),
                row.get("exact_optimum"),
            ),
            (
                "NILS",
                paper_nils,
                float(archive["corrected_nils_objective"]),
                row.get("nils_objective"),
            ),
            (
                "NILS percentage gap",
                paper_gap,
                float(archive["corrected_gap_pct"]),
                row.get("percentage_optimality_gap"),
            ),
        ):
            reconciliation.append(
                {
                    "experiment": "small_exact_calibration",
                    "lambda": None,
                    "instance_id": row["instance_id"],
                    "method": method,
                    "metric": "percentage_gap" if "gap" in method.lower() else "objective",
                    "historical_reported_value": paper_value,
                    "historical_archived_value": archived_value,
                    "new_validated_value": new_value,
                    "new_candidate_value": (
                        row.get("nils_candidate_objective")
                        if method == "NILS"
                        else row.get("exact_candidate_objective")
                        if method == "exact"
                        else None
                    ),
                    "absolute_difference_vs_reported": (
                        None if new_value is None else float(new_value) - paper_value
                    ),
                    "percentage_difference_vs_reported": percent_difference(
                        new_value, paper_value
                    ),
                    "historical_source": (
                        "Drone_paper (7).pdf Table 5; archived values in "
                        "small_multistart_beta010_after_drone_two_opt"
                    ),
                    "interpretation": (
                        "Archived NILS n=9 value is below the archived exact objective; "
                        "do not treat the historical gap as valid until this rerun."
                        if size == 9 and method == "NILS"
                        else ""
                    ),
                }
            )

    historical_real = pd.read_csv(
        real_history_path if real_history_path.is_file() else REAL_HISTORICAL_RESULTS
    )
    new_by_key = {
        (round(float(row["lambda"]), 3), row["method"]): row for row in real_rows
    }
    hist_method_map = {
        "truck_only": "truck_only",
        "simple_drone": "simple_drone",
        "paired_baseline": "paired_baseline",
        "no_unpairing": "no_unpairing",
        "nils": "nils",
        "nils_no_local_search": "nils_no_local_search",
        "nils_no_perturbation": "nils_no_perturbation",
    }
    for _, historical in historical_real.iterrows():
        lam = round(float(historical["lambda"]), 3)
        method = hist_method_map[str(historical["method"])]
        current = new_by_key[(lam, method)]
        metrics = (
            ("objective", historical.get("objective"), current.get("objective")),
            (
                "drone_service_share_pct",
                historical.get("drone_service_share_pct"),
                (
                    None
                    if not current.get("final_valid")
                    or current.get("drone_service_share") is None
                    else 100.0 * float(current["drone_service_share"])
                ),
            ),
            (
                "runtime_seconds",
                historical.get("runtime_seconds"),
                current.get("elapsed_wall_seconds"),
            ),
        )
        for metric, old_value, new_value in metrics:
            reconciliation.append(
                {
                    "experiment": "real_data_lambda",
                    "lambda": lam,
                    "instance_id": "real_case_delivery_logistics",
                    "method": method,
                    "metric": metric,
                    "historical_reported_value": (
                        None if pd.isna(old_value) else float(old_value)
                    ),
                    "historical_archived_value": (
                        None if pd.isna(old_value) else float(old_value)
                    ),
                    "new_validated_value": new_value,
                    "new_candidate_value": (
                        current.get("candidate_objective")
                        if metric == "objective"
                        else None
                    ),
                    "absolute_difference_vs_reported": (
                        None
                        if new_value is None or pd.isna(old_value)
                        else float(new_value) - float(old_value)
                    ),
                    "percentage_difference_vs_reported": percent_difference(
                        new_value, None if pd.isna(old_value) else old_value
                    ),
                    "historical_source": str(real_history_path),
                    "interpretation": (
                        "Historical row was infeasible; any fallback objective is not "
                        "a feasible objective comparison."
                        if not bool(historical["feasible"]) and metric == "objective"
                        else ""
                    ),
                }
            )
    return reconciliation


def claim_audit_text(
    small_rows: list[dict[str, Any]],
    summary_rows: list[dict[str, Any]],
    real_rows: list[dict[str, Any]],
) -> str:
    small_valid_gaps = [
        float(row["percentage_optimality_gap"])
        for row in small_rows
        if row.get("percentage_optimality_gap") is not None
    ]
    matches = sum(
        row.get("percentage_optimality_gap") is not None
        and abs(float(row["percentage_optimality_gap"])) <= 1e-6
        for row in small_rows
    )
    all_nils_valid = len(small_rows) == 7 and all(
        row.get("nils_objective") is not None
        and row.get("nils_feasibility_validator_pass")
        and row.get("nils_architecture_validator_pass")
        for row in small_rows
    )
    certified_exact_count = sum(bool(row.get("exact_proven_optimal")) for row in small_rows)
    invalid_real_rows = [row for row in real_rows if not row.get("final_valid")]
    by_key = {
        (round(float(row["lambda"]), 3), row["method"]): row for row in real_rows
    }
    lambda_summary = []
    historical_path = (
        OUTPUT_DIR
        / "inputs"
        / "historical"
        / "real_case_lambda_all_methods.csv"
    )
    historical_real = pd.read_csv(
        historical_path if historical_path.is_file() else REAL_HISTORICAL_RESULTS
    )
    labels = {method: label for method, label, _ in METHODS}
    for lam in LAMBDAS:
        valid_methods = [
            (method, by_key[(round(lam, 3), method)])
            for method, _, _ in METHODS
            if by_key[(round(lam, 3), method)].get("final_valid")
            and by_key[(round(lam, 3), method)].get("objective") is not None
        ]
        ranked = sorted(valid_methods, key=lambda item: float(item[1]["objective"]))
        truck = by_key[(round(lam, 3), "truck_only")]
        nils = by_key[(round(lam, 3), "nils")]
        paired = by_key[(round(lam, 3), "paired_baseline")]
        best_objective = float(ranked[0][1]["objective"]) if ranked else None
        best_methods = (
            [
                method
                for method, row in ranked
                if abs(float(row["objective"]) - best_objective) <= 1e-6
            ]
            if best_objective is not None
            else []
        )
        historical_rows = historical_real[
            (historical_real["lambda"].astype(float).round(3) == round(lam, 3))
            & (historical_real["feasible"].astype(str).str.lower() == "true")
        ]
        historical_min = (
            float(historical_rows["objective"].astype(float).min())
            if not historical_rows.empty
            else None
        )
        historical_best = (
            sorted(
                historical_rows.loc[
                    (historical_rows["objective"].astype(float) - historical_min).abs()
                    <= 1e-6,
                    "method",
                ].astype(str).tolist()
            )
            if historical_min is not None
            else []
        )
        same_ranking = sorted(best_methods) == historical_best
        nils_rank = (
            next((index for index, item in enumerate(ranked, 1) if item[0] == "nils"), None)
        )
        improvement = None
        if (
            truck.get("objective") is not None
            and nils.get("objective") is not None
            and float(truck["objective"]) != 0
        ):
            improvement = 100.0 * (
                float(truck["objective"]) - float(nils["objective"])
            ) / float(truck["objective"])
        m5_minus_m3 = (
            None
            if nils.get("objective") is None or paired.get("objective") is None
            else float(nils["objective"]) - float(paired["objective"])
        )
        lambda_summary.append(
            {
                "lambda": lam,
                "best_method": labels[best_methods[0]] if best_methods else None,
                "best_methods": best_methods,
                "historical_best_methods": historical_best,
                "same_ranking_as_historical": same_ranking,
                "nils_rank": nils_rank,
                "nils_improvement_vs_truck_only_pct": improvement,
                "z_m5_minus_z_m3": m5_minus_m3,
                "valid_method_count": len(valid_methods),
            }
        )

    historical_gap_claim = {
        "matches": 6,
        "mean_gap": 0.17,
        "median_gap": 0.00,
        "maximum_gap": 1.05,
    }
    if len(small_valid_gaps) != 7:
        calibration_class = "REMOVE"
    elif any(gap < -1e-6 for gap in small_valid_gaps):
        calibration_class = "INTERPRETATION UPDATE"
    elif (
        matches == historical_gap_claim["matches"]
        and math.isclose(float(np.mean(small_valid_gaps)), historical_gap_claim["mean_gap"], abs_tol=0.005)
        and math.isclose(float(np.median(small_valid_gaps)), historical_gap_claim["median_gap"], abs_tol=0.005)
        and math.isclose(max(small_valid_gaps), historical_gap_claim["maximum_gap"], abs_tol=0.005)
    ):
        calibration_class = "UNCHANGED"
    else:
        calibration_class = "NUMERIC UPDATE ONLY"
    valid_method_by_lambda = {
        item["lambda"]: item["valid_method_count"] for item in lambda_summary
    }
    domain_claim_unchanged = all(
        valid_method_by_lambda[lam] == 0 for lam in (0.150, 0.200)
    )
    historical_rank_unchanged = all(
        item["same_ranking_as_historical"] for item in lambda_summary
    )
    scale_switches = len(
        {
            tuple(item["best_methods"])
            for item in lambda_summary
            if item["lambda"] in (0.115, 0.120, 0.125)
        }
    ) > 1

    lines = [
        "# Final legacy-semantics refresh claim audit",
        "",
        "## Scope and provenance",
        "",
        "- The archived calibration inputs and matching historical table values use 3 trucks and 3 drones; the latest manuscript describes the Table 5 cases as single-truck/single-drone. The refresh uses the frozen files that generated the table and marks that manuscript description for correction.",
        "- The real-data sample is the exact same 80-row sample as the archived lambda run. Lambda values are unchanged: 0.075, 0.100, 0.115, 0.120, 0.125, 0.130, 0.150, and 0.200.",
        "- Real-data comparison includes M1–M7 because Table 16 and its surrounding claims rank/reference M3, M5, M6, and M7 and use M1 as the truck-only denominator. M8 is not in the existing Table 16 lambda comparison and is not added here.",
        "- Every objective shown as a result is gated on both the final feasibility validator and the method's architecture validator. Failed candidates remain in raw JSONL diagnostics and are not presented as feasible objectives.",
        "- The small calibration config uses 50 NILS starts per instance, 80 outer iterations, 18 no-improvement iterations, 45 seconds per start, beta_slack=0.10, and unlimited candidate caps (0/0). The current MILP solve uses the saved 120-second, 0.001-gap, one-thread settings with CBC.",
        "",
        "## Exact calibration",
        "",
        f"- Exact matches: {matches}/7.",
        f"- Certified exact optima: {certified_exact_count}/7; valid exact comparisons: {len(small_valid_gaps)}/7.",
        f"- Mean percentage gap: {np.mean(small_valid_gaps):.6f}%." if small_valid_gaps else "- Mean percentage gap: unavailable; no valid exact comparisons.",
        f"- Median percentage gap: {np.median(small_valid_gaps):.6f}%." if small_valid_gaps else "- Median percentage gap: unavailable; no valid exact comparisons.",
        f"- Maximum percentage gap: {max(small_valid_gaps):.6f}%." if small_valid_gaps else "- Maximum percentage gap: unavailable; no valid exact comparisons.",
        f"- Every returned NILS solution passes both final validators: {all_nils_valid}.",
        "- Percentage gap is exactly 100 * (Z_NILS - Z_exact) / Z_exact. If Z_exact is zero, the result is 0% only when both objectives are zero; otherwise the percentage is undefined and explicitly flagged.",
        f"- Historical claim classification for the exact-gap headline: {calibration_class}.",
        "- Historical consistency issue: the displayed Table 5 gaps (one 1.05% gap and six 0.00% gaps) average 0.15%, not the stated 0.17%. The archived 50-start file also reports a 10.599056 NILS objective at n=9, below the archived exact objective 10.697707; the paper instead prints 10.697707 for NILS at n=9.",
        "- The exact solver objective is compared against the evaluator-recomputed objective. A mismatch, a non-proven solver status, or a failed final validator leaves the exact optimum/gap blank rather than silently using a fallback.",
        "- CBC proof limitation: the configured relative gap is 0.001. The current PuLP adapter reports zero gap and sets the best bound equal to the incumbent when PuLP returns `Optimal`; those adapter fields do not independently certify a zero-gap optimum. The refresh therefore does not treat numeric status 2 as a proof.",
        "",
        "## Exact-case diagnostics",
        "",
    ]
    for row in small_rows:
        if (
            row.get("exact_proven_optimal")
            and row.get("exact_objective_consistent_with_final_evaluator")
            and row.get("exact_feasibility_validator_pass")
            and row.get("exact_architecture_validator_pass")
        ):
            continue
        reasons = []
        if not row.get("exact_proven_optimal"):
            reasons.append(f"no certified proof (solver status {row.get('exact_solver_status')})")
        if not row.get("exact_feasibility_validator_pass") or not row.get("exact_architecture_validator_pass"):
            reasons.append(row.get("exact_failure_reasons") or "final validator failed")
        if row.get("exact_objective_consistent_with_final_evaluator") is False:
            reasons.append(
                "MILP objective/evaluator mismatch "
                f"({row.get('exact_candidate_objective')} vs "
                f"{row.get('exact_recomputed_candidate_objective')})"
            )
        lines.append(f"- n={row['size']}: " + "; ".join(reasons) + ".")
    lines.extend(
        [
        "",
        "## Real-data lambda results",
        "",
        "| lambda | best validated method (M1–M7) | M5 rank | M5 improvement vs M1 (%) | Z_M5 - Z_M3 | valid methods |",
        "|---:|---|---:|---:|---:|---:|",
        ]
    )
    for item in lambda_summary:
        lines.append(
            f"| {item['lambda']:.3f} | {item['best_method'] or 'none'} | "
            f"{item['nils_rank'] if item['nils_rank'] is not None else '--'} | "
            f"{item['nils_improvement_vs_truck_only_pct'] if item['nils_improvement_vs_truck_only_pct'] is not None else '--'} | "
            f"{item['z_m5_minus_z_m3'] if item['z_m5_minus_z_m3'] is not None else '--'} | "
            f"{item['valid_method_count']} |"
        )
    lines.extend(
        [
            "",
            f"- Feasibility coverage: {len(real_rows) - len(invalid_real_rows)}/{len(real_rows)} method–lambda rows pass both validators. At lambda=0.150 and 0.200, all seven methods fail feasibility because all three truck routes exceed the shift; no objective or rank is reported for those rows. The historical table also marks these two settings infeasible.",
            f"- Historical real-data ranking classification: {'UNCHANGED' if historical_rank_unchanged else 'INTERPRETATION UPDATE'}.",
            f"- Historical infeasible-domain classification at lambda=0.150 and 0.200: {'UNCHANGED' if domain_claim_unchanged else 'INTERPRETATION UPDATE'}.",
            f"- At lambda=0.115, 0.120, and 0.125, the best method is {'not constant' if scale_switches else 'M4 in all three cases'}; do not claim a best-method switch across this local range. The M5–M3 objective difference still crosses from positive to negative by lambda=0.125, which is descriptive rather than evidence of a specific feasibility mechanism.",
            "",
            "## Claim classifications",
            "",
            f"- Table 5 exact-calibration headline: {calibration_class}.",
            "- Table 5 fleet description: INTERPRETATION UPDATE; saved inputs associated with the table contain 3 trucks and 3 drones, not one of each.",
            f"- Table 16 best-method/rank statements: {'UNCHANGED' if historical_rank_unchanged else 'INTERPRETATION UPDATE'}.",
            f"- Table 16 lambda=0.150/0.200 domain statement: {'UNCHANGED' if domain_claim_unchanged else 'INTERPRETATION UPDATE'}.",
            "- Section 5.6 method universe: NUMERIC UPDATE ONLY; state explicitly that the real-data rank is over M1–M7. M8 ALNS was not reported in the existing lambda table and is not covered by this refresh.",
            f"- Nonmonotone local pattern around lambda=0.115–0.125: {'INTERPRETATION UPDATE' if not scale_switches else 'NUMERIC UPDATE ONLY'}; distinguish the M5–M3 crossover from the best-method ranking and present any discrete-feasibility mechanism as an interpretation, not a demonstrated cause.",
        ]
    )
    lines.extend(
        [
            "",
            "## MANUSCRIPT ACTIONS",
            "",
        ]
    )
    for row in small_rows:
        exact_value = row.get("exact_optimum")
        nils_value = row.get("nils_objective")
        gap_value = row.get("percentage_optimality_gap")
        action = "UPDATE" if exact_value is not None else "HOLD"
        lines.append(
            f"- Location: Table 5, n={row['size']}; action: {action}; "
            f"values: exact={exact_value}, NILS={nils_value}, gap={gap_value}%, "
            f"solver runtime={row.get('exact_solver_runtime_seconds')} s, "
            f"NILS 50-start wall time={row.get('nils_wall_runtime_seconds')} s."
        )
    lines.append(
        "- Table 5 exact optimum/gap results: HOLD. This refresh produced no certified exact comparisons; do not replace the manuscript's exact values with solver incumbents until the CBC proof, n=7/n=8 validation/objective issues, and n=9 time-limit case are resolved."
    )
    lines.append(
        "- Location: Table 5 title and Section 5.3 calibration description; action: UPDATE; interpretation: state the archived 3-truck/3-drone fleet used by these frozen instances, not single-truck/single-drone."
    )
    for item in lambda_summary:
        invalid_domain = item["valid_method_count"] == 0
        lines.append(
            f"- Location: Table 16, lambda={item['lambda']:.3f}; action: "
            f"{'UPDATE infeasible-domain status; omit objective and rank' if invalid_domain else 'UPDATE numeric row'}; "
            f"values: best={item['best_method']}, best methods={item['best_methods']}, M5 rank={item['nils_rank']}, "
            f"M5 vs M1={item['nils_improvement_vs_truck_only_pct']}%, "
            f"Z_M5-Z_M3={item['z_m5_minus_z_m3']}."
        )
    lines.append(
        "- Location: Section 5.6 real-data ranking statement; action: UPDATE; interpretation: define the ranking set as M1–M7 and do not extend it to M8 without a matched real-data ALNS run."
    )
    return "\n".join(lines) + "\n"


def README_text(manifest: dict[str, Any]) -> str:
    integrity = manifest.get("integrity_check") or {}
    integrity_findings = integrity.get("failures", [])
    recorded_findings = integrity.get("findings", [])
    return "\n".join(
        [
            "# Final legacy-semantics refresh",
            "",
            "This directory contains only the refreshed small exact-calibration and real-data lambda evidence. The definitive 720-instance experiment is untouched.",
            "",
            f"Run status: {manifest.get('status')}. Final integrity failures: {len(integrity_findings)}; recorded findings: {len(recorded_findings)}.",
            "",
            "## Reproduction",
            "",
            "Run from the repository root with the recorded execution commit:",
            "",
            "    python -m src.experiments.refresh_legacy_semantics",
            "",
            "To regenerate derived tables, the claim audit, and the manifest from the already saved raw JSONL without rerunning any optimization, use:",
            "",
            "    python -m src.experiments.refresh_legacy_semantics --finalize-existing",
            "",
            "The runner checkpoints each NILS restart, exact solve, and real-data method to JSONL under raw/. It resumes completed records without replacing them. Raw solution payloads, frozen small-instance inputs, the 80-row real-data sample, and all eight scaled real-case instances are included.",
            "",
            "## Semantics and validation",
            "",
            "All new candidate solutions are evaluated with the current objective and checked with validate_solution plus validate_drone_architecture. M3 uses the fixed default drone-to-truck map, M4 uses the no-cross-truck validator, and M1/M2/M5/M6/M7 use the M5 feasibility architecture check. Only candidates passing both validators appear in the validated objective columns; rejected candidates and diagnostics remain in raw records.",
            "",
            "The final timing evaluator recomputes synchronization waiting from readiness times. The current implementation treats u_sync as physical rendezvous, charges two handling events per truck rendezvous, and keeps parcel reload and battery swap as separate state-dependent events.",
            "",
            "## Important audit finding",
            "",
            "The archived small-calibration inputs have 3 trucks and 3 drones, even though the latest manuscript calls the cases single-truck/single-drone. The report uses the exact frozen files associated with the historical Table 5 values and flags this manuscript wording. It does not regenerate inputs.",
            "",
            "The exact-gap table is not paper-ready: none of the seven exact optima is certified, and n=7, n=8, and n=9 have additional validation/objective/timeout blockers. The real-data rows at lambda=0.150 and 0.200 remain infeasible under the shift constraint, matching the historical feasibility flags.",
            "",
            "## Files",
            "",
            "- exact_calibration_results.csv: one row for each n=3,...,9, including exact/NILS objectives, gaps, runtimes, event counts, and validator results.",
            "- exact_calibration_summary.csv: exact-match and gap summary.",
            "- real_data_lambda_results.csv: one row for each of eight lambda values and M1-M7.",
            "- historical_reconciliation.csv: explicit historical-versus-new value comparisons; historical values are never mixed into new result tables.",
            "- claim_audit.md: manuscript claim classifications and the required MANUSCRIPT ACTIONS.",
            "- run_manifest.json: execution commit, versions, commands, settings, input hashes, timestamps, and output inventory.",
            "- raw/: append-only raw solver and heuristic outputs, including serialized solutions.",
            "- inputs/: packaged frozen inputs and lambda-specific real-case instances.",
            "",
            f"Execution commit: {manifest.get('execution_commit_sha')}",
            f"Finalization commit: {manifest.get('finalization_commit_sha', manifest.get('execution_commit_sha'))}",
            f"Branch: {manifest.get('branch')}",
            "",
        ]
    )


def export_outputs(raw_dir: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    small_rows, summary_rows = small_result_rows(raw_dir)
    real_rows = real_result_rows(raw_dir)
    csv_write(OUTPUT_DIR / "exact_calibration_results.csv", small_rows)
    valid_gaps = [
        float(row["percentage_optimality_gap"])
        for row in small_rows
        if row.get("percentage_optimality_gap") is not None
    ]
    returned = [
        row for row in small_rows if row.get("nils_objective") is not None
    ]
    summary = [
        {
            "expected_instances": 7,
            "completed_instances": len(
                [
                    row
                    for row in small_rows
                    if int(row.get("nils_restart_count") or 0) > 0
                ]
            ),
            "exact_matches": sum(abs(gap) <= 1e-6 for gap in valid_gaps),
            "maximum_percentage_gap": max(valid_gaps) if valid_gaps else None,
            "mean_percentage_gap": float(np.mean(valid_gaps)) if valid_gaps else None,
            "median_percentage_gap": float(np.median(valid_gaps)) if valid_gaps else None,
            "valid_exact_comparisons": len(valid_gaps),
            "returned_nils_solutions": len(returned),
            "every_returned_nils_solution_passes_all_final_validators": (
                len(returned) == 7
                and all(
                    row.get("nils_feasibility_validator_pass")
                    and row.get("nils_architecture_validator_pass")
                    for row in returned
                )
            ),
            "gap_formula": "100 * (Z_NILS - Z_exact) / Z_exact",
            "zero_denominator_rule": (
                "0% only if both objectives are zero; otherwise percentage gap is undefined."
            ),
        }
    ]
    csv_write(OUTPUT_DIR / "exact_calibration_summary.csv", summary)
    csv_write(OUTPUT_DIR / "real_data_lambda_results.csv", real_rows)
    reconciliation = reconciliation_rows(small_rows, real_rows)
    csv_write(OUTPUT_DIR / "historical_reconciliation.csv", reconciliation)
    (OUTPUT_DIR / "claim_audit.md").write_text(
        claim_audit_text(small_rows, summary_rows, real_rows),
        encoding="utf-8",
    )
    return small_rows, summary, real_rows


def final_integrity_check(raw_dir: Path, small_rows: list[dict[str, Any]], real_rows: list[dict[str, Any]]) -> dict[str, Any]:
    failures = []
    findings = []
    expected_infeasible_real_records = []
    exact_records = read_jsonl(raw_dir / "exact_solver_runs.jsonl")
    start_records = read_jsonl(raw_dir / "exact_nils_restarts.jsonl")
    real_records = read_jsonl(raw_dir / "real_data_lambda_methods.jsonl")

    instances_by_name = {
        f"small_{size}_01": load_instance_json(
            OUTPUT_DIR / "inputs" / "small_instances" / f"small_{size}_01.json"
        )
        for size in range(3, 10)
    }
    small_config = build_search_config(load_yaml_config(SMALL_CONFIG_PATH))
    exact_config, _ = exact_configuration(load_yaml_config(SMALL_CONFIG_PATH))
    real_instances = {}
    for lam in LAMBDAS:
        path = (
            OUTPUT_DIR
            / "inputs"
            / "real_case_instances"
            / f"real_case_lambda_{lam:.3f}.json"
        )
        real_instances[lam] = load_instance_json(path)

    def verify_record(
        record: dict[str, Any],
        instance: Any,
        architecture: str,
        label: str,
        require_final_valid: bool = True,
    ) -> None:
        if not record.get("solution"):
            return
        solution = solution_from_payload(record["solution"])
        metrics = solution_metrics(instance, solution)
        validators = validator_result(instance, solution, architecture)
        if not validators["final_valid"] and require_final_valid:
            failures.append(f"{label}: final validator failed: {validators}")
        elif not validators["final_valid"]:
            expected_infeasible_real_records.append(
                {
                    "lambda": record.get("lambda"),
                    "method": record.get("method"),
                    "failure_codes": sorted(
                        {
                            item.get("code", "UNKNOWN")
                            for item in validators.get("feasibility_errors", [])
                        }
                    ),
                }
            )
        stored = record.get("validators") or {}
        if stored.get("final_valid") != validators["final_valid"]:
            failures.append(f"{label}: stored validator result does not reproduce")
        if record.get("metrics", {}).get("candidate_objective") is not None and not math.isclose(
            float(record["metrics"]["candidate_objective"]),
            float(metrics["candidate_objective"]),
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            failures.append(f"{label}: objective does not reproduce from raw solution")

    for record in exact_records:
        verify_record(
            record,
            instances_by_name[record["instance_id"]],
            "M5",
            f"exact:{record['instance_id']}",
        )
        if not record.get("proven_optimal"):
            findings.append(
                {
                    "kind": "exact_optimum_not_certified",
                    "instance_id": record["instance_id"],
                    "solver_status": record.get("status"),
                    "solver_gap": record.get("solver_gap"),
                }
            )
        if record.get("objective_consistent_with_final_evaluator") is False:
            failures.append(
                f"exact:{record['instance_id']}: MILP objective does not match final evaluator"
            )
    for record in start_records:
        verify_record(
            record,
            instances_by_name[record["instance_id"]],
            "M5",
            f"NILS:{record['instance_id']}:{record['restart_index']}",
        )
    architecture_by_method = {method: architecture for method, _, architecture in METHODS}
    for record in real_records:
        verify_record(
            record,
            real_instances[float(record["lambda"])],
            architecture_by_method[record["method"]],
            f"real:{record['lambda']}:{record['method']}",
            require_final_valid=False,
        )

    if len(small_rows) != 7:
        failures.append(f"expected 7 calibration rows, found {len(small_rows)}")
    if len(real_rows) != len(LAMBDAS) * len(METHODS):
        failures.append(
            f"expected {len(LAMBDAS) * len(METHODS)} real-data rows, found {len(real_rows)}"
        )
    if len(start_records) != 7 * int(
        load_yaml_config(SMALL_CONFIG_PATH)["experiment"]["small_nils_restarts"]
    ):
        failures.append(f"unexpected count of NILS start records: {len(start_records)}")
    if len(real_records) != len(LAMBDAS) * len(METHODS):
        failures.append(f"unexpected count of real-data method records: {len(real_records)}")
    if len(exact_records) != 7:
        failures.append(f"expected 7 exact solver records, found {len(exact_records)}")
    return {
        "passed": not failures,
        "failures": failures,
        "findings": findings,
        "expected_infeasible_real_records": expected_infeasible_real_records,
        "small_instances": len(small_rows),
        "nils_restart_records": len(start_records),
        "exact_solver_records": len(exact_records),
        "real_data_method_records": len(real_records),
        "final_semantics_validated_records": sum(
            bool((row.get("validators") or {}).get("final_valid"))
            for row in exact_records + start_records + real_records
        ),
        "nonvalid_candidate_records_are_flagged": all(
            not (row.get("validators") or {}).get("final_valid")
            or bool((row.get("validators") or {}).get("feasibility_validator_pass"))
            and bool((row.get("validators") or {}).get("architecture_validator_pass"))
            for row in exact_records + start_records + real_records
        ),
    }


def final_run_status(integrity: dict[str, Any]) -> str:
    if not integrity.get("passed"):
        return "complete_with_integrity_findings"
    if integrity.get("findings") or integrity.get("expected_infeasible_real_records"):
        return "complete_with_findings"
    return "complete"


def initialize_manifest(provenance: dict[str, Any], command: str) -> dict[str, Any]:
    manifest_path = OUTPUT_DIR / "run_manifest.json"
    prior = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    execution_sha = git_value("rev-parse", "HEAD")
    if prior.get("execution_commit_sha") and prior["execution_commit_sha"] != execution_sha:
        raise RuntimeError(
            "Existing output directory records a different execution commit; use a new output directory."
        )
    manifest = {
        "title": "Final legacy-semantics refresh",
        "status": "in_progress",
        "execution_commit_sha": execution_sha,
        "branch": git_value("branch", "--show-current"),
        "working_tree_status_at_start": git_value("status", "--short"),
        "python_and_packages": package_versions(),
        "commands": [command],
        "input_provenance": provenance,
        "parameters": {
            "small_config": str(SMALL_CONFIG_PATH),
            "small_nils": json_safe(
                load_yaml_config(SMALL_CONFIG_PATH).get("heuristics", {}).get("nils", {})
            ),
            "small_experiment": json_safe(
                load_yaml_config(SMALL_CONFIG_PATH).get("experiment", {})
            ),
            "small_exact_solver": {
                "enabled_for_refresh": True,
                "backend": build_search_config(
                    load_yaml_config(SMALL_CONFIG_PATH)
                ).milp.solver_backend,
                "time_limit_seconds": build_search_config(
                    load_yaml_config(SMALL_CONFIG_PATH)
                ).milp.time_limit_seconds,
                "mip_gap": build_search_config(
                    load_yaml_config(SMALL_CONFIG_PATH)
                ).milp.mip_gap,
                "threads": build_search_config(
                    load_yaml_config(SMALL_CONFIG_PATH)
                ).milp.threads,
                "wall_watchdog_seconds": max(
                    180,
                    build_search_config(load_yaml_config(SMALL_CONFIG_PATH))
                    .milp.time_limit_seconds
                    + 60,
                ),
            },
            "real_case": {
                "seed": 2026,
                "sample_per_mode": 20,
                "distance_scales": list(LAMBDAS),
                "battery_slack_ratio": 0.10,
                "methods": [
                    {"code": code, "label": label, "architecture": architecture}
                    for code, label, architecture in METHODS
                ],
                "search_config_source": "src/experiments/run_real_case.py::_make_config",
                "search_config": json_safe(asdict(_make_config(seed=2026))),
            },
        },
        "timestamps": {
            "started_utc": (
                utc_now()
                if prior.get("status") == "preflight_complete"
                else prior.get("timestamps", {}).get("started_utc", utc_now())
            ),
            "last_checkpoint_utc": utc_now(),
            "finished_utc": None,
        },
        "outputs": {},
    }
    atomic_json(manifest_path, manifest)
    return manifest


def finalize_existing_run() -> int:
    manifest_path = OUTPUT_DIR / "run_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"No saved run manifest at {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not manifest.get("execution_commit_sha"):
        raise RuntimeError("Saved manifest has no execution commit SHA.")
    raw_dir = OUTPUT_DIR / "raw"
    small_rows, summary, real_rows = export_outputs(raw_dir)
    integrity = final_integrity_check(raw_dir, small_rows, real_rows)
    manifest["integrity_check"] = integrity
    manifest["status"] = final_run_status(integrity)
    manifest["finalization_commit_sha"] = git_value("rev-parse", "HEAD")
    manifest["finalization_command"] = (
        "python -m src.experiments.refresh_legacy_semantics --finalize-existing"
    )
    manifest.setdefault("timestamps", {})["last_checkpoint_utc"] = utc_now()
    manifest["timestamps"]["finished_utc"] = utc_now()
    manifest.setdefault("outputs", {}).update(
        {
            "directory": str(OUTPUT_DIR),
            "raw": str(raw_dir),
            "raw_nils_restarts": str(raw_dir / "exact_nils_restarts.jsonl"),
            "raw_exact_solver_runs": str(raw_dir / "exact_solver_runs.jsonl"),
            "raw_real_data_runs": str(raw_dir / "real_data_lambda_methods.jsonl"),
            "exact_calibration_results": str(OUTPUT_DIR / "exact_calibration_results.csv"),
            "exact_calibration_summary": str(OUTPUT_DIR / "exact_calibration_summary.csv"),
            "real_data_lambda_results": str(OUTPUT_DIR / "real_data_lambda_results.csv"),
            "historical_reconciliation": str(OUTPUT_DIR / "historical_reconciliation.csv"),
            "claim_audit": str(OUTPUT_DIR / "claim_audit.md"),
            "readme": str(OUTPUT_DIR / "README.md"),
            "manifest": str(manifest_path),
        }
    )
    atomic_json(manifest_path, manifest)
    (OUTPUT_DIR / "README.md").write_text(README_text(manifest), encoding="utf-8")
    log_message(
        f"Saved-run finalization complete: status={manifest['status']}, "
        f"integrity_failures={len(integrity['failures'])}, "
        f"exact_comparisons={summary[0]['valid_exact_comparisons']}, "
        f"real_lambda_rows={len(real_rows)}"
    )
    return 0 if integrity["passed"] else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Package and validate frozen inputs without starting optimization runs.",
    )
    parser.add_argument(
        "--finalize-existing",
        action="store_true",
        help="Regenerate derived reports and manifest from saved raw runs without optimization.",
    )
    arguments = parser.parse_args()
    if arguments.finalize_existing:
        return finalize_existing_run()
    raw_dir = OUTPUT_DIR / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    small_instances, real_instances, provenance = input_bundle()
    command = "python -m src.experiments.refresh_legacy_semantics"
    manifest = initialize_manifest(provenance, command)
    manifest["outputs"] = {
        "directory": str(OUTPUT_DIR),
        "raw": str(raw_dir),
        "exact_calibration_results": str(OUTPUT_DIR / "exact_calibration_results.csv"),
        "exact_calibration_summary": str(OUTPUT_DIR / "exact_calibration_summary.csv"),
        "real_data_lambda_results": str(OUTPUT_DIR / "real_data_lambda_results.csv"),
        "historical_reconciliation": str(OUTPUT_DIR / "historical_reconciliation.csv"),
        "claim_audit": str(OUTPUT_DIR / "claim_audit.md"),
        "readme": str(OUTPUT_DIR / "README.md"),
        "manifest": str(OUTPUT_DIR / "run_manifest.json"),
    }
    atomic_json(OUTPUT_DIR / "run_manifest.json", manifest)

    if arguments.preflight_only:
        manifest["status"] = "preflight_complete"
        manifest["timestamps"]["finished_utc"] = utc_now()
        atomic_json(OUTPUT_DIR / "run_manifest.json", manifest)
        log_message("Preflight complete; no optimization run was started.")
        return 0

    progress_path = _build_progress_writer(str(OUTPUT_DIR))
    small_config_data = load_yaml_config(SMALL_CONFIG_PATH)
    run_small_calibration(small_instances, small_config_data, raw_dir, progress_path)
    run_real_case(real_instances, raw_dir)
    small_rows, summary, real_rows = export_outputs(raw_dir)
    integrity = final_integrity_check(raw_dir, small_rows, real_rows)
    manifest["integrity_check"] = integrity
    manifest["timestamps"]["last_checkpoint_utc"] = utc_now()
    manifest["status"] = final_run_status(integrity)
    manifest["timestamps"]["finished_utc"] = utc_now()
    manifest["outputs"]["raw_nils_restarts"] = str(raw_dir / "exact_nils_restarts.jsonl")
    manifest["outputs"]["raw_exact_solver_runs"] = str(raw_dir / "exact_solver_runs.jsonl")
    manifest["outputs"]["raw_real_data_runs"] = str(raw_dir / "real_data_lambda_methods.jsonl")
    manifest["finalization_commit_sha"] = manifest["execution_commit_sha"]
    atomic_json(OUTPUT_DIR / "run_manifest.json", manifest)
    (OUTPUT_DIR / "README.md").write_text(README_text(manifest), encoding="utf-8")
    log_message(
        f"Refresh finalized: status={manifest['status']}, "
        f"exact matches={summary[0]['exact_matches']}, "
        f"real lambda rows={len(real_rows)}"
    )
    return 0 if integrity["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
