from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..feasibility import is_feasible, validate_solution
from ..heuristics.architecture import count_cross_truck_recoveries, sortie_associations, validate_drone_architecture
from ..heuristics.nils import default_drone_to_truck, run_nils
from ..instance_generator import load_instance_json
from ..parameters import load_and_build_config
from .heuristic_study import (
    _run_method,
    _scenario_config,
    _truck_routes_sha256,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config" / "experiment_full_revised_usync_20261030_budgeted.yaml"
DEFAULT_SOURCE = ROOT / "outputs" / "nils_candidate_cap_reservoir_diagnostic_20261105"
DEFAULT_OUTPUT = ROOT / "outputs" / "matched_m3_m4_m5_alns_common_backbone_20261106"
DEFAULT_PAPER_RUN = ROOT / "outputs" / "full_revised_usync_20261101_paper_ready"
METHODS = ("paired_baseline", "no_unpairing", "nils", "feasibility_aware_alns")
METHOD_LABELS = {
    "paired_baseline": "M3",
    "no_unpairing": "M4",
    "nils": "M5",
    "feasibility_aware_alns": "ALNS",
}
RUN_FIELDS = [
    "instance_id",
    "scenario_id",
    "size",
    "method",
    "method_label",
    "instance_seed",
    "method_seed",
    "objective",
    "feasible",
    "high_priority_tardiness_minutes",
    "drone_served_customers",
    "drone_service_share",
    "cross_truck_recoveries",
    "architecture",
    "architecture_valid",
    "architecture_violations",
    "iterations",
    "cpu_seconds",
    "runtime_limit_seconds",
    "max_iterations",
    "max_displacement_candidates",
    "max_repair_candidates",
    "initial_truck_backbone_sha256",
    "initial_backbone_reused",
    "status",
    "error",
]
SUMMARY_FIELDS = [
    "size",
    "method",
    "runs",
    "feasible_runs",
    "feasibility_rate",
    "mean_objective_feasible",
    "median_objective_feasible",
    "mean_high_priority_tardiness_feasible",
    "mean_drone_service_share_feasible",
    "mean_cross_truck_recoveries",
    "mean_iterations",
    "mean_cpu_seconds",
]


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temp_path.replace(path)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
    temp_path.replace(path)


def _append_csv(path: Path, row: dict[str, Any], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        if write_header:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()


def _git_value(*args: str) -> str:
    try:
        return subprocess.check_output(args, cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def _selected_instances(source_dir: Path) -> tuple[list[dict[str, str]], dict[str, Any]]:
    runs_path = source_dir / "nils_timeout_diagnostic_runs.csv"
    metadata_path = source_dir / "run_provenance.json"
    rows = _read_csv(runs_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    ids = list(metadata.get("selected_instance_ids", []))
    by_id = {row["instance_id"]: row for row in rows if row.get("status") == "completed"}
    selected = [by_id[instance_id] for instance_id in ids if instance_id in by_id]
    if len(selected) != len(ids) or len(selected) != 30:
        raise RuntimeError(f"Expected the exact completed 30-instance diagnostic; found {len(selected)} of {len(ids)}")
    counts: dict[int, int] = {}
    for row in selected:
        size = int(row["size"])
        counts[size] = counts.get(size, 0) + 1
    if sorted(counts.values()) != [10, 10, 10]:
        raise RuntimeError(f"Expected 10 instances at each of three sizes; observed {counts}")
    return selected, metadata


def _load_verified_backbone(
    instance,
    instance_path: Path,
    cache_entry: dict[str, Any],
    config,
) -> tuple[dict[int, list[int]], str]:
    if cache_entry.get("source_instance_sha256") != _sha256_file(instance_path):
        raise RuntimeError(f"Backbone instance checksum does not match {instance.name}")
    raw_routes = cache_entry.get("truck_routes")
    if not isinstance(raw_routes, dict):
        raise RuntimeError(f"Backbone routes are missing for {instance.name}")
    routes = {
        int(truck_id): [int(node) for node in route]
        for truck_id, route in raw_routes.items()
    }
    legacy_hash = _sha256_bytes(json.dumps(raw_routes, sort_keys=True).encode("utf-8"))
    if cache_entry.get("truck_routes_sha256") != legacy_hash:
        raise RuntimeError(f"Persisted source-backbone hash mismatch for {instance.name}")
    route_hash = _truck_routes_sha256(routes)
    check = run_nils(
        instance,
        seed=config.heuristics.random_seed,
        max_iter=0,
        max_no_improve=max(1, config.heuristics.max_no_improve),
        time_limit=0,
        max_displacement_candidates=config.heuristics.max_displacement_candidates,
        max_repair_candidates=config.heuristics.max_repair_candidates,
        initial_truck_routes=routes,
        require_feasible_incumbent=False,
    )
    if check.truck_routes != routes or not is_feasible(instance, check):
        raise RuntimeError(f"Common initial backbone is not unchanged and feasible for {instance.name}")
    return routes, route_hash


def _rendezvous_associations(solution, initial_truck_routes: dict[int, list[int]] | None = None) -> list[dict[str, Any]]:
    source_truck_by_customer = {
        int(node): int(truck_id)
        for truck_id, route in (initial_truck_routes or {}).items()
        for node in route
        if int(node) != 0
    }
    return [
        {
            **association,
            "source_truck_from_initial_backbone": source_truck_by_customer.get(association["customer_id"]),
        }
        for association in sortie_associations(solution)
    ]


def _cross_truck_recoveries(solution) -> int:
    return count_cross_truck_recoveries(solution)


def _high_priority_tardiness(instance, solution) -> float:
    return float(
        sum(
            max(0.0, float(solution.tardiness.get(customer.node_id, 0.0)))
            for customer in instance.customers
            if customer.priority == "high"
        )
    )


def _summary_rows(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[int, str], list[dict[str, str]]] = {}
    for row in rows:
        if row.get("status") != "completed":
            continue
        grouped.setdefault((int(row["size"]), row["method"]), []).append(row)
    output = []
    for (size, method), group in sorted(grouped.items()):
        feasible = [row for row in group if row.get("feasible", "").lower() == "true"]
        def values(items, field):
            return [float(row[field]) for row in items if row.get(field, "") not in {"", "nan", "None"}]
        objectives = values(feasible, "objective")
        hp_tardiness = values(feasible, "high_priority_tardiness_minutes")
        drone_share = values(feasible, "drone_service_share")
        recoveries = values(group, "cross_truck_recoveries")
        iterations = values(group, "iterations")
        cpu = values(group, "cpu_seconds")
        output.append(
            {
                "size": size,
                "method": method,
                "runs": len(group),
                "feasible_runs": len(feasible),
                "feasibility_rate": len(feasible) / max(1, len(group)),
                "mean_objective_feasible": statistics.mean(objectives) if objectives else "",
                "median_objective_feasible": statistics.median(objectives) if objectives else "",
                "mean_high_priority_tardiness_feasible": statistics.mean(hp_tardiness) if hp_tardiness else "",
                "mean_drone_service_share_feasible": statistics.mean(drone_share) if drone_share else "",
                "mean_cross_truck_recoveries": statistics.mean(recoveries) if recoveries else "",
                "mean_iterations": statistics.mean(iterations) if iterations else "",
                "mean_cpu_seconds": statistics.mean(cpu) if cpu else "",
            }
        )
    return output


def _update_outputs(output_dir: Path, rows: list[dict[str, str]]) -> None:
    _write_csv(output_dir / "matched_architecture_diagnostic_runs.csv", rows, RUN_FIELDS)
    _write_csv(output_dir / "matched_architecture_diagnostic_by_size.csv", _summary_rows(rows), SUMMARY_FIELDS)
    verification = []
    by_instance: dict[str, dict[str, str]] = {}
    for row in rows:
        if row.get("status") == "completed" and row.get("method") in {"paired_baseline", "no_unpairing", "nils"}:
            by_instance.setdefault(row["instance_id"], {})[row["method"]] = row.get("initial_truck_backbone_sha256", "")
    for instance_id, hashes in sorted(by_instance.items()):
        unique = {value for value in hashes.values() if value}
        verification.append(
            {
                "instance_id": instance_id,
                "M3_backbone_sha256": hashes.get("paired_baseline", ""),
                "M4_backbone_sha256": hashes.get("no_unpairing", ""),
                "M5_backbone_sha256": hashes.get("nils", ""),
                "identical_M3_M4_M5": len(hashes) == 3 and len(unique) == 1,
                "all_NILS_starts_reused": all(
                    row.get("initial_backbone_reused", "").lower() == "true"
                    for row in rows
                    if row.get("instance_id") == instance_id
                    and row.get("method") in {"paired_baseline", "no_unpairing", "nils"}
                    and row.get("status") == "completed"
                ),
            }
        )
    _write_csv(
        output_dir / "initial_backbone_verification.csv",
        verification,
        ["instance_id", "M3_backbone_sha256", "M4_backbone_sha256", "M5_backbone_sha256", "identical_M3_M4_M5", "all_NILS_starts_reused"],
    )


def run_targeted_replays(
    config_path: Path,
    paper_run_dir: Path,
    output_dir: Path,
    backbone_source_path: Path | None = None,
) -> None:
    config = load_and_build_config(config_path)
    backbone_path = backbone_source_path or output_dir / "initial_truck_backbones.json"
    backbone_data = json.loads(backbone_path.read_text(encoding="utf-8"))
    raw_dir = paper_run_dir / "raw"
    case_methods = [
        ("S0045_R02", "paired_baseline"),
        ("S0037_R01", "no_unpairing"),
        ("S0053_R01", "no_unpairing"),
    ]
    audit_path = output_dir / "targeted_feasibility_replay.json"
    audit = {
        "replay_started_at_utc": datetime.now(timezone.utc).isoformat(),
        "replay_execution_commit": _git_value("git", "rev-parse", "HEAD"),
        "note": "These are diagnostic replays, not replacements for the completed matched-run rows; wall-clock cutoffs can change search trajectories.",
        "cases": [],
    }
    for index, (instance_id, method) in enumerate(case_methods, start=1):
        instance_path = raw_dir / f"{instance_id}.json"
        instance = load_instance_json(instance_path)
        entry = backbone_data["instances"][instance_id]
        initial_routes = {
            int(truck_id): [int(node) for node in route]
            for truck_id, route in entry["truck_routes"].items()
        }
        scenario = {
            "n": instance.num_customers,
            "eligible_share": float(instance.metadata.get("eligible_share_target", 0.25)),
            "drones_available": instance.num_drones,
            "endurance": str(instance.metadata.get("endurance_level", "medium")),
            "speed_ratio": float(instance.metadata.get("speed_ratio", 1.25)),
            "handling_time": str(instance.metadata.get("handling_time_class", "short")),
            "spatial_pattern": str(instance.metadata.get("spatial_pattern", "uniform")),
            "priority_weight_profile": str(instance.metadata.get("priority_weight_profile", "base")),
            "battery_slack_ratio": float(instance.metadata.get("battery_slack_ratio", 0.10)),
        }
        scenario_config = _scenario_config(config, scenario)
        started = time.perf_counter()
        solution = _run_method(
            instance,
            scenario_config,
            method,
            initial_truck_routes=initial_routes if method in {"paired_baseline", "no_unpairing", "nils"} else None,
        )
        cpu_seconds = time.perf_counter() - started
        feasible = bool(is_feasible(instance, solution))
        findings = validate_solution(instance, solution)
        architecture = {"paired_baseline": "M3", "no_unpairing": "M4"}.get(method, "M5")
        pairing = default_drone_to_truck(instance) if architecture == "M3" else None
        architecture_findings = validate_drone_architecture(instance, solution, architecture, pairing)
        record = {
            "instance_id": instance_id,
            "method": method,
            "size": instance.num_customers,
            "objective": float(solution.objective),
            "feasible": feasible,
            "architecture": architecture,
            "architecture_valid": not architecture_findings,
            "architecture_violations": architecture_findings,
            "status": str(solution.status),
            "iterations": float(solution.components.get("iterations", 0.0)),
            "cpu_seconds": cpu_seconds,
            "initial_backbone_sha256": _truck_routes_sha256(initial_routes),
            "initial_backbone_reused": bool(solution.components.get("initial_backbone_reused", 0.0) >= 0.5),
            "validation_findings": [
                {
                    "code": item.code,
                    "severity": item.severity,
                    "message": item.message,
                    "vehicle": item.vehicle,
                    "node": item.node,
                }
                for item in findings
            ],
            "truck_routes": {str(key): value for key, value in solution.truck_routes.items()},
            "drone_routes": {str(key): value for key, value in solution.drone_routes.items()},
            "z1_launch_or_reload_events": [
                {"node": node, "drone": drone, "truck": truck, "value": value}
                for (node, drone, truck), value in solution.z1.items()
                if value > 0.5
            ],
            "z2_recovery_or_swap_events": [
                {"node": node, "drone": drone, "truck": truck, "value": value}
                for (node, drone, truck), value in solution.z2.items()
                if value > 0.5
            ],
            "rendezvous_associations": _rendezvous_associations(solution, initial_routes),
            "cross_truck_recoveries": _cross_truck_recoveries(solution),
        }
        audit["cases"].append(record)
        audit["updated_at_utc"] = datetime.now(timezone.utc).isoformat()
        _atomic_json(audit_path, audit)
        print(
            f"replay {index}/{len(case_methods)} {instance_id} {METHOD_LABELS[method]} "
            f"feasible={feasible} cpu={cpu_seconds:.1f}s",
            flush=True,
        )
    audit["replay_finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    _atomic_json(audit_path, audit)


def run_diagnostic(config_path: Path, source_dir: Path, paper_run_dir: Path, output_dir: Path) -> None:
    config = load_and_build_config(config_path)
    if config.heuristics.max_displacement_candidates != 100 or config.heuristics.max_repair_candidates != 100:
        raise RuntimeError("The matched diagnostic requires both configured NILS candidate caps to equal 100")
    selected, _ = _selected_instances(source_dir)
    backbone_source_path = source_dir / "initial_truck_backbones.json"
    backbone_source = json.loads(backbone_source_path.read_text(encoding="utf-8"))
    paper_raw_dir = paper_run_dir / "raw"
    output_dir.mkdir(parents=True, exist_ok=True)
    results_path = output_dir / "matched_architecture_diagnostic_runs.csv"
    provenance_path = output_dir / "run_provenance.json"
    results = _read_csv(results_path) if results_path.exists() else []
    results = [row for row in results if row.get("status") != "error"]
    if results_path.exists():
        _write_csv(results_path, results, RUN_FIELDS)
    completed = {
        (row.get("instance_id", ""), row.get("method", ""))
        for row in results
        if row.get("status") == "completed"
    }
    error_keys = {
        (row.get("instance_id", ""), row.get("method", ""))
        for row in results
        if row.get("status") == "error"
    }
    prior = json.loads(provenance_path.read_text(encoding="utf-8")) if provenance_path.exists() else {}
    current_signature = {
        "config_sha256": _sha256_file(config_path),
        "source_diagnostic_runs_sha256": _sha256_file(source_dir / "nils_timeout_diagnostic_runs.csv"),
        "source_backbone_cache_sha256": _sha256_file(backbone_source_path),
        "selected_instance_ids": [row["instance_id"] for row in selected],
        "methods": list(METHODS),
        "architecture_validation_version": 2,
    }
    for key, value in current_signature.items():
        if prior and prior.get(key) != value:
            raise RuntimeError(f"Cannot resume diagnostic: provenance mismatch for {key}")

    target_backbones: dict[str, Any] = {"schema_version": 1, "instances": {}}
    for baseline in selected:
        instance_id = baseline["instance_id"]
        instance_path = paper_raw_dir / f"{instance_id}.json"
        instance = load_instance_json(instance_path)
        cache_entry = backbone_source.get("instances", {}).get(instance_id)
        if cache_entry is None:
            raise RuntimeError(f"No persisted initial backbone exists for {instance_id}")
        routes, route_hash = _load_verified_backbone(instance, instance_path, cache_entry, config)
        entry = dict(cache_entry)
        entry["truck_routes"] = {
            str(truck_id): route for truck_id, route in sorted(routes.items())
        }
        entry["truck_routes_sha256"] = route_hash
        entry["source_backbone_cache_sha256"] = cache_entry.get("truck_routes_sha256", "")
        entry["verified_feasible"] = True
        target_backbones["instances"][instance_id] = entry
    _atomic_json(output_dir / "initial_truck_backbones.json", target_backbones)

    metadata = {
        **current_signature,
        "status": "running",
        "started_at_utc": prior.get("started_at_utc", datetime.now(timezone.utc).isoformat()),
        "updated_at_utc": datetime.now(timezone.utc).isoformat(),
        "execution_commit": _git_value("git", "rev-parse", "HEAD"),
        "worktree_status": _git_value("git", "status", "--short"),
        "python_executable": sys.executable,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "source_paper_run_dir": str(paper_run_dir),
        "source_paper_run_provenance_sha256": _sha256_file(paper_run_dir / "run_provenance.json"),
        "source_initial_backbones": str(backbone_source_path),
        "source_initial_backbones_verified_feasible": len(target_backbones["instances"]) == 30,
        "common_nils_seed": int(config.heuristics.random_seed),
        "nils_max_outer_iterations": int(config.heuristics.max_outer_iter),
        "nils_max_no_improve": int(config.heuristics.max_no_improve),
        "nils_time_limit_seconds": int(config.heuristics.time_limit_seconds),
        "max_displacement_candidates": int(config.heuristics.max_displacement_candidates),
        "max_repair_candidates": int(config.heuristics.max_repair_candidates),
        "alns_seed": int(config.seed + 211),
        "alns_max_iterations": int(dict(config.experiment or {}).get("alns_max_iter", 40)),
        "alns_time_limit_seconds": float(dict(config.experiment or {}).get("alns_time_limit_seconds", 120)),
        "alns_implementation_modified": False,
        "search_operators_modified": False,
        "objective_or_feasibility_modified": False,
        "matched_instance_ids": [row["instance_id"] for row in selected],
        "completed_method_runs": len(completed),
        "failed_method_runs": len(error_keys),
    }
    _atomic_json(provenance_path, metadata)

    for instance_index, baseline in enumerate(selected, start=1):
        instance_id = baseline["instance_id"]
        instance_path = paper_raw_dir / f"{instance_id}.json"
        instance = load_instance_json(instance_path)
        entry = target_backbones["instances"][instance_id]
        routes = {int(truck_id): [int(node) for node in route] for truck_id, route in entry["truck_routes"].items()}
        route_hash = entry["truck_routes_sha256"]
        scenario = {
            "scenario_id": baseline["scenario_id"],
            "n": int(baseline["size"]),
            "eligible_share": float(baseline["scenario_class"].split("_p", 1)[1].split("_", 1)[0]),
            "drones_available": int(instance.num_drones),
            "endurance": str(instance.metadata.get("endurance_level", "medium")),
            "speed_ratio": float(instance.metadata.get("speed_ratio", 1.25)),
            "spatial_pattern": str(instance.metadata.get("spatial_pattern", "uniform")),
            "handling_time": str(instance.metadata.get("handling_time_class", "short")),
            "priority_weight_profile": str(instance.metadata.get("priority_weight_profile", "base")),
            "battery_slack_ratio": float(instance.metadata.get("battery_slack_ratio", 0.10)),
        }
        scenario_config = _scenario_config(config, scenario)
        for method in METHODS:
            key = (instance_id, method)
            if key in completed:
                continue
            started = time.perf_counter()
            method_backbone = routes if method in {"paired_baseline", "no_unpairing", "nils"} else None
            try:
                solution = _run_method(
                    instance,
                    scenario_config,
                    method,
                    initial_truck_routes=method_backbone,
                )
                cpu_seconds = time.perf_counter() - started
                feasible = bool(is_feasible(instance, solution))
                drone_customers = solution.served_by_drone()
                architecture = {
                    "paired_baseline": "M3",
                    "no_unpairing": "M4",
                    "nils": "M5",
                }.get(method)
                architecture_findings = (
                    validate_drone_architecture(
                        instance,
                        solution,
                        architecture,
                        default_drone_to_truck(instance) if architecture == "M3" else None,
                    )
                    if architecture
                    else []
                )
                row = {
                    "instance_id": instance_id,
                    "scenario_id": baseline["scenario_id"],
                    "size": int(instance.num_customers),
                    "method": method,
                    "method_label": METHOD_LABELS[method],
                    "instance_seed": int(baseline["instance_seed"]),
                    "method_seed": int(config.seed + 211 if method == "feasibility_aware_alns" else config.heuristics.random_seed),
                    "objective": float(solution.objective),
                    "feasible": feasible,
                    "high_priority_tardiness_minutes": _high_priority_tardiness(instance, solution),
                    "drone_served_customers": len(drone_customers),
                    "drone_service_share": len(drone_customers) / max(1, instance.num_customers),
                    "cross_truck_recoveries": _cross_truck_recoveries(solution),
                    "architecture": architecture or "",
                    "architecture_valid": not architecture_findings if architecture else "",
                    "architecture_violations": json.dumps(architecture_findings, separators=(",", ":")),
                    "iterations": float(solution.components.get("iterations", 0.0)),
                    "cpu_seconds": float(cpu_seconds),
                    "runtime_limit_seconds": float(
                        dict(scenario_config.experiment or {}).get("alns_time_limit_seconds", 120)
                        if method == "feasibility_aware_alns"
                        else scenario_config.heuristics.time_limit_seconds
                    ),
                    "max_iterations": int(
                        dict(scenario_config.experiment or {}).get("alns_max_iter", 40)
                        if method == "feasibility_aware_alns"
                        else scenario_config.heuristics.max_outer_iter
                    ),
                    "max_displacement_candidates": int(scenario_config.heuristics.max_displacement_candidates)
                    if method != "feasibility_aware_alns" else "",
                    "max_repair_candidates": int(scenario_config.heuristics.max_repair_candidates)
                    if method != "feasibility_aware_alns" else "",
                    "initial_truck_backbone_sha256": route_hash if method_backbone is not None else "",
                    "initial_backbone_reused": bool(
                        method_backbone is not None
                        and solution.components.get("initial_backbone_reused", 0.0) >= 0.5
                    ),
                    "status": "completed",
                    "error": "",
                }
                completed.add(key)
                error_keys.discard(key)
            except Exception as error:
                cpu_seconds = time.perf_counter() - started
                row = {
                    "instance_id": instance_id,
                    "scenario_id": baseline["scenario_id"],
                    "size": int(instance.num_customers),
                    "method": method,
                    "method_label": METHOD_LABELS[method],
                    "instance_seed": int(baseline["instance_seed"]),
                    "method_seed": int(config.seed + 211 if method == "feasibility_aware_alns" else config.heuristics.random_seed),
                    "objective": "",
                    "feasible": "",
                    "high_priority_tardiness_minutes": "",
                    "drone_served_customers": "",
                    "drone_service_share": "",
                    "cross_truck_recoveries": "",
                    "iterations": "",
                    "cpu_seconds": float(cpu_seconds),
                    "runtime_limit_seconds": "",
                    "max_iterations": "",
                    "max_displacement_candidates": int(config.heuristics.max_displacement_candidates),
                    "max_repair_candidates": int(config.heuristics.max_repair_candidates),
                    "initial_truck_backbone_sha256": route_hash if method_backbone is not None else "",
                    "initial_backbone_reused": False,
                    "status": "error",
                    "error": f"{type(error).__name__}: {error}",
                }
                error_keys.add(key)
            _append_csv(results_path, row, RUN_FIELDS)
            results.append({key: str(value) for key, value in row.items()})
            _update_outputs(output_dir, results)
            metadata["completed_method_runs"] = len(completed)
            metadata["failed_method_runs"] = len(error_keys)
            metadata["updated_at_utc"] = datetime.now(timezone.utc).isoformat()
            metadata["last_instance_id"] = instance_id
            metadata["last_method"] = method
            _atomic_json(provenance_path, metadata)
            print(
                f"[{instance_index}/{len(selected)}] {instance_id} {METHOD_LABELS[method]} "
                f"{row['status']} feasible={row.get('feasible', '')} cpu={cpu_seconds:.1f}s "
                f"obj={row.get('objective', '')}",
                flush=True,
            )

    metadata["status"] = "complete" if not error_keys else "complete_with_errors"
    metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    _atomic_json(provenance_path, metadata)


def main() -> None:
    parser = argparse.ArgumentParser(description="Matched M3/M4/M5/ALNS diagnostic with shared NILS starts")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source-diagnostic-dir", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--paper-run-dir", type=Path, default=DEFAULT_PAPER_RUN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--audit-replay", action="store_true")
    parser.add_argument("--backbone-source", type=Path)
    args = parser.parse_args()
    if args.audit_replay:
        run_targeted_replays(args.config, args.paper_run_dir, args.output_dir, args.backbone_source)
    else:
        run_diagnostic(args.config, args.source_diagnostic_dir, args.paper_run_dir, args.output_dir)


if __name__ == "__main__":
    main()
