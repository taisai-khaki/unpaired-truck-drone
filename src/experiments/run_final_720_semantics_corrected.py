from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
import traceback
from collections import Counter, defaultdict
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import ortools
import pandas as pd
import scipy
import yaml

from ..drone_state import route_intersection_events
from ..feasibility import validate_solution
from ..heuristics.architecture import validate_drone_architecture
from ..heuristics.nils import default_drone_to_truck, evaluate_solution
from ..instance_generator import load_instance_json
from ..parameters import build_search_config
from ..timing import active_rendezvous_events, compute_synchronization_waiting
from .heuristic_study import _collect_row, _run_method, _scenario_config, _truck_routes_sha256
from .run_event_semantics_diagnostic import _sortie_rows, _support_trace


REPO = Path(__file__).resolve().parents[2]
VALIDATED_GATE_COMMIT = "19762f7097564011cf968519e62f2cbeb5fc7035"
DIAGNOSTIC_HEAD = "3953d4f7ffd12ed863e03e06749bc2887b9097c1"
SOURCE_OUTPUT = REPO / "outputs/full_m3_m4_m5_alns_720_frozen_20261106"
SOURCE_RAW = SOURCE_OUTPUT / "raw"
SOURCE_BACKBONES = SOURCE_RAW / "initial_truck_backbones"
SOURCE_CONFIG = SOURCE_OUTPUT / "run_config.yaml"
REFERENCE_AUDIT = (
    REPO
    / "outputs/manuscript_semantics_reconciliation_20261112/truck_only_720/"
    / "truck_only_720_backbone_audit.csv"
)
SEMANTICS_MANIFEST = REPO / "outputs/manuscript_semantics_reconciliation_20261112/run_manifest.json"
OUTPUT = REPO / "outputs/final_m3_m4_m5_alns_720_semantics_corrected_20261112"
METHODS = ["paired_baseline", "no_unpairing", "nils", "feasibility_aware_alns"]
COMMON_METHODS = set(METHODS[:3])
ARCHITECTURES = {
    "paired_baseline": "M3",
    "no_unpairing": "M4",
    "nils": "M5",
    "feasibility_aware_alns": "M5",
}
SORTIE_TYPES = ["depot-depot", "depot-truck", "truck-depot", "same-truck", "cross-truck"]
EXPECTED_RUNS = 720 * len(METHODS)
RESULT_FILE = OUTPUT / "tables/raw_720_method_results.csv"
CHECKPOINT_FILE = OUTPUT / "checkpoints/heuristic_study_rows.jsonl"
RUN_RECORDS = OUTPUT / "run_records"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _python_environment() -> dict[str, str]:
    return {
        "executable": str(Path(sys.executable).resolve()),
        "version": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "pandas": pd.__version__,
        "scipy": scipy.__version__,
        "ortools": ortools.__version__,
        "pyyaml": yaml.__version__,
    }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_routes(routes: dict[int, list[int]]) -> dict[str, list[int]]:
    return {
        str(int(truck_id)): [int(node) for node in route]
        for truck_id, route in sorted(routes.items(), key=lambda item: int(item[0]))
    }


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "__dict__"):
        return _json_safe(vars(value))
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(_json_safe(payload), indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _git(command: list[str], *, check: bool = True) -> str:
    result = subprocess.run(
        ["git", *command], cwd=REPO, check=check, capture_output=True, text=True
    )
    return result.stdout.strip()


def _assert_frozen_settings(data: dict[str, Any]) -> None:
    heuristics = data.get("heuristics", {}).get("nils", {})
    experiment = data.get("experiment", {})
    required = {
        "base seed": (int(data.get("seed", -1)), 2026),
        "NILS seed": (int(heuristics.get("random_seed", -1)), 2026),
        "NILS max iterations": (int(heuristics.get("max_outer_iter", -1)), 25),
        "NILS no-improve limit": (int(heuristics.get("max_no_improve", -1)), 8),
        "NILS time limit": (int(heuristics.get("time_limit_seconds", -1)), 120),
        "NILS displacement cap": (int(heuristics.get("max_displacement_candidates", -1)), 100),
        "NILS repair cap": (int(heuristics.get("max_repair_candidates", -1)), 100),
        "ALNS max iterations": (int(experiment.get("alns_max_iter", -1)), 40),
        "ALNS time limit": (int(experiment.get("alns_time_limit_seconds", -1)), 120),
        "ALNS max positions": (int(experiment.get("alns_max_positions_per_route", -1)), 3),
        "ALNS max destroy": (int(experiment.get("alns_max_destroy", -1)), 5),
    }
    mismatches = [
        f"{key}: expected {expected}, found {actual}"
        for key, (actual, expected) in required.items()
        if actual != expected
    ]
    for key in ("battery_slack_ratio", "alns_battery_slack_ratio"):
        actual = float(experiment.get(key, float("nan")))
        if not math.isclose(actual, 0.10, rel_tol=0.0, abs_tol=1e-12):
            mismatches.append(f"{key}: expected 0.10, found {actual}")
    if experiment.get("study_methods") != METHODS:
        mismatches.append(f"method list must be exactly {METHODS}")
    if mismatches:
        raise RuntimeError("Frozen run settings do not match:\n" + "\n".join(mismatches))


def _make_effective_config(source_data: dict[str, Any]) -> dict[str, Any]:
    data = deepcopy(source_data)
    _assert_frozen_settings(data)
    experiment = data.setdefault("experiment", {})
    experiment["output_dir"] = OUTPUT.relative_to(REPO).as_posix()
    experiment["study_force_feasible_outputs"] = False
    experiment["study_resume"] = False
    experiment["study_methods"] = list(METHODS)
    return data


def _source_semantics_hashes() -> dict[str, str]:
    prior = json.loads(SEMANTICS_MANIFEST.read_text(encoding="utf-8"))
    prior_hashes = {
        entry["path"]: entry["sha256"]
        for entry in prior["source_and_input_file_hashes"]
        if entry.get("path", "").startswith("src/")
    }
    paths = [
        "src/heuristics/alns.py",
        "src/drone_state.py",
        "src/timing.py",
        "src/feasibility.py",
        "src/heuristics/architecture.py",
        "src/heuristics/nils.py",
    ]
    current = {path: _sha256_file(REPO / path) for path in paths}
    mismatches = [path for path in paths if prior_hashes.get(path) != current[path]]
    if mismatches:
        raise RuntimeError(
            "Frozen semantic implementation differs from the passed targeted gate: "
            + ", ".join(mismatches)
        )
    return current


def _parse_reference_rows() -> dict[str, dict[str, Any]]:
    with REFERENCE_AUDIT.open("r", newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    result = {str(row["instance_id"]): row for row in rows}
    if len(rows) != 720 or len(result) != 720:
        raise RuntimeError(f"Expected 720 unique truck-only reference rows; found {len(result)}")
    return result


def _collect_input_manifest() -> list[dict[str, Any]]:
    input_files = sorted(SOURCE_RAW.glob("*.json"))
    backbone_files = sorted(SOURCE_BACKBONES.glob("*.json"))
    if len(input_files) != 720 or len(backbone_files) != 720:
        raise RuntimeError(
            f"Expected exactly 720 frozen instances and backbones; got "
            f"{len(input_files)} instances and {len(backbone_files)} backbones"
        )
    reference = _parse_reference_rows()
    input_ids = {path.stem for path in input_files}
    if input_ids != {path.stem for path in backbone_files} or input_ids != set(reference):
        raise RuntimeError("Frozen inputs, persisted backbones, and matched reference IDs differ")

    rows: list[dict[str, Any]] = []
    for instance_path in input_files:
        raw = json.loads(instance_path.read_text(encoding="utf-8"))
        instance_id = str(raw["name"])
        if instance_id != instance_path.stem:
            raise RuntimeError(f"Instance name mismatch in {instance_path}")
        match = re.fullmatch(r"(.+)_R(\d+)", instance_id)
        if match is None:
            raise RuntimeError(f"Cannot extract replication ID from {instance_id}")
        metadata = raw.get("metadata", {})
        if not metadata.get("scenario_id"):
            raise RuntimeError(f"Missing scenario ID in {instance_id}")
        instance_sha = _sha256_file(instance_path)
        backbone_path = SOURCE_BACKBONES / f"{instance_id}.json"
        backbone_sha = _sha256_file(backbone_path)
        backbone = json.loads(backbone_path.read_text(encoding="utf-8"))
        routes = {
            int(truck_id): [int(node) for node in route]
            for truck_id, route in backbone["truck_routes"].items()
        }
        route_sha = _truck_routes_sha256(routes)
        ref = reference[instance_id]
        checks = {
            "backbone instance hash": (backbone.get("source_instance_sha256"), instance_sha),
            "reference backbone file hash": (ref.get("backbone_file_sha256"), backbone_sha),
            "reference route hash": (ref.get("backbone_routes_sha256"), route_sha),
            "reference instance hash": (ref.get("instance_file_sha256"), instance_sha),
        }
        for label, (actual, expected) in checks.items():
            if actual != expected:
                raise RuntimeError(f"{label} mismatch for {instance_id}: {actual} != {expected}")
        if backbone.get("initialization_feasible") is not True or ref.get("feasible", "").lower() != "true":
            raise RuntimeError(f"Persisted backbone/reference is not marked feasible for {instance_id}")
        objective = float(ref["objective"])
        if not math.isfinite(objective):
            raise RuntimeError(f"Non-finite truck-only reference objective for {instance_id}")
        rows.append(
            {
                "instance_id": instance_id,
                "instance_sha256": instance_sha,
                "n": len(raw["customers"]),
                "scenario_id": str(metadata["scenario_id"]),
                "replication_id": int(match.group(2)),
                "original_instance_seed": int(raw["seed"]),
                "eligible_share_target": float(metadata.get("eligible_share_target", float("nan"))),
                "eligible_share_realized": float(metadata.get("eligible_share_realized", float("nan"))),
                "number_of_drones": len(raw["drones"]),
                "endurance_level": str(metadata.get("endurance_level", "")),
                "speed_ratio": float(metadata.get("speed_ratio", float("nan"))),
                "handling_time_class": str(metadata.get("handling_time_class", "")),
                "spatial_pattern": str(metadata.get("spatial_pattern", "")),
                "priority_weight_profile": str(metadata.get("priority_weight_profile", "")),
                "battery_slack_ratio": float(metadata.get("battery_slack_ratio", float("nan"))),
                "backbone_file": backbone_path.relative_to(REPO).as_posix(),
                "backbone_file_sha256": backbone_sha,
                "backbone_routes_sha256": route_sha,
                "backbone_routes_json": json.dumps(
                    _canonical_routes(routes), sort_keys=True, separators=(",", ":")
                ),
                "matched_truck_only_objective": objective,
                "matched_truck_only_feasible": True,
            }
        )
    return rows


def _run_pytest() -> dict[str, Any]:
    started = time.monotonic()
    result = subprocess.run(
        [sys.executable, "-m", "pytest", "-q"],
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    elapsed = time.monotonic() - started
    output = (result.stdout or "") + (result.stderr or "")
    passed = re.search(r"(\d+)\s+passed", output)
    failed = re.search(r"(\d+)\s+failed", output)
    skipped = re.search(r"(\d+)\s+skipped", output)
    evidence = {
        "command": f"{Path(sys.executable).name} -m pytest -q",
        "return_code": result.returncode,
        "passed": int(passed.group(1)) if passed else 0,
        "failed": int(failed.group(1)) if failed else 0,
        "skipped": int(skipped.group(1)) if skipped else 0,
        "elapsed_seconds": elapsed,
        "output": output[-12000:],
    }
    if result.returncode != 0 or evidence["failed"] != 0 or evidence["passed"] == 0:
        raise RuntimeError(f"Test suite preflight failed:\n{output[-6000:]}")
    return evidence


def _git_preflight() -> dict[str, str]:
    head = _git(["rev-parse", "HEAD"])
    if head == DIAGNOSTIC_HEAD:
        raise RuntimeError("Commit the runner-integrity changes before recording provenance")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", VALIDATED_GATE_COMMIT, head],
        cwd=REPO,
        capture_output=True,
    )
    if ancestor.returncode != 0:
        raise RuntimeError("Execution commit does not descend from the validated semantic gate")
    changed = _git(["status", "--porcelain", "--untracked-files=no"])
    if changed:
        raise RuntimeError(f"Tracked working tree must be clean before preflight:\n{changed}")
    upstream = _git(["rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"], check=False)
    return {
        "execution_commit": head,
        "branch": _git(["branch", "--show-current"]),
        "upstream": upstream,
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(key for row in rows for key in row)) if rows else []
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        if fields:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def preflight() -> None:
    if OUTPUT.exists():
        raise RuntimeError(f"Final output directory already exists; refusing to inherit data: {OUTPUT}")
    git_info = _git_preflight()
    semantic_hashes = _source_semantics_hashes()
    prior = json.loads(SEMANTICS_MANIFEST.read_text(encoding="utf-8"))
    gate = prior.get("targeted_gate", {})
    if not gate.get("gate_passed"):
        raise RuntimeError("Previously required targeted semantic gate is not PASS")
    truck_audit = prior.get("truck_only_audit", {})
    if (
        truck_audit.get("status") != "PASS_REFERENCE_AUDIT"
        or int(truck_audit.get("instances", 0)) != 720
        or int(truck_audit.get("feasible", 0)) != 720
        or int(truck_audit.get("infeasible", -1)) != 0
    ):
        raise RuntimeError("Validated matched truck-only reference audit is not 720/720 PASS")
    test_evidence = _run_pytest()
    original_config = yaml.safe_load(SOURCE_CONFIG.read_text(encoding="utf-8"))
    effective_config = _make_effective_config(original_config)
    if effective_config["experiment"].get("study_force_feasible_outputs") is not False:
        raise RuntimeError("Study-level post-method repair/fallback was not disabled")
    input_rows = _collect_input_manifest()
    if len(input_rows) != 720 or len({row["instance_id"] for row in input_rows}) != 720:
        raise RuntimeError("Frozen input manifest is not exactly 720 unique instances")

    OUTPUT.mkdir(parents=True, exist_ok=False)
    for directory in (
        RUN_RECORDS,
        OUTPUT / "tables",
        OUTPUT / "checkpoints",
        OUTPUT / "logs",
        OUTPUT / "provenance",
    ):
        directory.mkdir(parents=True, exist_ok=True)
    config_path = OUTPUT / "run_config.yaml"
    config_path.write_text(
        yaml.safe_dump(effective_config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    input_path = OUTPUT / "input_manifest.csv"
    _write_csv(input_path, input_rows)
    CHECKPOINT_FILE.write_text("", encoding="utf-8")
    hashes = {
        "input_manifest_sha256": _sha256_file(input_path),
        "source_config_sha256": _sha256_file(SOURCE_CONFIG),
        "effective_config_sha256": _sha256_file(config_path),
        "semantic_source_hashes": semantic_hashes,
        "all_src_python_sha256": {
            path.relative_to(REPO).as_posix(): _sha256_file(path)
            for path in sorted((REPO / "src").rglob("*.py"))
        },
    }
    preflight_report = {
        "status": "PASS",
        "execution": git_info,
        "python_environment": _python_environment(),
        "validated_gate_commit": VALIDATED_GATE_COMMIT,
        "prior_diagnostic_head": DIAGNOSTIC_HEAD,
        "created_at_utc": _utc_now(),
        "test_suite": test_evidence,
        "targeted_semantic_gate": gate,
        "matched_truck_only_audit": truck_audit,
        "input_instances": 720,
        "persisted_backbones": 720,
        "matched_truck_only_rows": 720,
        "methods": METHODS,
        "expected_method_instance_runs": EXPECTED_RUNS,
        "new_checkpoint_rows": 0,
        "historical_method_rows_resumed": 0,
        "post_method_repair_enabled": False,
        "post_method_fallback_enabled": False,
        "legacy_study_resume_enabled": False,
        "dedicated_final_run_resume_enabled": True,
        "shared_initial_backbone_methods": sorted(COMMON_METHODS),
        "alns_uses_shared_initial_backbone": False,
        "settings": {
            "nils_seed": 2026,
            "nils_max_outer_iterations": 25,
            "nils_max_no_improve": 8,
            "nils_time_limit_seconds": 120,
            "max_displacement_candidates": 100,
            "max_repair_candidates": 100,
            "alns_seed": 2237,
            "alns_max_iterations": 40,
            "alns_time_limit_seconds": 120,
            "alns_max_positions_per_route": 3,
            "alns_max_destroy": 5,
            "battery_slack_ratio": 0.10,
            "runtime_clock": "elapsed wall time measured with time.monotonic; same basis as algorithm deadlines",
        },
        "hashes": hashes,
    }
    _atomic_json(OUTPUT / "preflight_audit.json", preflight_report)
    _atomic_json(
        OUTPUT / "run_manifest.json",
        {
            "dataset_status": "PREFLIGHT_PASSED",
            "created_at_utc": _utc_now(),
            "execution_commit": git_info["execution_commit"],
            "branch": git_info["branch"],
            "upstream": git_info["upstream"],
            "python_environment": _python_environment(),
            "output_directory": OUTPUT.relative_to(REPO).as_posix(),
            "configuration_sha256": hashes["effective_config_sha256"],
            "source_configuration_sha256": hashes["source_config_sha256"],
            "input_manifest_sha256": hashes["input_manifest_sha256"],
            "input_instances": 720,
            "expected_method_instance_runs": EXPECTED_RUNS,
            "methods": METHODS,
            "historical_method_rows_resumed": 0,
            "post_method_repair_enabled": False,
            "post_method_fallback_enabled": False,
            "legacy_study_resume_enabled": False,
            "dedicated_final_run_resume_enabled": True,
            "preflight_audit": "preflight_audit.json",
            "input_manifest": "input_manifest.csv",
            "effective_configuration": "run_config.yaml",
            "status": "ready_to_run",
        },
    )
    print(
        json.dumps(
            {"preflight": "PASS", "inputs": 720, "expected_runs": EXPECTED_RUNS, "output": str(OUTPUT)},
            indent=2,
        )
    )


def _read_manifest_rows() -> list[dict[str, Any]]:
    with (OUTPUT / "input_manifest.csv").open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _load_backbone(instance_row: dict[str, Any]) -> tuple[dict[int, list[int]], dict[str, Any]]:
    path = REPO / str(instance_row["backbone_file"])
    if _sha256_file(path) != instance_row["backbone_file_sha256"]:
        raise RuntimeError(f"Persisted backbone file changed after preflight: {path}")
    entry = json.loads(path.read_text(encoding="utf-8"))
    routes = {
        int(truck_id): [int(node) for node in route]
        for truck_id, route in entry["truck_routes"].items()
    }
    if _truck_routes_sha256(routes) != instance_row["backbone_routes_sha256"]:
        raise RuntimeError(f"Persisted route hash changed after preflight: {path}")
    if entry.get("source_instance_sha256") != instance_row["instance_sha256"]:
        raise RuntimeError(f"Persisted backbone is attached to another instance: {path}")
    return routes, entry


def _scenario_from_instance(instance: Any) -> dict[str, Any]:
    metadata = instance.metadata
    return {
        "scenario_id": str(metadata["scenario_id"]),
        "n": int(instance.num_customers),
        "eligible_share": float(metadata["eligible_share_target"]),
        "drones_available": int(instance.num_drones),
        "endurance": str(metadata["endurance_level"]),
        "speed_ratio": float(metadata["speed_ratio"]),
        "handling_time": str(metadata["handling_time_class"]),
        "spatial_pattern": str(metadata["spatial_pattern"]),
        "priority_weight_profile": str(metadata["priority_weight_profile"]),
        "battery_slack_ratio": float(metadata.get("battery_slack_ratio", 0.10)),
        "depot_position": str(metadata.get("depot_position", "peripheral")),
    }


def _solution_payload(solution: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    for name, value in vars(solution).items():
        if name in {"truck_routes", "drone_routes"}:
            payload[name] = {
                str(int(vehicle)): [int(node) for node in route]
                for vehicle, route in sorted(value.items(), key=lambda pair: int(pair[0]))
            }
        elif isinstance(value, dict) and any(isinstance(key, tuple) for key in value):
            payload[name] = [
                {
                    "key": _json_safe(list(key) if isinstance(key, tuple) else key),
                    "value": _json_safe(item),
                }
                for key, item in sorted(value.items(), key=lambda pair: repr(pair[0]))
            ]
        else:
            payload[name] = _json_safe(value)
    return payload


def _record_path(instance_id: str, method: str) -> Path:
    return RUN_RECORDS / f"{instance_id}__{method}.json.gz"


def _raw_return_path(instance_id: str, method: str) -> Path:
    return OUTPUT / "raw_returns" / f"{instance_id}__{method}.json.gz"


def _write_record(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with gzip.open(temporary, "wt", encoding="utf-8", newline="\n", compresslevel=6) as stream:
        json.dump(_json_safe(payload), stream, sort_keys=True, allow_nan=True)
        stream.write("\n")
    os.replace(temporary, path)


def _read_record(path: Path) -> dict[str, Any]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def _append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(_json_safe(payload), sort_keys=True, allow_nan=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def _rebuild_checkpoint_views(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows.sort(key=lambda row: (str(row["instance_id"]), METHODS.index(str(row["method_name"]))))
    _write_csv(RESULT_FILE, rows)
    temporary = CHECKPOINT_FILE.with_suffix(CHECKPOINT_FILE.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(_json_safe(row), sort_keys=True, allow_nan=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, CHECKPOINT_FILE)
    return rows


def _scan_run_records() -> tuple[list[dict[str, Any]], set[tuple[str, str]]]:
    rows: list[dict[str, Any]] = []
    keys: set[tuple[str, str]] = set()
    allowed_ids = {str(row["instance_id"]) for row in _read_manifest_rows()}
    expected_commit = _git(["rev-parse", "HEAD"])
    for path in sorted(RUN_RECORDS.glob("*.json.gz")):
        record = _read_record(path)
        row = record.get("row", {})
        key = (str(row.get("instance_id", "")), str(row.get("method_name", "")))
        if key[0] not in allowed_ids or key[1] not in METHODS:
            raise RuntimeError(f"Unexpected run artifact in final checkpoint directory: {path}")
        if key in keys:
            raise RuntimeError(f"Duplicate run record for {key}")
        if record.get("execution_commit_sha") != expected_commit:
            raise RuntimeError(f"Execution commit mismatch in run artifact: {path}")
        keys.add(key)
        rows.append(row)
    return rows, keys


def _event_diagnostics(instance: Any, solution: Any) -> dict[str, Any]:
    events = active_rendezvous_events(solution)
    intersections = route_intersection_events(solution)
    waiting = compute_synchronization_waiting(instance, solution)
    wait_by_key = {
        (int(row["node"]), int(row["drone_id"]), int(row["truck_id"])): row
        for row in waiting["events"]
    }
    rendezvous_rows: list[dict[str, Any]] = []
    drone_legs: list[dict[str, Any]] = []
    state_issues: dict[str, list[str]] = defaultdict(list)
    for drone_id in instance.drone_ids():
        legs, support_rows, issues = _support_trace(instance, solution, drone_id, events)
        drone_legs.extend(legs)
        for label, messages in issues.items():
            state_issues[label].extend(messages)
        for row in support_rows:
            key = tuple(int(part) for part in row["event_key"])
            row["u_sync_declared"] = bool(solution.u_sync.get(key, 0) > 0.5)
            wait_row = wait_by_key.get(key, {})
            row["truck_readiness_minutes"] = wait_row.get("truck_readiness")
            row["drone_readiness_minutes"] = wait_row.get("drone_readiness")
            row["truck_wait_minutes"] = wait_row.get("truck_wait")
            row["drone_wait_minutes"] = wait_row.get("drone_wait")
            row["total_wait_minutes"] = wait_row.get("total_wait")
            rendezvous_rows.append(row)
    declared = {
        (int(node), int(drone), int(truck))
        for (node, drone, truck), value in solution.u_sync.items()
        if value > 0.5
    }
    return {
        "u_sync_events": [list(event) for event in sorted(events)],
        "route_intersection_events": [list(event) for event in sorted(intersections)],
        "intersection_without_declared_u_sync": [
            list(event) for event in sorted(intersections - declared)
        ],
        "declared_u_sync_without_route_intersection": [
            list(event) for event in sorted(declared - intersections)
        ],
        "rendezvous": rendezvous_rows,
        "drone_legs": drone_legs,
        "sorties": _sortie_rows(solution, events),
        "waiting": _json_safe(waiting),
        "state_issues": dict(state_issues),
    }


def _decorate_row(
    instance: Any,
    solution: Any,
    raw_solution: Any,
    scenario: dict[str, Any],
    method: str,
    runtime: float,
    findings: list[Any],
    architecture_violations: list[str],
    diagnostics: dict[str, Any],
    instance_row: dict[str, Any],
    execution_commit: str,
) -> dict[str, Any]:
    row = _collect_row(instance, solution, scenario, method)
    error_findings = [finding for finding in findings if finding.severity == "ERROR"]
    rendezvous = diagnostics["rendezvous"]
    sorties = diagnostics["sorties"]
    waiting = diagnostics["waiting"]
    sort_counts = Counter(str(item["sortie_type"]) for item in sorties)
    declared_usync = {
        (int(node), int(drone), int(truck))
        for (node, drone, truck), value in solution.u_sync.items()
        if value > 0.5 and int(node) != 0
    }
    z1_active = {
        tuple(int(part) for part in key)
        for key, value in solution.z1.items()
        if value > 0.5 and int(key[0]) != 0
    }
    z2_active = {
        tuple(int(part) for part in key)
        for key, value in solution.z2.items()
        if value > 0.5 and int(key[0]) != 0
    }
    feasible = not error_findings
    arch_valid = not architecture_violations
    common = method in COMMON_METHODS
    row.update(
        {
            "scenario_class_id": scenario["scenario_id"],
            "replication_id": int(instance_row["replication_id"]),
            "original_instance_seed": int(instance.seed),
            "method_seed": 2026 if common or method == "nils" else 2237,
            "execution_commit_sha": execution_commit,
            "instance_sha256": instance_row["instance_sha256"],
            "initial_backbone_file_sha256": instance_row["backbone_file_sha256"] if common else "",
            "initial_backbone_routes_sha256": instance_row["backbone_routes_sha256"] if common else "",
            "initial_backbone_routes_json": instance_row["backbone_routes_json"] if common else "",
            "matched_truck_only_objective": float(instance_row["matched_truck_only_objective"]),
            "shared_initial_backbone_supplied": common,
            "shared_initial_backbone_reused": (
                common and float(solution.components.get("initial_backbone_reused", 0.0)) >= 0.5
            ),
            "alns_independent_initializer": method == "feasibility_aware_alns",
            "architecture": ARCHITECTURES[method],
            "full_feasibility": feasible,
            "feasibility_violations_json": json.dumps(
                [
                    {
                        "severity": item.severity,
                        "code": item.code,
                        "message": item.message,
                        "vehicle": item.vehicle,
                        "node": item.node,
                    }
                    for item in findings
                ],
                sort_keys=True,
            ),
            "architecture_valid": arch_valid,
            "architecture_violations_json": json.dumps(architecture_violations, sort_keys=True),
            "raw_returned_status": str(raw_solution.status),
            "raw_returned_objective": float(raw_solution.objective),
            "raw_returned_objective_components_json": json.dumps(
                _json_safe(raw_solution.components), sort_keys=True, allow_nan=True
            ),
            "recomputed_objective": float(solution.objective),
            "recomputed_objective_components_json": json.dumps(
                _json_safe(solution.components), sort_keys=True, allow_nan=True
            ),
            "objective_recompute_delta": float(solution.objective - raw_solution.objective),
            "post_method_repair_applied": False,
            "post_method_truck_only_fallback_applied": False,
            "cpu_time_seconds": float(runtime),
            "elapsed_wall_time_seconds": float(runtime),
            "algorithm_reported_runtime_seconds": float(raw_solution.run_time_seconds),
            "u_sync_count": len(declared_usync),
            "route_intersection_u_sync_count": len(diagnostics["route_intersection_events"]),
            "intersection_without_declared_u_sync_count": len(
                diagnostics["intersection_without_declared_u_sync"]
            ),
            "declared_u_sync_without_route_intersection_count": len(
                diagnostics["declared_u_sync_without_route_intersection"]
            ),
            "z1_count": len(z1_active),
            "z2_count": len(z2_active),
            "rendezvous_neither_reload_nor_swap_count": sum(
                not event["z1"] and not event["z2"] for event in rendezvous
            ),
            "synchronization_handling_minutes": sum(
                float(event.get("handling_minutes", 0.0)) for event in rendezvous
            ),
            "synchronization_waiting_minutes": float(waiting["total_waiting_minutes"]),
            "sortie_count": len(sorties),
            "depot_depot_sortie_count": int(sort_counts.get("depot-depot", 0)),
            "depot_truck_sortie_count": int(sort_counts.get("depot-truck", 0)),
            "truck_depot_sortie_count": int(sort_counts.get("truck-depot", 0)),
            "same_truck_sortie_count": int(sort_counts.get("same-truck", 0)),
            "cross_truck_sortie_count": int(sort_counts.get("cross-truck", 0)),
            "all_cross_truck_sorties_continuous": all(
                bool(item["continuous_from_previous"]) and not bool(item["ambiguous_event_truck"])
                for item in sorties
                if item["sortie_type"] == "cross-truck"
            ),
            "priority_tardiness_cost": float(solution.components.get("tardiness_cost", 0.0)),
            "truck_travel_cost": float(solution.components.get("truck_cost", 0.0)),
            "drone_travel_cost": float(solution.components.get("drone_cost", 0.0)),
        }
    )
    base_objective = float(instance_row["matched_truck_only_objective"])
    row["improvement_vs_truck_only_pct"] = (
        (base_objective - float(solution.objective)) / abs(base_objective) * 100.0
        if abs(base_objective) > 1e-12
        else float("nan")
    )
    for sortie_type in SORTIE_TYPES:
        row[f"{sortie_type.replace('-', '_')}_sortie_count"] = int(sort_counts.get(sortie_type, 0))
    row["method_status"] = "feasible" if feasible else "infeasible"
    return row


def _update_manifest(status: str, completed: int, **extra: Any) -> None:
    path = OUTPUT / "run_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest.update(
        {
            "status": status,
            "dataset_status": status,
            "completed_method_instance_runs": int(completed),
            "expected_method_instance_runs": EXPECTED_RUNS,
            "last_progress_update_utc": _utc_now(),
        }
    )
    manifest.update(_json_safe(extra))
    _atomic_json(path, manifest)


def _verify_run_preflight() -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = json.loads((OUTPUT / "run_manifest.json").read_text(encoding="utf-8"))
    audit = json.loads((OUTPUT / "preflight_audit.json").read_text(encoding="utf-8"))
    allowed = {"ready_to_run", "running", "runs_complete", "finalized"}
    if manifest.get("status") not in allowed:
        raise RuntimeError(f"Output is not authorized by a passing preflight: {manifest.get('status')}")
    if manifest.get("historical_method_rows_resumed") != 0:
        raise RuntimeError("Manifest indicates historical method rows were resumed")
    if manifest.get("legacy_study_resume_enabled") is not False:
        raise RuntimeError("Legacy study checkpoint/resume is not disabled")
    if manifest.get("post_method_repair_enabled") or manifest.get("post_method_fallback_enabled"):
        raise RuntimeError("Study-level post-method repair/fallback is enabled")
    if _git(["rev-parse", "HEAD"]) != manifest["execution_commit"]:
        raise RuntimeError("Execution commit differs from frozen preflight commit")
    if manifest.get("python_environment") != _python_environment():
        raise RuntimeError("Python/runtime dependency versions differ from preflight")
    changed = _git(["status", "--porcelain", "--untracked-files=no"])
    if changed:
        raise RuntimeError(f"Tracked code changed after preflight:\n{changed}")
    if _sha256_file(OUTPUT / "run_config.yaml") != manifest["configuration_sha256"]:
        raise RuntimeError("Effective configuration hash changed after preflight")
    if _sha256_file(OUTPUT / "input_manifest.csv") != manifest["input_manifest_sha256"]:
        raise RuntimeError("Input manifest hash changed after preflight")
    if audit.get("status") != "PASS":
        raise RuntimeError("Preflight audit does not show PASS")
    return manifest, audit


def _run_full() -> None:
    manifest, _audit = _verify_run_preflight()
    raw_config = yaml.safe_load((OUTPUT / "run_config.yaml").read_text(encoding="utf-8"))
    config = build_search_config(raw_config)
    if config.experiment.get("study_force_feasible_outputs") is not False:
        raise RuntimeError("Harness-level feasibility repair/fallback must remain disabled")
    input_rows = _read_manifest_rows()
    if len(input_rows) != 720:
        raise RuntimeError(f"Input manifest has {len(input_rows)} rows, expected 720")
    rows, completed_keys = _scan_run_records()
    if not rows and CHECKPOINT_FILE.stat().st_size:
        raise RuntimeError("Fresh final checkpoint unexpectedly contains prior result rows")
    _rebuild_checkpoint_views(rows)
    execution_commit = str(manifest["execution_commit"])
    _update_manifest("running", len(rows), started_at_utc=manifest.get("started_at_utc", _utc_now()))
    progress_log = OUTPUT / "logs/final_720_progress.log"
    with progress_log.open("a", encoding="utf-8") as stream:
        stream.write(f"[{_utc_now()}] run start/resume; complete={len(rows)}/{EXPECTED_RUNS}\n")
        stream.flush()
        os.fsync(stream.fileno())

    for instance_row in input_rows:
        instance_id = str(instance_row["instance_id"])
        instance_path = SOURCE_RAW / f"{instance_id}.json"
        if _sha256_file(instance_path) != instance_row["instance_sha256"]:
            raise RuntimeError(f"Frozen instance changed after preflight: {instance_id}")
        instance = load_instance_json(instance_path)
        scenario = _scenario_from_instance(instance)
        scenario_config = _scenario_config(config, scenario)
        shared_routes, backbone_entry = _load_backbone(instance_row)
        if int(instance_row["n"]) != instance.num_customers:
            raise RuntimeError(f"Frozen n mismatch for {instance_id}")
        if int(instance_row["original_instance_seed"]) != instance.seed:
            raise RuntimeError(f"Frozen seed mismatch for {instance_id}")

        for method in METHODS:
            key = (instance_id, method)
            if key in completed_keys:
                continue
            record_path = _record_path(instance_id, method)
            if record_path.exists():
                raise RuntimeError(f"Unindexed run artifact exists: {record_path}")
            raw_return_path = _raw_return_path(instance_id, method)
            if raw_return_path.exists():
                raise RuntimeError(f"Orphaned raw return exists; inspect before resuming: {raw_return_path}")
            method_backbone = deepcopy(shared_routes) if method in COMMON_METHODS else None
            if method in COMMON_METHODS and method_backbone != shared_routes:
                raise RuntimeError(f"Shared backbone differs before {instance_id}/{method}")
            supplied_backbone_hash = (
                _truck_routes_sha256(method_backbone) if method_backbone is not None else ""
            )
            if method == "feasibility_aware_alns" and method_backbone is not None:
                raise RuntimeError("ALNS must use its independent initializer")

            started = time.monotonic()
            wall_started = time.time()
            try:
                raw_solution = _run_method(
                    instance,
                    scenario_config,
                    method,
                    initial_truck_routes=method_backbone,
                )
            except Exception as error:
                failure = {
                    "instance_id": instance_id,
                    "method": method,
                    "execution_commit": execution_commit,
                    "started_at_utc": datetime.fromtimestamp(wall_started, timezone.utc).isoformat(),
                    "elapsed_seconds": time.monotonic() - started,
                    "exception": repr(error),
                    "traceback": traceback.format_exc(),
                    "fallback_solution_used": False,
                }
                _append_jsonl(OUTPUT / "run_errors.jsonl", failure)
                _update_manifest("stopped_method_exception", len(completed_keys), stopping_error=failure)
                raise RuntimeError(
                    f"Method execution exception at {instance_id}/{method}; stopped without substitution"
                ) from error

            runtime = time.monotonic() - started
            _write_record(
                raw_return_path,
                {
                    "schema": "final-720-raw-return-v1",
                    "instance_id": instance_id,
                    "method_name": method,
                    "execution_commit_sha": execution_commit,
                    "instance_sha256": instance_row["instance_sha256"],
                    "method_runtime_seconds": runtime,
                    "raw_returned_solution": _solution_payload(raw_solution),
                },
            )
            evaluated = deepcopy(raw_solution)
            raw_objective = float(raw_solution.objective)
            raw_components = deepcopy(raw_solution.components)
            evaluated.objective, evaluated.components = evaluate_solution(instance, evaluated)
            evaluated.run_time_seconds = float(runtime)
            findings = validate_solution(instance, deepcopy(evaluated))
            architecture = ARCHITECTURES[method]
            pairing = default_drone_to_truck(instance) if architecture == "M3" else None
            architecture_violations = validate_drone_architecture(
                instance, evaluated, architecture, pairing
            )
            diagnostics = _event_diagnostics(instance, evaluated)
            row = _decorate_row(
                instance,
                evaluated,
                raw_solution,
                scenario,
                method,
                runtime,
                findings,
                architecture_violations,
                diagnostics,
                instance_row,
                execution_commit,
            )
            row["raw_returned_objective"] = raw_objective
            row["raw_returned_objective_components_json"] = json.dumps(
                _json_safe(raw_components), sort_keys=True, allow_nan=True
            )
            row["shared_backbone_integrity_violation"] = bool(
                method in COMMON_METHODS and not row["shared_initial_backbone_reused"]
            )
            row["initial_backbone_argument_sha256"] = supplied_backbone_hash
            row["initial_backbone_argument_unchanged"] = (
                method_backbone == shared_routes if method_backbone is not None else None
            )
            if method in COMMON_METHODS and not row["initial_backbone_argument_unchanged"]:
                row["shared_backbone_integrity_violation"] = True
            record = {
                "schema": "final-720-method-run-v1",
                "run_started_at_utc": datetime.fromtimestamp(wall_started, timezone.utc).isoformat(),
                "run_completed_at_utc": _utc_now(),
                "instance_id": instance_id,
                "method_name": method,
                "method_seed": row["method_seed"],
                "execution_commit_sha": execution_commit,
                "instance_sha256": instance_row["instance_sha256"],
                "initial_backbone_file_sha256": row["initial_backbone_file_sha256"],
                "initial_backbone_routes_sha256": row["initial_backbone_routes_sha256"],
                "method_runtime_seconds": runtime,
                "row": row,
                "raw_returned_solution": _solution_payload(raw_solution),
                "evaluated_solution": _solution_payload(evaluated),
                "feasibility_findings": [
                    {
                        "severity": item.severity,
                        "code": item.code,
                        "message": item.message,
                        "vehicle": item.vehicle,
                        "node": item.node,
                    }
                    for item in findings
                ],
                "architecture_violations": architecture_violations,
                "diagnostics": diagnostics,
                "post_method_repair_applied": False,
                "post_method_fallback_applied": False,
            }
            _write_record(record_path, record)
            rows.append(row)
            completed_keys.add(key)
            _append_jsonl(CHECKPOINT_FILE, row)
            if len(completed_keys) == 1 or len(completed_keys) % 40 == 0:
                _rebuild_checkpoint_views(rows)
            _atomic_json(
                OUTPUT / "progress.json",
                {
                    "status": "running",
                    "completed_method_instance_runs": len(completed_keys),
                    "expected_method_instance_runs": EXPECTED_RUNS,
                    "latest_instance_id": instance_id,
                    "latest_method": method,
                    "latest_runtime_seconds": runtime,
                    "updated_at_utc": _utc_now(),
                },
            )
            _update_manifest(
                "running",
                len(completed_keys),
                last_completed_instance_id=instance_id,
                last_completed_method=method,
            )
            with progress_log.open("a", encoding="utf-8") as stream:
                stream.write(
                    f"[{_utc_now()}] {len(completed_keys)}/{EXPECTED_RUNS} {instance_id}/{method} "
                    f"objective={row['recomputed_objective']} feasible={row['full_feasibility']} "
                    f"architecture_valid={row['architecture_valid']} runtime={runtime:.2f}s\n"
                )
            if len(completed_keys) % 4 == 0 or len(completed_keys) == EXPECTED_RUNS:
                print(
                    f"FINAL720 progress {len(completed_keys)}/{EXPECTED_RUNS}; "
                    f"last={instance_id}/{method}; runtime={runtime:.1f}s",
                    flush=True,
                )
            if row["shared_backbone_integrity_violation"]:
                _update_manifest(
                    "stopped_shared_backbone_integrity_violation",
                    len(completed_keys),
                    stopping_error={
                        "instance_id": instance_id,
                        "method": method,
                        "reason": "Method did not mark persisted initial backbone as reused",
                    },
                )
                raise RuntimeError(f"Shared-backbone integrity failure at {instance_id}/{method}")

    if len(completed_keys) != EXPECTED_RUNS:
        raise RuntimeError(f"Run loop ended with {EXPECTED_RUNS - len(completed_keys)} missing runs")
    _rebuild_checkpoint_views(rows)
    _update_manifest(
        "runs_complete",
        len(completed_keys),
        completed_at_utc=_utc_now(),
        no_historical_method_rows_resumed=True,
        post_method_repair_applied=False,
        post_method_fallback_applied=False,
    )
    finalize()


def _rank_biserial_improvement(improvement: np.ndarray) -> float:
    from scipy.stats import rankdata

    nonzero = improvement[np.abs(improvement) > 1e-12]
    if not len(nonzero):
        return 0.0
    ranks = rankdata(np.abs(nonzero), method="average")
    positive = float(ranks[nonzero > 0].sum())
    negative = float(ranks[nonzero < 0].sum())
    return (positive - negative) / float(ranks.sum())


def _paired_test_rows(
    frame: pd.DataFrame,
    method_a: str,
    method_b: str,
    *,
    group_values: dict[str, Any] | None = None,
) -> dict[str, Any]:
    from scipy.stats import wilcoxon

    lookup = frame.set_index("instance_id")
    valid = (
        lookup[f"full_feasibility_{method_a}"].astype(bool)
        & lookup[f"architecture_valid_{method_a}"].astype(bool)
        & lookup[f"full_feasibility_{method_b}"].astype(bool)
        & lookup[f"architecture_valid_{method_b}"].astype(bool)
        & np.isfinite(pd.to_numeric(lookup[f"objective_total_{method_a}"], errors="coerce"))
        & np.isfinite(pd.to_numeric(lookup[f"objective_total_{method_b}"], errors="coerce"))
    )
    matched = lookup.loc[valid]
    a = pd.to_numeric(matched[f"objective_total_{method_a}"], errors="coerce").to_numpy(dtype=float)
    b = pd.to_numeric(matched[f"objective_total_{method_b}"], errors="coerce").to_numpy(dtype=float)
    improvement = b - a
    ties = np.isclose(a, b, rtol=0.0, atol=1e-9)
    pct = np.divide(
        (b - a) * 100.0,
        np.abs(b),
        out=np.full_like(a, np.nan),
        where=np.abs(b) > 1e-12,
    )
    try:
        p_value = float(wilcoxon(a, b, alternative="two-sided", method="auto").pvalue) if len(a) else float("nan")
    except ValueError:
        p_value = 1.0 if len(a) else float("nan")
    result = {
        "method_a": method_a,
        "method_b": method_b,
        "matched_pairs": int(len(frame)),
        "valid_pairs": int(len(matched)),
        "invalid_or_infeasible_pairs": int(len(frame) - len(matched)),
        "wins_a_objective": int((improvement > 1e-9).sum()),
        "wins_b_objective": int((improvement < -1e-9).sum()),
        "ties_objective": int(ties.sum()),
        "mean_objective_a": float(a.mean()) if len(a) else float("nan"),
        "mean_objective_b": float(b.mean()) if len(b) else float("nan"),
        "mean_improvement_a_vs_b_pct": float(np.nanmean(pct)) if np.isfinite(pct).any() else float("nan"),
        "median_improvement_a_vs_b_pct": float(np.nanmedian(pct)) if np.isfinite(pct).any() else float("nan"),
        "wilcoxon_p_two_sided": p_value,
        "paired_rank_biserial_effect_a_better": _rank_biserial_improvement(improvement),
        "mean_elapsed_wall_time_a_seconds": float(
            pd.to_numeric(lookup[f"elapsed_wall_time_seconds_{method_a}"], errors="coerce").mean()
        ),
        "mean_elapsed_wall_time_b_seconds": float(
            pd.to_numeric(lookup[f"elapsed_wall_time_seconds_{method_b}"], errors="coerce").mean()
        ),
        "feasible_a": int(lookup[f"full_feasibility_{method_a}"].astype(bool).sum()),
        "feasible_b": int(lookup[f"full_feasibility_{method_b}"].astype(bool).sum()),
        "architecture_valid_a": int(lookup[f"architecture_valid_{method_a}"].astype(bool).sum()),
        "architecture_valid_b": int(lookup[f"architecture_valid_{method_b}"].astype(bool).sum()),
    }
    if group_values:
        result.update(group_values)
    for metric, label in (
        ("high_priority_tardiness_minutes", "high_tardiness"),
        ("medium_priority_tardiness_minutes", "medium_tardiness"),
        ("low_priority_tardiness_minutes", "low_tardiness"),
        ("priority_tardiness_cost", "priority_cost"),
    ):
        left = pd.to_numeric(matched[f"{metric}_{method_a}"], errors="coerce").to_numpy(dtype=float)
        right = pd.to_numeric(matched[f"{metric}_{method_b}"], errors="coerce").to_numpy(dtype=float)
        finite = np.isfinite(left) & np.isfinite(right)
        left, right = left[finite], right[finite]
        delta = right - left
        result[f"mean_{label}_a"] = float(left.mean()) if len(left) else float("nan")
        result[f"mean_{label}_b"] = float(right.mean()) if len(right) else float("nan")
        result[f"wins_a_{label}"] = int((delta > 1e-9).sum())
        result[f"wins_b_{label}"] = int((delta < -1e-9).sum())
        result[f"ties_{label}"] = int(np.isclose(left, right, atol=1e-9, rtol=0.0).sum())
    return result


def _comparison_wide(df: pd.DataFrame, factor_columns: list[str]) -> pd.DataFrame:
    values = [
        "objective_total",
        "elapsed_wall_time_seconds",
        "full_feasibility",
        "architecture_valid",
        "high_priority_tardiness_minutes",
        "medium_priority_tardiness_minutes",
        "low_priority_tardiness_minutes",
        "priority_tardiness_cost",
    ]
    wide = df.pivot(index=["instance_id", "n", *factor_columns], columns="method_name", values=values)
    wide.columns = [f"{measure}_{method}" for measure, method in wide.columns]
    return wide.reset_index()


def _summaries(df: pd.DataFrame, tables: Path) -> dict[str, Any]:
    tables.mkdir(parents=True, exist_ok=True)
    aggregate = {
        "runs": ("instance_id", "count"),
        "feasible_runs": ("full_feasibility", "sum"),
        "architecture_valid_runs": ("architecture_valid", "sum"),
        "mean_objective": ("recomputed_objective", "mean"),
        "median_objective": ("recomputed_objective", "median"),
        "mean_priority_tardiness_cost": ("priority_tardiness_cost", "mean"),
        "mean_high_priority_tardiness": ("high_priority_tardiness_minutes", "mean"),
        "median_high_priority_tardiness": ("high_priority_tardiness_minutes", "median"),
        "mean_medium_priority_tardiness": ("medium_priority_tardiness_minutes", "mean"),
        "mean_low_priority_tardiness": ("low_priority_tardiness_minutes", "mean"),
        "mean_elapsed_wall_seconds": ("elapsed_wall_time_seconds", "mean"),
        "median_elapsed_wall_seconds": ("elapsed_wall_time_seconds", "median"),
        "mean_matched_truck_only_objective": ("matched_truck_only_objective", "mean"),
        "mean_improvement_vs_truck_only_pct": ("improvement_vs_truck_only_pct", "mean"),
        "mean_drone_share": ("drone_service_share", "mean"),
        "mean_drone_distance_km": ("drone_flight_distance_km", "mean"),
        "mean_truck_distance_km": ("truck_route_distance_km", "mean"),
        "mean_synchronization_waiting_minutes": ("synchronization_waiting_minutes", "mean"),
        "mean_synchronization_handling_minutes": ("synchronization_handling_minutes", "mean"),
    }
    overall = df.groupby("method_name", dropna=False).agg(**aggregate).reset_index()
    _write_csv(tables / "overall_method_summary.csv", overall.to_dict("records"))
    by_size = (
        df.groupby(["method_name", "n"], dropna=False)
        .agg(
            runs=("instance_id", "count"),
            feasible_runs=("full_feasibility", "sum"),
            architecture_valid_runs=("architecture_valid", "sum"),
            mean_objective=("recomputed_objective", "mean"),
            median_objective=("recomputed_objective", "median"),
            mean_priority_tardiness_cost=("priority_tardiness_cost", "mean"),
            mean_high_priority_tardiness=("high_priority_tardiness_minutes", "mean"),
            median_high_priority_tardiness=("high_priority_tardiness_minutes", "median"),
            mean_medium_priority_tardiness=("medium_priority_tardiness_minutes", "mean"),
            mean_low_priority_tardiness=("low_priority_tardiness_minutes", "mean"),
            mean_elapsed_wall_seconds=("elapsed_wall_time_seconds", "mean"),
            median_elapsed_wall_seconds=("elapsed_wall_time_seconds", "median"),
            mean_improvement_vs_truck_only_pct=("improvement_vs_truck_only_pct", "mean"),
            mean_drone_share=("drone_service_share", "mean"),
            mean_drone_distance_km=("drone_flight_distance_km", "mean"),
            mean_truck_distance_km=("truck_route_distance_km", "mean"),
            mean_synchronization_waiting_minutes=("synchronization_waiting_minutes", "mean"),
        )
        .reset_index()
    )
    _write_csv(tables / "method_summary_by_size.csv", by_size.to_dict("records"))

    factors = [
        "eligible_share_target",
        "endurance_level",
        "speed_ratio",
        "launch_retrieval_time_class",
        "spatial_pattern",
        "number_of_drones",
        "priority_weight_profile",
        "battery_slack_ratio",
    ]
    factor_rows: list[dict[str, Any]] = []
    for factor in factors:
        grouped = (
            df.groupby(["method_name", factor], dropna=False)
            .agg(
                runs=("instance_id", "count"),
                feasible_runs=("full_feasibility", "sum"),
                architecture_valid_runs=("architecture_valid", "sum"),
                mean_objective=("recomputed_objective", "mean"),
                median_objective=("recomputed_objective", "median"),
                mean_priority_tardiness_cost=("priority_tardiness_cost", "mean"),
                mean_high_priority_tardiness=("high_priority_tardiness_minutes", "mean"),
                mean_medium_priority_tardiness=("medium_priority_tardiness_minutes", "mean"),
                mean_low_priority_tardiness=("low_priority_tardiness_minutes", "mean"),
                mean_elapsed_wall_seconds=("elapsed_wall_time_seconds", "mean"),
                mean_drone_share=("drone_service_share", "mean"),
            )
            .reset_index()
        )
        for item in grouped.to_dict("records"):
            factor_rows.append({"factor": factor, "level": item.pop(factor), **item})
    _write_csv(tables / "results_by_factor.csv", factor_rows)

    priority_rows: list[dict[str, Any]] = []
    for method, group in df.groupby("method_name"):
        for label, time_field, cost_field in (
            ("high", "high_priority_tardiness_minutes", "high_priority_tardiness_weighted_cost"),
            ("medium", "medium_priority_tardiness_minutes", "medium_priority_tardiness_weighted_cost"),
            ("low", "low_priority_tardiness_minutes", "low_priority_tardiness_weighted_cost"),
        ):
            priority_rows.append(
                {
                    "method_name": method,
                    "priority_class": label,
                    "runs": len(group),
                    "mean_tardiness_minutes": pd.to_numeric(group[time_field], errors="coerce").mean(),
                    "median_tardiness_minutes": pd.to_numeric(group[time_field], errors="coerce").median(),
                    "mean_weighted_tardiness_cost": pd.to_numeric(group[cost_field], errors="coerce").mean(),
                }
            )
    _write_csv(tables / "priority_tardiness_by_class.csv", priority_rows)

    pairs = [
        ("paired_baseline", "no_unpairing"),
        ("no_unpairing", "nils"),
        ("paired_baseline", "nils"),
        ("nils", "feasibility_aware_alns"),
    ]
    wide = _comparison_wide(df, factors)
    pair_rows = [_paired_test_rows(wide, left, right) for left, right in pairs]
    _write_csv(tables / "pairwise_comparisons.csv", pair_rows)
    size_rows: list[dict[str, Any]] = []
    for size in sorted(pd.to_numeric(df["n"], errors="coerce").dropna().unique()):
        subset = wide[wide["n"] == size]
        for left, right in pairs:
            size_rows.append(
                _paired_test_rows(subset, left, right, group_values={"n": int(size)})
            )
    _write_csv(tables / "pairwise_comparisons_by_size.csv", size_rows)

    sortie_rows: list[dict[str, Any]] = []
    rendezvous_rows: list[dict[str, Any]] = []
    leg_rows: list[dict[str, Any]] = []
    unnecessary_z1: Counter[str] = Counter()
    unnecessary_z2: Counter[str] = Counter()
    missing_z1: Counter[str] = Counter()
    missing_z2: Counter[str] = Counter()
    unnecessary_z1_valid: Counter[str] = Counter()
    unnecessary_z2_valid: Counter[str] = Counter()
    missing_z1_valid: Counter[str] = Counter()
    missing_z2_valid: Counter[str] = Counter()
    for path in sorted(RUN_RECORDS.glob("*.json.gz")):
        record = _read_record(path)
        row, diag = record["row"], record["diagnostics"]
        for item in diag["sorties"]:
            sortie_rows.append({"instance_id": row["instance_id"], "method_name": row["method_name"], "n": row["n"], **item})
        for item in diag["rendezvous"]:
            rendezvous_rows.append(
                {
                    "instance_id": row["instance_id"],
                    "method_name": row["method_name"],
                    "n": row["n"],
                    "full_feasibility": row["full_feasibility"],
                    "architecture_valid": row["architecture_valid"],
                    **item,
                }
            )
            if item.get("z1") and not item.get("z1_required"):
                unnecessary_z1[row["method_name"]] += 1
                if row["full_feasibility"] and row["architecture_valid"]:
                    unnecessary_z1_valid[row["method_name"]] += 1
            if item.get("z2") and not item.get("z2_required"):
                unnecessary_z2[row["method_name"]] += 1
                if row["full_feasibility"] and row["architecture_valid"]:
                    unnecessary_z2_valid[row["method_name"]] += 1
            if item.get("z1_required") and not item.get("z1"):
                missing_z1[row["method_name"]] += 1
                if row["full_feasibility"] and row["architecture_valid"]:
                    missing_z1_valid[row["method_name"]] += 1
            if item.get("z2_required") and not item.get("z2"):
                missing_z2[row["method_name"]] += 1
                if row["full_feasibility"] and row["architecture_valid"]:
                    missing_z2_valid[row["method_name"]] += 1
        for item in diag["drone_legs"]:
            leg_rows.append({"instance_id": row["instance_id"], "method_name": row["method_name"], "n": row["n"], **item})
    _write_csv(tables / "sorties.csv", sortie_rows)
    _write_csv(tables / "rendezvous_events.csv", rendezvous_rows)
    _write_csv(tables / "drone_legs.csv", leg_rows)
    counts = Counter((str(item["method_name"]), str(item["sortie_type"])) for item in sortie_rows)
    sortie_dist = []
    for method in METHODS:
        total = sum(counts[(method, kind)] for kind in SORTIE_TYPES)
        for kind in SORTIE_TYPES:
            count = counts[(method, kind)]
            sortie_dist.append(
                {
                    "method_name": method,
                    "sortie_type": kind,
                    "sortie_count": count,
                    "total_method_sorties": total,
                    "share_of_method_sorties": count / total if total else float("nan"),
                }
            )
    _write_csv(tables / "sortie_type_distribution.csv", sortie_dist)
    support_summary = (
        df.groupby(["method_name", "n"], dropna=False)
        .agg(
            runs=("instance_id", "count"),
            mean_u_sync_count=("u_sync_count", "mean"),
            total_u_sync_events=("u_sync_count", "sum"),
            mean_z1_count=("z1_count", "mean"),
            total_z1_events=("z1_count", "sum"),
            mean_z2_count=("z2_count", "mean"),
            total_z2_events=("z2_count", "sum"),
            mean_rendezvous_without_support=("rendezvous_neither_reload_nor_swap_count", "mean"),
            total_rendezvous_without_support=("rendezvous_neither_reload_nor_swap_count", "sum"),
            mean_handling_minutes=("synchronization_handling_minutes", "mean"),
            mean_waiting_minutes=("synchronization_waiting_minutes", "mean"),
        )
        .reset_index()
    )
    _write_csv(tables / "rendezvous_support_summary.csv", support_summary.to_dict("records"))
    support_audit = [
        {
            "method_name": method,
            "unnecessary_z1_events": unnecessary_z1[method],
            "unnecessary_z2_events": unnecessary_z2[method],
            "missing_required_z1_events": missing_z1[method],
            "missing_required_z2_events": missing_z2[method],
            "unnecessary_z1_in_feasible_architecture_valid_runs": unnecessary_z1_valid[method],
            "unnecessary_z2_in_feasible_architecture_valid_runs": unnecessary_z2_valid[method],
            "missing_z1_in_feasible_architecture_valid_runs": missing_z1_valid[method],
            "missing_z2_in_feasible_architecture_valid_runs": missing_z2_valid[method],
        }
        for method in METHODS
    ]
    _write_csv(tables / "support_state_audit.csv", support_audit)
    return {
        "overall_method_summary": overall.to_dict("records"),
        "method_summary_by_size": by_size.to_dict("records"),
        "pairwise_comparisons": pair_rows,
        "pairwise_comparisons_by_size": size_rows,
        "support_state_audit": support_audit,
        "sortie_type_distribution": sortie_dist,
        "rendezvous_support_summary": support_summary.to_dict("records"),
    }


def _final_validation(df: pd.DataFrame) -> dict[str, Any]:
    if df["instance_id"].nunique() != 720 or len(df) != EXPECTED_RUNS:
        raise RuntimeError("Final audit did not find 720 instances and 2880 method rows")
    if df.duplicated(["instance_id", "method_name"]).any():
        raise RuntimeError("Final audit found duplicate instance-method rows")
    if set(df["method_name"].unique()) != set(METHODS):
        raise RuntimeError("Final output methods differ from the frozen four-method list")
    if (df.groupby("instance_id")["method_name"].nunique() != 4).any():
        raise RuntimeError("An instance does not have exactly four method rows")
    for field in (
        "full_feasibility",
        "architecture_valid",
        "post_method_repair_applied",
        "post_method_truck_only_fallback_applied",
    ):
        if df[field].isna().any():
            raise RuntimeError(f"Missing explicit final status field: {field}")
    if df["post_method_repair_applied"].astype(bool).any():
        raise RuntimeError("Post-method repair was applied")
    if df["post_method_truck_only_fallback_applied"].astype(bool).any():
        raise RuntimeError("Truck-only fallback was applied")
    for instance_id, group in df[df["method_name"].isin(COMMON_METHODS)].groupby("instance_id"):
        if group["initial_backbone_routes_sha256"].nunique() != 1:
            raise RuntimeError(f"M3/M4/M5 persisted backbone mismatch for {instance_id}")
        if group["initial_backbone_file_sha256"].nunique() != 1:
            raise RuntimeError(f"M3/M4/M5 persisted backbone file hash mismatch for {instance_id}")
        if not group["shared_initial_backbone_reused"].astype(bool).all():
            raise RuntimeError(f"Not all M3/M4/M5 methods reused the backbone for {instance_id}")
        if not group["initial_backbone_argument_unchanged"].astype(bool).all():
            raise RuntimeError(f"A method mutated its supplied persisted backbone for {instance_id}")
    method_validity = (
        df.groupby("method_name")
        .agg(
            runs=("instance_id", "count"),
            feasible=("full_feasibility", "sum"),
            architecture_valid=("architecture_valid", "sum"),
        )
        .reset_index()
        .to_dict("records")
    )
    return {
        "status": "PASS_STRUCTURE_WITH_METHOD_VALIDITY_REPORTED",
        "unique_instances": 720,
        "method_instance_rows": EXPECTED_RUNS,
        "duplicate_instance_method_rows": 0,
        "four_methods_per_instance": True,
        "no_post_method_repair": True,
        "no_truck_only_fallback": True,
        "backbone_hashes_consistent_for_m3_m4_m5": True,
        "all_outputs_have_explicit_feasibility_and_architecture_status": True,
        "invalid_run_count": int((~df["full_feasibility"].astype(bool)).sum()),
        "architecture_invalid_run_count": int((~df["architecture_valid"].astype(bool)).sum()),
        "method_validity_by_method": method_validity,
    }


def _write_report(manifest: dict[str, Any], audit: dict[str, Any], summaries: dict[str, Any]) -> None:
    overall = pd.DataFrame(summaries["overall_method_summary"]).set_index("method_name")
    by_size = pd.DataFrame(summaries["method_summary_by_size"])
    pairs = pd.DataFrame(summaries["pairwise_comparisons"])
    lines = [
        "# Definitive 720-instance experiment",
        "",
        "- Dataset status: FINAL_720_SEMANTICS_CORRECTED_FROZEN",
        f"- Execution commit: {manifest.get('execution_commit')}",
        f"- Configuration SHA256: {manifest.get('configuration_sha256')}",
        "- Frozen instances: 720",
        "- Method-instance runs: 2880",
        "- Historical method rows resumed: 0",
        "- Experiment-level repair/fallback: disabled",
        "",
        "## Validity",
        "",
        "| Method | Feasible | Architecture-valid |",
        "|---|---:|---:|",
    ]
    for item in audit["method_validity_by_method"]:
        lines.append(f"| {item['method_name']} | {item['feasible']}/720 | {item['architecture_valid']}/720 |")
    lines.extend(
        [
            "",
            f"Invalid or architecture-invalid method outputs retained: "
            f"{audit['invalid_run_count']} infeasible, {audit['architecture_invalid_run_count']} architecture-invalid.",
            "",
            "## Overall results",
            "",
            "| Method | Mean objective | Median objective | Mean HP tardiness | Mean elapsed wall time (s) |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for method in METHODS:
        item = overall.loc[method]
        lines.append(
            f"| {method} | {item['mean_objective']:.6g} | {item['median_objective']:.6g} | "
            f"{item['mean_high_priority_tardiness']:.6g} | {item['mean_elapsed_wall_seconds']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Results by size",
            "",
            "| n | Method | Mean objective | Median objective | Mean HP tardiness | Mean elapsed wall time (s) |",
            "|---:|---|---:|---:|---:|---:|",
        ]
    )
    for _, item in by_size.sort_values(["n", "method_name"]).iterrows():
        lines.append(
            f"| {int(item['n'])} | {item['method_name']} | {item['mean_objective']:.6g} | "
            f"{item['median_objective']:.6g} | {item['mean_high_priority_tardiness']:.6g} | "
            f"{item['mean_elapsed_wall_seconds']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Matched comparisons",
            "",
            "| A vs B | Valid pairs | Wins–ties–losses (A) | Mean improvement (%) | Median improvement (%) | Wilcoxon p | Rank-biserial effect |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, item in pairs.iterrows():
        lines.append(
            f"| {item['method_a']} vs {item['method_b']} | {int(item['valid_pairs'])} | "
            f"{int(item['wins_a_objective'])}–{int(item['ties_objective'])}–{int(item['wins_b_objective'])} | "
            f"{item['mean_improvement_a_vs_b_pct']:.4f} | {item['median_improvement_a_vs_b_pct']:.4f} | "
            f"{item['wilcoxon_p_two_sided']:.6g} | {item['paired_rank_biserial_effect_a_better']:.4f} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation guardrails",
            "",
            "The tables evaluate the solutions returned by each algorithm without harness repair or fallback. "
            "Paired tests use instances where both outputs pass full feasibility and their required architecture validator. "
            "Invalid outputs remain in the raw data and validation tables.",
            "",
            "M3–M4 compares fixed-pair and no-cross-truck coordination; M4–M5 describes the incremental unrestricted "
            "cross-truck architecture. These are descriptive comparisons under the frozen common-backbone design. "
            "Runtime is elapsed wall time measured with the same monotonic clock as the algorithm deadline.",
        ]
    )
    (OUTPUT / "reconciliation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def finalize() -> None:
    manifest, _preflight = _verify_run_preflight()
    rows, _keys = _scan_run_records()
    if len(rows) != EXPECTED_RUNS:
        raise RuntimeError(f"Cannot finalize: found {len(rows)} run rows; expected {EXPECTED_RUNS}")
    _rebuild_checkpoint_views(rows)
    df = pd.DataFrame(rows)
    numeric = [
        "n",
        "objective_total",
        "recomputed_objective",
        "matched_truck_only_objective",
        "priority_tardiness_cost",
        "high_priority_tardiness_minutes",
        "medium_priority_tardiness_minutes",
        "low_priority_tardiness_minutes",
        "cpu_time_seconds",
        "elapsed_wall_time_seconds",
        "drone_service_share",
        "truck_route_distance_km",
        "drone_flight_distance_km",
        "synchronization_handling_minutes",
        "synchronization_waiting_minutes",
        "u_sync_count",
        "z1_count",
        "z2_count",
        "rendezvous_neither_reload_nor_swap_count",
    ]
    for column in numeric:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    bool_fields = [
        "full_feasibility",
        "architecture_valid",
        "post_method_repair_applied",
        "post_method_truck_only_fallback_applied",
        "shared_initial_backbone_reused",
        "all_cross_truck_sorties_continuous",
    ]
    for column in bool_fields:
        df[column] = df[column].map(
            lambda value: str(value).lower() in {"true", "1"} if isinstance(value, str) else bool(value)
        )
    df["improvement_vs_truck_only_pct"] = np.where(
        np.abs(df["matched_truck_only_objective"]) > 1e-12,
        (df["matched_truck_only_objective"] - df["recomputed_objective"])
        / np.abs(df["matched_truck_only_objective"])
        * 100.0,
        np.nan,
    )
    audit = _final_validation(df)
    m3_violations = df.loc[df["method_name"] == "paired_baseline", "architecture_violations_json"]
    audit["m3_architecture_invalid_runs"] = int(m3_violations.map(lambda value: bool(json.loads(value))).sum())
    audit["m3_unpaired_event_violations"] = int(
        m3_violations.map(
            lambda value: sum(
                "paired truck" in item.lower() or "unpaired" in item.lower()
                for item in json.loads(value)
            )
        ).sum()
    )
    audit["m4_cross_truck_sortie_count"] = int(
        df.loc[df["method_name"] == "no_unpairing", "cross_truck_sortie_count"].sum()
    )
    audit["m4_cross_truck_recovery_prohibited"] = audit["m4_cross_truck_sortie_count"] == 0
    audit["m5_cross_truck_sortie_count"] = int(
        df.loc[df["method_name"] == "nils", "cross_truck_sortie_count"].sum()
    )
    audit["alns_cross_truck_sortie_count"] = int(
        df.loc[df["method_name"] == "feasibility_aware_alns", "cross_truck_sortie_count"].sum()
    )
    audit["m5_cross_truck_continuity_failures"] = int(
        (
            (df["method_name"] == "nils")
            & ~df["all_cross_truck_sorties_continuous"]
        ).sum()
    )
    audit["alns_cross_truck_continuity_failures"] = int(
        (
            (df["method_name"] == "feasibility_aware_alns")
            & ~df["all_cross_truck_sorties_continuous"]
        ).sum()
    )
    audit["route_intersection_usync_mismatch_runs"] = int(
        (
            (pd.to_numeric(df["intersection_without_declared_u_sync_count"], errors="coerce") > 0)
            | (pd.to_numeric(df["declared_u_sync_without_route_intersection_count"], errors="coerce") > 0)
        ).sum()
    )
    audit["alns_architecture_valid_runs"] = int(
        df.loc[df["method_name"] == "feasibility_aware_alns", "architecture_valid"].sum()
    )
    reference_report = json.loads(
        (REFERENCE_AUDIT.parent / "truck_only_720_backbone_report.json").read_text(encoding="utf-8")
    )
    audit["matched_truck_only_reference"] = reference_report
    audit["dataset_label"] = "FINAL_720_SEMANTICS_CORRECTED_FROZEN"
    summaries = _summaries(df, OUTPUT / "tables")
    audit["support_state_audit"] = summaries["support_state_audit"]
    audit["no_state_unnecessary_support_in_valid_runs"] = all(
        row["unnecessary_z1_in_feasible_architecture_valid_runs"] == 0
        and row["unnecessary_z2_in_feasible_architecture_valid_runs"] == 0
        for row in summaries["support_state_audit"]
    )
    audit["no_missing_required_support_in_valid_runs"] = all(
        row["missing_z1_in_feasible_architecture_valid_runs"] == 0
        and row["missing_z2_in_feasible_architecture_valid_runs"] == 0
        for row in summaries["support_state_audit"]
    )
    _atomic_json(OUTPUT / "validation_audit.json", audit)
    _write_report(manifest, audit, summaries)

    artifact_rows = []
    for path in sorted(OUTPUT.rglob("*")):
        if path.is_file() and path.name not in {"artifact_sha256_manifest.csv", "run_manifest.json"} and not path.name.endswith(".tmp"):
            artifact_rows.append(
                {
                    "path": path.relative_to(REPO).as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": _sha256_file(path),
                }
            )
    _write_csv(OUTPUT / "artifact_sha256_manifest.csv", artifact_rows)
    manifest.update(
        {
            "status": "finalized",
            "dataset_status": "FINAL_720_SEMANTICS_CORRECTED_FROZEN",
            "completed_method_instance_runs": EXPECTED_RUNS,
            "completed_at_utc": _utc_now(),
            "validation_audit": "validation_audit.json",
            "reconciliation_report": "reconciliation_report.md",
            "artifact_sha256_manifest": "artifact_sha256_manifest.csv",
            "artifact_sha256_manifest_sha256": _sha256_file(OUTPUT / "artifact_sha256_manifest.csv"),
            "raw_method_result_rows": len(df),
            "feasibility_by_method": audit["method_validity_by_method"],
            "post_method_repair_applied": False,
            "post_method_fallback_applied": False,
            "finalization": "PASS_STRUCTURE_WITH_METHOD_VALIDITY_REPORTED",
        }
    )
    _atomic_json(OUTPUT / "run_manifest.json", manifest)
    print(
        json.dumps(
            {
                "dataset_status": manifest["dataset_status"],
                "completed_runs": EXPECTED_RUNS,
                "feasibility": audit["method_validity_by_method"],
                "output": str(OUTPUT),
            },
            indent=2,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Preflight, run, and audit the definitive 720-instance study.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--preflight", action="store_true")
    group.add_argument("--run", action="store_true")
    group.add_argument("--finalize", action="store_true")
    args = parser.parse_args()
    if args.preflight:
        preflight()
    elif args.run:
        _run_full()
    else:
        finalize()


if __name__ == "__main__":
    main()

