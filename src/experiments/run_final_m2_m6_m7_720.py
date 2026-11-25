from __future__ import annotations

import csv
import gzip
import hashlib
import json
import math
import os
import platform
import random
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import pulp
import scipy
import yaml
from scipy.stats import rankdata, wilcoxon

from ..data_models import SolutionData
from ..distance_utils import euclidean_distance, manhattan_distance
from ..feasibility import validate_solution
from ..heuristics.architecture import active_rendezvous_events, validate_drone_architecture
from ..heuristics.baselines import (
    nils_no_local_search_baseline,
    nils_no_perturbation_baseline,
    simple_drone_assignment_baseline,
)
from ..heuristics.nils import compute_drone_operation_metrics, evaluate_solution
from ..instance_generator import load_instance_json
from ..parameters import build_search_config
from ..timing import compute_synchronization_waiting
from .final_submission_evidence_audit import verify_frozen_artifacts
from .heuristic_study import _scenario_config
from .refresh_legacy_semantics import sortie_diagnostics
from .run_final_720_semantics_corrected import (
    _json_safe,
    _scenario_from_instance,
    _sha256_file,
    _solution_payload,
    _truck_routes_sha256,
)
from .final_submission_evidence_audit import _read_csv, _write_csv, _write_json
from ..heuristics.construction import _fill_solution_from_routes


REPO = Path(__file__).resolve().parents[2]
FROZEN = REPO / "outputs/final_m3_m4_m5_alns_720_semantics_corrected_20261113"
PRIOR_AUDIT = REPO / "outputs/final_submission_evidence_audit_20261117"
LEGACY_TRUCK_ONLY = REPO / "outputs/manuscript_semantics_reconciliation_20261112/truck_only_720"
OUTPUT = REPO / "outputs/final_submission_evidence_audit_20261125"
RAW_INSTANCES = FROZEN / "raw"
INPUT_MANIFEST = FROZEN / "input_manifest.csv"
RAW_METHODS = FROZEN / "tables/raw_720_method_results.csv"
RUN_CONFIG = FROZEN / "run_config.yaml"
DIAGNOSTIC_IDS = (
    "S0003_R04",
    "S0016_R02",
    "S0024_R03",
    "S0037_R07",
    "S0048_R03",
    "S0050_R01",
    "S0061_R09",
    "S0072_R02",
)
METHODS = {
    "M2": "simple_drone",
    "M6": "nils_no_local_search",
    "M7": "nils_no_perturbation",
}
METHOD_LABELS = {
    "M1": "Truck-only",
    "M2": "Simple-drone",
    "M3": "Paired baseline",
    "M4": "No-unpairing",
    "M5": "NILS",
    "M6": "NILS without local search",
    "M7": "NILS without perturbation",
    "M8": "Feasibility-aware ALNS",
}
SIZES = (15, 50, 75, 100)
EXPECTED_BY_SIZE = {15: 230, 50: 210, 75: 90, 100: 190}
EXPECTED_RUNS = 720
EXPECTED_NEW_RUNS = 2160
AUDIT_TOL = 1e-6
RAW_JSONL = OUTPUT / "new_method_raw_results.jsonl"
PROGRESS_PATH = OUTPUT / "run_progress.json"
PROGRESS_LOG = OUTPUT / "run_progress.log"
RUN_MANIFEST_PATH = OUTPUT / "run_manifest.json"


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO, text=True).strip()


def _atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(_json_safe(payload), indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def _append_line(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(_json_safe(payload), sort_keys=True, allow_nan=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def _append_log(line: str) -> None:
    with PROGRESS_LOG.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(f"{_now()} {line}\n")
        stream.flush()
        os.fsync(stream.fileno())


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _bool(value: Any) -> bool:
    return value is True or value == 1 or str(value).strip().lower() in {"true", "1", "1.0"}


def _float(value: Any, default: float | None = None) -> float | None:
    if value in (None, "", "None", "nan", "NaN"):
        return default
    result = float(value)
    return result if math.isfinite(result) else default


def _merge_search_and_objective_components(
    search_components: dict[str, Any], objective_components: dict[str, Any]
) -> dict[str, Any]:
    return {**search_components, **objective_components}


def _finding_record(item: Any) -> dict[str, Any]:
    return {
        "code": str(getattr(item, "code", "")),
        "severity": str(getattr(item, "severity", "")),
        "message": str(getattr(item, "message", item)),
        "vehicle": getattr(item, "vehicle", None),
        "node": getattr(item, "node", None),
    }


def _load_frozen_inputs() -> tuple[list[dict[str, str]], dict[str, dict[str, str]], dict[str, dict[str, str]]]:
    inputs = _read_csv(INPUT_MANIFEST)
    raw_methods = _read_csv(RAW_METHODS)
    m5 = {row["instance_id"]: row for row in raw_methods if row["method_name"] == "nils"}
    if len(inputs) != EXPECTED_RUNS or len(m5) != EXPECTED_RUNS:
        raise RuntimeError(f"Frozen design mismatch: {len(inputs)} inputs and {len(m5)} M5 records")
    counts = defaultdict(int)
    for row in inputs:
        counts[int(row["n"])] += 1
    if dict(counts) != EXPECTED_BY_SIZE:
        raise RuntimeError(f"Frozen size design mismatch: {dict(counts)}")
    return inputs, {row["instance_id"]: row for row in inputs}, m5


def _verify_frozen_design() -> dict[str, Any]:
    artifact_audit = verify_frozen_artifacts()
    if not artifact_audit["passed"]:
        raise RuntimeError("Frozen principal artifacts are missing or changed")
    inputs, by_id, m5 = _load_frozen_inputs()
    mismatches = []
    for row in inputs:
        instance_id = row["instance_id"]
        instance_path = RAW_INSTANCES / f"{instance_id}.json"
        backbone_path = REPO / row["backbone_file"]
        if not instance_path.is_file() or _sha256_file(instance_path) != row["instance_sha256"]:
            mismatches.append(f"{instance_id}: input hash")
            continue
        if not backbone_path.is_file() or _sha256_file(backbone_path) != row["backbone_file_sha256"]:
            mismatches.append(f"{instance_id}: backbone file hash")
            continue
        backbone_payload = json.loads(backbone_path.read_text(encoding="utf-8"))
        routes = {int(t): [int(node) for node in route] for t, route in backbone_payload["truck_routes"].items()}
        route_hash = _truck_routes_sha256(routes)
        if route_hash != row["backbone_routes_sha256"]:
            mismatches.append(f"{instance_id}: backbone route hash")
        m5_row = m5[instance_id]
        if route_hash != m5_row["initial_backbone_routes_sha256"]:
            mismatches.append(f"{instance_id}: M5 backbone hash")
        if str(m5_row["method_seed"]) != "2026":
            mismatches.append(f"{instance_id}: M5 method seed")
    if mismatches:
        raise RuntimeError("Frozen design integrity failed: " + "; ".join(mismatches[:12]))
    return {
        "inputs": len(inputs),
        "size_counts": {str(size): EXPECTED_BY_SIZE[size] for size in SIZES},
        "M5_reference_records": len(m5),
        "frozen_artifact_manifest": artifact_audit,
        "all_input_and_backbone_hashes_match": True,
    }


def _package_manifest() -> dict[str, Any]:
    packages = {}
    for name, version in (("pulp", pulp.__version__), ("numpy", np.__version__), ("scipy", scipy.__version__), ("PyYAML", yaml.__version__)):
        packages[name] = version
    return {
        "python": sys.version,
        "platform": platform.platform(),
        "packages": packages,
        "available_pulp_solvers": pulp.listSolvers(onlyAvailable=True),
        "cbc_path": pulp.PULP_CBC_CMD().path,
    }


def _initialize_output() -> dict[str, Any]:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    inputs, _, _ = _load_frozen_inputs()
    frozen_check = _verify_frozen_design()
    config_data = yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8"))
    config = build_search_config(config_data)
    head = _git("rev-parse", "HEAD")
    tracked_dirty = bool(_git("status", "--porcelain", "--untracked-files=no"))
    existing = json.loads(RUN_MANIFEST_PATH.read_text(encoding="utf-8")) if RUN_MANIFEST_PATH.is_file() else None
    code_sha = _sha256_file(Path(__file__))
    input_manifest_sha = _sha256_file(INPUT_MANIFEST)
    run_id = hashlib.sha256(f"{head}|{code_sha}|{input_manifest_sha}".encode("utf-8")).hexdigest()
    if existing and (existing.get("run_id") != run_id or existing.get("execution_commit") != head):
        raise RuntimeError("Existing audit output belongs to a different code/input run; refusing to mix records")

    OUTPUT.mkdir(parents=True, exist_ok=True)
    for source, dest in (
        (INPUT_MANIFEST, OUTPUT / "input_manifest.csv"),
        (RUN_CONFIG, OUTPUT / "frozen_run_config.yaml"),
    ):
        if not dest.exists():
            dest.write_bytes(source.read_bytes())

    prior_exact = PRIOR_AUDIT
    exact_copy_map = (
        ("exact_results.csv", "exact_results.csv"),
        ("exact_certificates.csv", "exact_solver_certificates.csv"),
        ("exact_objective_reconciliation.csv", "exact_objective_reconciliation.csv"),
        ("exact_constraint_diagnostics.csv", "exact_constraint_diagnostics.csv"),
        ("exact_validation_summary.md", "exact_validation_report.md"),
        ("exact_table5.tex", "table5_exact_validation.tex"),
    )
    for source_name, dest_name in exact_copy_map:
        destination = OUTPUT / dest_name
        source = prior_exact / source_name
        if not destination.exists():
            destination.write_bytes(source.read_bytes())
    logs_dest = OUTPUT / "exact_solver_logs"
    if not logs_dest.exists():
        import shutil

        shutil.copytree(prior_exact / "exact_solver_logs", logs_dest)
    matrix_rows = [
        {"manuscript_equations": "(10)-(20)", "implementation_location": "src/milp/constraints.py::add_service_and_flow; src/milp/model_builder.py variable construction", "operational_interpretation": "Each customer is assigned exactly once to truck or drone service.", "verification_status": "Implemented and checked on extracted incumbents; valid incumbents pass assignment checks.", "discrepancy": "n=9 has no native CBC incumbent; its relaxation values are not an assignment.", "required_correction": "Retain no-incumbent status for n=9; do not treat relaxation values as a route."},
        {"manuscript_equations": "(10)-(20)", "implementation_location": "src/milp/constraints.py::add_service_and_flow", "operational_interpretation": "Truck arcs obey node flow and route continuity.", "verification_status": "Implemented; validated incumbent routes reconstruct consistently.", "discrepancy": "No fixed-route NILS-to-MILP representation audit was completed.", "required_correction": "Do not claim complete feasible-set equivalence until the fixed-route check is done."},
        {"manuscript_equations": "(10)-(20)", "implementation_location": "src/milp/constraints.py::add_service_and_flow", "operational_interpretation": "Drone arcs obey node flow and route continuity.", "verification_status": "Implemented; valid extracted incumbents pass route reconstruction checks.", "discrepancy": "n=7 fails independent architecture/feasibility validation; n=9 has no incumbent.", "required_correction": "Exclude these two cases from feasible incumbent claims."},
        {"manuscript_equations": "(10)-(20)", "implementation_location": "src/milp/constraints.py::add_return_to_depot; add_service_and_flow", "operational_interpretation": "Vehicles depart from and return to the depot under the modeled route convention.", "verification_status": "Implemented and checked for validated incumbent routes.", "discrepancy": "No additional discrepancy found in valid extracted cases.", "required_correction": "Retain the existing depot route convention; do not infer validity for n=7 or n=9."},
        {"manuscript_equations": "(67)-(70), Appendix A.6", "implementation_location": "src/milp/model_builder.py::_mtz_subtour", "operational_interpretation": "MTZ ordering constraints eliminate disconnected subtours.", "verification_status": "Implementation uses MTZ, consistent with Appendix A.6.", "discrepancy": "Code applies the configured global big-M rather than a separately calibrated MTZ bound.", "required_correction": "Describe the implemented MTZ formulation and global-M choice accurately."},
        {"manuscript_equations": "(21)-(28)", "implementation_location": "src/milp/constraints.py::add_demand_and_load", "operational_interpretation": "Truck inventory and delivery loads are conserved along truck routes.", "verification_status": "Constraint family is implemented; valid incumbents pass independent load checks.", "discrepancy": "No two-way NILS-to-MILP fixed-route compatibility test was completed.", "required_correction": "Avoid asserting full truck-load feasible-set equivalence."},
        {"manuscript_equations": "(21)-(28)", "implementation_location": "src/milp/constraints.py::add_demand_and_load; src/drone_state.py", "operational_interpretation": "Drone parcel-loaded/empty state changes with service and reload events.", "verification_status": "State constraints and independent sortie-state checks are present.", "discrepancy": "n=7/n=9 do not provide fully validated extracted evidence.", "required_correction": "Use only validated incumbents for empirical consistency claims."},
        {"manuscript_equations": "(29)-(40), Appendix A.3", "implementation_location": "src/milp/constraints.py::add_energy; src/energy.py; src/drone_state.py", "operational_interpretation": "Loaded and empty flight legs use distinct energy rates, with power W, energy Wh, and speed converted km/h to m/s.", "verification_status": "Aerodynamic constants and unit conversions are covered by calibration tests.", "discrepancy": "Table 2's nominal 20-km range does not determine the implemented 1,200-Wh battery capacity.", "required_correction": "Reconcile the manuscript's range/capacity statement before claiming parameter consistency."},
        {"manuscript_equations": "(29)-(40)", "implementation_location": "src/milp/constraints.py::add_energy; src/drone_state.py", "operational_interpretation": "Battery state tracks flight use, reserve, and restoration at support events.", "verification_status": "Battery constraints are present; valid incumbents pass battery checks.", "discrepancy": "MILP permits optional z2 restoration while the final validator rejects an unnecessary swap; n=7 exhibits this conflict.", "required_correction": "Resolve the optional-swap/model-validator convention; do not disable the validator."},
        {"manuscript_equations": "(41)-(60)", "implementation_location": "src/milp/constraints.py::add_timing; src/heuristics/architecture.py", "operational_interpretation": "u_sync represents common truck-drone rendezvous at a customer node.", "verification_status": "MILP links u_sync to joint visitation; heuristic architecture checks use physical route intersections.", "discrepancy": "Exact temporal and physical feasible-set equivalence is not established for all extracted cases.", "required_correction": "Keep the fixed-route cross-validation qualification explicit."},
        {"manuscript_equations": "(21)-(40), (54)", "implementation_location": "src/milp/constraints.py::add_demand_and_load; add_energy; src/timing.py::synchronization_handling_minutes", "operational_interpretation": "Parcel reload z1/rho and battery swap z2/sigma are distinct state-dependent support actions; rendezvous handling is charged separately.", "verification_status": "Separate variables and timing terms are implemented.", "discrepancy": "The MILP permits a swap not required by battery state, rejected by the validator at n=7.", "required_correction": "Reconcile optional support-action semantics in the formulation or validator."},
        {"manuscript_equations": "(41)-(60), especially (54)", "implementation_location": "src/milp/constraints.py::add_timing; src/timing.py::synchronization_handling_minutes", "operational_interpretation": "Each physical rendezvous charges 2*tau_hand, plus separate reload/swap durations when selected.", "verification_status": "The heuristic evaluator charges handling on active rendezvous and propagates it to departure times.", "discrepancy": "The MILP wait variables are not exact readiness equalities; timing reconciliation remains incomplete.", "required_correction": "State the operative convention and repair exact waiting formulation before claiming full MILP agreement."},
        {"manuscript_equations": "(41)-(60)", "implementation_location": "src/milp/constraints.py::add_timing; src/timing.py::compute_synchronization_waiting", "operational_interpretation": "Truck/drone waiting is calculated from vehicle readiness after service and compared at rendezvous.", "verification_status": "Heuristic schedule evaluator recomputes readiness-based waits.", "discrepancy": "MILP waiting variables are lower bounds and lack upper/equality links to exact max readiness; this is unresolved.", "required_correction": "Correct the MILP waiting linearization or describe and validate an equivalent exact formulation."},
        {"manuscript_equations": "(41)-(60)", "implementation_location": "src/milp/constraints.py::add_timing; src/heuristics/nils.py::_schedule_routes", "operational_interpretation": "Arrival/departure times propagate travel, service, waiting, and handling through routes.", "verification_status": "Heuristic evaluator recomputes schedules; MILP timing constraints are present.", "discrepancy": "The unresolved wait-variable equality can make the MILP schedule differ from the readiness-based evaluator.", "required_correction": "Do not call timing formulations reconciled until the exact waiting gap is closed."},
        {"manuscript_equations": "(7)-(9), Appendix A.1-A.2", "implementation_location": "src/milp/constraints.py::add_tardiness_linearization; src/milp/extract_solution.py", "operational_interpretation": "Delivery times are linked to the serving vehicle and tardiness is the positive part beyond the deadline.", "verification_status": "Validated incumbents reconcile to the independent evaluator.", "discrepancy": "n=9 has no incumbent; n=7 is invalid for a separate swap issue.", "required_correction": "Limit reconciliation claims to valid incumbent cases."},
        {"manuscript_equations": "(1)-(9)", "implementation_location": "src/milp/objective.py::build_objective; src/milp/constraints.py::add_tardiness_linearization", "operational_interpretation": "Objective sums truck cost, drone cost, and priority-weighted tardiness; waiting is not a separate cost.", "verification_status": "Solver objective, objective components, and independent evaluator agree for feasible validated incumbents.", "discrepancy": "n=8's former mismatch was stale evaluator waiting; corrected readiness-based evaluation reconciles the archived objective.", "required_correction": "Use corrected evaluation and report only genuine solver incumbents."},
        {"manuscript_equations": "Appendix A.5; (41)-(70)", "implementation_location": "config/default.yaml::constraints.big_m; src/milp/constraints.py; src/milp/model_builder.py", "operational_interpretation": "Big-M relaxes timing, load/energy, and MTZ implications.", "verification_status": "The implementation uses a global M=10,000.", "discrepancy": "Appendix A.5 discusses calibrated bounds; implementation does not use separate group-specific values.", "required_correction": "Report the global value or implement calibrated bounds in a separate, controlled formulation revision."},
        {"manuscript_equations": "All; Appendix A", "implementation_location": "src/milp/extract_solution.py; src/experiments/refresh_legacy_semantics.py", "operational_interpretation": "Solver variables are reconstructed into routes, events, timing states, and objective components for independent validation.", "verification_status": "n=3,4,5,6,8 have valid extracted incumbents; n=7 fails validation; n=9 has no feasible incumbent.", "discrepancy": "No fixed-route NILS-to-MILP representation audit was completed.", "required_correction": "Report statuses per case and do not claim bidirectional model/evaluator compatibility."},
    ]
    _write_csv(OUTPUT / "exact_model_code_matrix.csv", matrix_rows)

    manifest = {
        "schema_version": 1,
        "run_id": run_id,
        "created_utc": existing.get("created_utc") if existing else _now(),
        "execution_commit": head,
        "branch": _git("branch", "--show-current"),
        "remote_origin": _git("remote", "get-url", "origin"),
        "tracked_worktree_clean_at_execution": not tracked_dirty,
        "tracked_dirty_at_execution": tracked_dirty,
        "source_code_sha256": code_sha,
        "input_manifest_sha256": input_manifest_sha,
        "frozen_output_dir": str(FROZEN),
        "frozen_output_execution_commit": "8abdc6b7d9f82a7dc630468e4f108e48f2246e84",
        "frozen_artifact_verification": frozen_check,
        "methods": METHODS,
        "expected_records": EXPECTED_NEW_RUNS,
        "design": {"total_instances": EXPECTED_RUNS, "classes": 72, "replications_per_class": 10, "instances_by_size": EXPECTED_BY_SIZE},
        "settings": {
            "heuristic_seed": int(config.heuristics.random_seed),
            "max_outer_iter": int(config.heuristics.max_outer_iter),
            "max_no_improve": int(config.heuristics.max_no_improve),
            "nominal_time_limit_seconds": int(config.heuristics.time_limit_seconds),
            "max_displacement_candidates": int(config.heuristics.max_displacement_candidates),
            "max_repair_candidates": int(config.heuristics.max_repair_candidates),
            "battery_slack_ratio": float(config.experiment.get("battery_slack_ratio", 0.10)),
            "objective_and_validator": "Current readiness-based evaluator; common feasibility validator; M5 architecture validator.",
            "M1_M3_M4_M5_M8_references": "Read-only corrected evaluator revalidation from the 20261117 audit; original frozen outputs are not modified.",
        },
        "environment": _package_manifest(),
        "exact_milp_evidence": {
            "reused_from": str(prior_exact),
            "exact_cases": 7,
            "certified_optima": 1,
            "new_exact_solve_started": False,
            "solver": "CBC 2.10.3; prior per-instance 120-second audit logs retained",
        },
        "status": existing.get("status", "PREPARED") if existing else "PREPARED",
    }
    _atomic_json(RUN_MANIFEST_PATH, manifest)
    return manifest


def _load_instance_context(input_row: dict[str, str], m5_row: dict[str, str], config: Any) -> tuple[Any, Any, dict[int, list[int]], float]:
    instance_id = input_row["instance_id"]
    path = RAW_INSTANCES / f"{instance_id}.json"
    if _sha256_file(path) != input_row["instance_sha256"]:
        raise RuntimeError(f"Input hash changed for {instance_id}")
    instance = load_instance_json(path)
    backbone_path = REPO / input_row["backbone_file"]
    if _sha256_file(backbone_path) != input_row["backbone_file_sha256"]:
        raise RuntimeError(f"Backbone file hash changed for {instance_id}")
    backbone_payload = json.loads(backbone_path.read_text(encoding="utf-8"))
    routes = {int(t): [int(node) for node in route] for t, route in backbone_payload["truck_routes"].items()}
    route_hash = _truck_routes_sha256(routes)
    if route_hash != input_row["backbone_routes_sha256"] or route_hash != m5_row["initial_backbone_routes_sha256"]:
        raise RuntimeError(f"Common M5 backbone hash mismatch for {instance_id}")
    initial = _fill_solution_from_routes(instance, routes)
    initial_objective, _ = evaluate_solution(instance, initial)
    m5_initial = float(m5_row["matched_truck_only_objective"])
    if not math.isclose(initial_objective, m5_initial, rel_tol=1e-12, abs_tol=AUDIT_TOL):
        raise RuntimeError(f"Initial truck-backbone objective mismatch for {instance_id}: {initial_objective} vs {m5_initial}")
    initial_findings = validate_solution(instance, initial)
    initial_errors = [item for item in initial_findings if item.severity == "ERROR"]
    if initial_errors or not _bool(input_row["matched_truck_only_feasible"]):
        details = "; ".join(str(item) for item in initial_errors[:5])
        raise RuntimeError(f"Frozen initial truck backbone is infeasible for {instance_id}: {details}")
    scenario = _scenario_from_instance(instance)
    scenario_config = _scenario_config(config, scenario)
    return instance, scenario_config, routes, float(initial_objective)


def _customer_metrics(instance: Any, solution: SolutionData) -> dict[str, Any]:
    delivery: dict[int, float] = {}
    mode: dict[int, str] = {}
    for customer in instance.customers:
        cid = int(customer.node_id)
        drone = next((d for d in instance.drone_ids() if solution.u_drone.get((d, cid), 0) >= 0.5), None)
        truck = next((t for t in instance.truck_ids() if solution.u_truck.get((t, cid), 0) >= 0.5), None)
        if drone is not None:
            at = solution.a_drone.get((drone, cid))
            if at is not None:
                delivery[cid] = float(at)
            mode[cid] = "drone"
        elif truck is not None:
            at = solution.a_truck.get((truck, cid))
            if at is not None:
                delivery[cid] = float(at)
            mode[cid] = "truck"
    values: dict[str, Any] = {}
    for priority in ("high", "medium", "low"):
        customers = [customer for customer in instance.customers if customer.priority == priority]
        times = [delivery[customer.node_id] for customer in customers if customer.node_id in delivery]
        tardy = [float(solution.tardiness.get(customer.node_id, 0.0)) for customer in customers]
        values[f"{priority}_priority_customer_count"] = len(customers)
        values[f"{priority}_priority_tardiness_minutes"] = sum(tardy)
        values[f"{priority}_priority_mean_tardiness_minutes"] = statistics.mean(tardy) if tardy else None
        values[f"{priority}_priority_mean_delivery_time"] = statistics.mean(times) if times else None
        values[f"{priority}_priority_delivery_count"] = len(times)
        if priority == "high":
            on_time = sum(delivery.get(customer.node_id, float("inf")) <= customer.ub + 1e-9 for customer in customers)
            values["high_priority_on_time_count"] = on_time
            values["high_priority_late_count"] = len(customers) - on_time
            values["high_priority_on_time_rate"] = on_time / len(customers) if customers and len(times) == len(customers) else None
            values["high_priority_late_rate"] = (len(customers) - on_time) / len(customers) if customers and len(times) == len(customers) else None
            early = [customer.ub - delivery[customer.node_id] for customer in customers if customer.node_id in delivery and delivery[customer.node_id] <= customer.ub]
            values["high_priority_mean_minutes_early_when_on_time"] = statistics.mean(early) if early else None
            values["high_priority_served_by_drone_share"] = (
                sum(mode.get(customer.node_id) == "drone" for customer in customers) / len(customers)
                if customers and len(times) == len(customers) else None
            )
    return values


def _solution_metrics(
    instance: Any,
    solution: SolutionData,
    *,
    method: str,
    method_seed: int | None,
    instance_row: dict[str, str],
    backbone_hash: str,
    initial_objective: float,
    elapsed_seconds: float,
    algorithm_wall_seconds: float | None = None,
    exception: str = "",
) -> dict[str, Any]:
    if exception or solution is None:
        return {
            "method": method,
            "method_name": METHODS[method],
            "instance_id": instance_row["instance_id"],
            "scenario_class_id": instance_row["scenario_id"],
            "seed": int(instance_row["original_instance_seed"]),
            "size": int(instance_row["n"]),
            "input_sha256": instance_row["instance_sha256"],
            "backbone_routes_sha256": backbone_hash if method in {"M6", "M7"} else "",
            "shared_initial_backbone_objective": initial_objective if method in {"M6", "M7"} else None,
            "method_seed": method_seed,
            "status": "RUN_EXCEPTION",
            "candidate_objective": None,
            "objective": None,
            "feasible": False,
            "architecture_valid": False,
            "objective_consistent": False,
            "failure_type": "RUN_EXCEPTION",
            "feasibility_errors": [],
            "architecture_errors": [],
            "error": exception,
            "elapsed_wall_seconds": elapsed_seconds,
            "algorithm_wall_seconds": algorithm_wall_seconds,
        }

    raw_returned_objective = _float(solution.objective)
    search_components = dict(solution.components)
    objective, objective_components = evaluate_solution(instance, solution)
    components = _merge_search_and_objective_components(search_components, objective_components)
    solution.objective = float(objective)
    solution.components = dict(components)
    findings = validate_solution(instance, solution)
    errors = [_finding_record(item) for item in findings if item.severity == "ERROR"]
    arch_errors = validate_drone_architecture(instance, solution, "M5")
    arch_errors_serialized = _json_safe(arch_errors)
    feasible = not errors
    architecture_valid = not arch_errors
    component_sum = sum(float(components.get(key, 0.0)) for key in ("truck_cost", "drone_cost", "tardiness_cost"))
    raw_objective_consistent = raw_returned_objective is not None and math.isclose(
        float(objective), raw_returned_objective, rel_tol=1e-9, abs_tol=1e-6
    )
    objective_consistent = math.isclose(float(objective), component_sum, rel_tol=1e-9, abs_tol=1e-6) and raw_objective_consistent
    if method in {"M6", "M7"}:
        flags_valid = (
            float(solution.components.get("initial_backbone_reused", 0.0)) >= 0.5
            and math.isclose(float(solution.components.get("local_search_enabled", -1.0)), 0.0 if method == "M6" else 1.0)
            and math.isclose(float(solution.components.get("perturbation_enabled", -1.0)), 1.0 if method == "M6" else 0.0)
            and (method != "M6" or math.isclose(float(solution.components.get("local_search_calls", 0.0)), 0.0))
            and (method != "M7" or math.isclose(float(solution.components.get("perturbation_moves", 0.0)), 0.0))
        )
    else:
        flags_valid = True
    coords = instance.coordinate_map()
    truck_distance = sum(
        manhattan_distance(coords[left], coords[right])
        for route in solution.truck_routes.values()
        for left, right in zip(route, route[1:])
    )
    drone_distance = sum(
        euclidean_distance(coords[left], coords[right])
        for route in solution.drone_routes.values()
        for left, right in zip(route, route[1:])
    )
    sync = compute_synchronization_waiting(instance, solution)
    sync_handling = sum(float(event.get("handling_minutes", 0.0)) for event in sync["events"])
    operation = compute_drone_operation_metrics(instance, solution)
    sorties = sortie_diagnostics(instance, solution)
    customer = _customer_metrics(instance, solution)
    failure_type = ""
    if not feasible:
        failure_type = "INFEASIBLE_SCHEDULE"
    elif not architecture_valid:
        failure_type = "ARCHITECTURE_INVALID"
    elif not objective_consistent or not flags_valid:
        failure_type = "IMPLEMENTATION_INCONSISTENCY"
    result = {
        "method": method,
        "method_name": METHODS[method],
        "instance_id": instance_row["instance_id"],
        "scenario_class_id": instance_row["scenario_id"],
        "seed": int(instance_row["original_instance_seed"]),
        "size": int(instance_row["n"]),
        "eligible_share_target": float(instance_row["eligible_share_target"]),
        "eligible_share_realized": float(instance_row["eligible_share_realized"]),
        "number_of_drones": int(instance_row["number_of_drones"]),
        "endurance_level": instance_row["endurance_level"],
        "speed_ratio": float(instance_row["speed_ratio"]),
        "handling_time_class": instance_row["handling_time_class"],
        "spatial_pattern": instance_row["spatial_pattern"],
        "priority_weight_profile": instance_row["priority_weight_profile"],
        "battery_slack_ratio": float(instance_row["battery_slack_ratio"]),
        "input_sha256": instance_row["instance_sha256"],
        "backbone_file_sha256": instance_row["backbone_file_sha256"] if method in {"M6", "M7"} else "",
        "backbone_routes_sha256": backbone_hash if method in {"M6", "M7"} else "",
        "same_initial_backbone_as_M5": bool(method in {"M6", "M7"}),
        "initial_backbone_reused": bool(float(search_components.get("initial_backbone_reused", 0.0)) >= 0.5),
        "shared_initial_backbone_objective": initial_objective if method in {"M6", "M7"} else None,
        "method_seed": method_seed,
        "status": str(solution.status),
        "raw_returned_objective": raw_returned_objective,
        "candidate_objective": float(objective),
        "objective": float(objective) if feasible and architecture_valid and objective_consistent else None,
        "truck_cost": float(components.get("truck_cost", 0.0)),
        "drone_cost": float(components.get("drone_cost", 0.0)),
        "priority_tardiness_cost": float(components.get("tardiness_cost", 0.0)),
        "objective_component_sum": component_sum,
        "objective_consistent": objective_consistent,
        "raw_objective_consistent": raw_objective_consistent,
        "feasible": feasible,
        "architecture_valid": architecture_valid,
        "method_flags_valid": flags_valid,
        "valid_run": feasible and architecture_valid and objective_consistent and flags_valid,
        "failure_type": failure_type,
        "feasibility_errors": errors,
        "architecture_errors": arch_errors_serialized,
        "error": "",
        "elapsed_wall_seconds": elapsed_seconds,
        "algorithm_wall_seconds": algorithm_wall_seconds,
        "algorithm_reported_runtime_seconds": _float(solution.run_time_seconds),
        "iterations": _float(solution.components.get("iterations")),
        "local_search_calls": _float(solution.components.get("local_search_calls")),
        "local_search_reverts": _float(solution.components.get("local_search_reverts")),
        "perturbation_moves": _float(solution.components.get("perturbation_moves")),
        "candidate_reassignments_evaluated": _float(solution.components.get("candidate_reassignments_evaluated")),
        "displacement_candidates_generated": _float(solution.components.get("displacement_candidates_generated")),
        "displacement_candidates_fully_evaluated": _float(solution.components.get("displacement_candidates_fully_evaluated")),
        "displacement_deadline_terminated": _float(solution.components.get("displacement_deadline_terminated")),
        "truck_distance": float(truck_distance),
        "drone_distance": float(drone_distance),
        "truck_customers_served": len(solution.served_by_truck()),
        "drone_customers_served": int(operation.get("drone_served_customers", 0)),
        "drone_share": int(operation.get("drone_served_customers", 0)) / max(1, instance.num_customers),
        "active_drones_used": int(operation.get("active_drones_used", 0)),
        "physical_rendezvous_count": len(active_rendezvous_events(solution)),
        "u_sync_count": int(sum(value >= 0.5 for value in solution.u_sync.values())),
        "parcel_reload_count": int(sum(value >= 0.5 for value in solution.z1.values())),
        "battery_swap_count": int(sum(value >= 0.5 for value in solution.z2.values())),
        "synchronization_waiting_minutes": float(sync.get("total_waiting_minutes", 0.0)),
        "synchronization_handling_minutes": float(sync_handling),
        "drone_launches_reported": int(operation.get("drone_launches", 0)),
        "drone_retrievals_reported": int(operation.get("drone_retrievals", 0)),
        "drone_sorties": int(sorties.get("drone_sorties", 0)),
        "depot_depot_sorties": int(sorties.get("depot_depot_sorties", 0)),
        "depot_truck_sorties": int(sorties.get("depot_truck_sorties", 0)),
        "truck_depot_sorties": int(sorties.get("truck_depot_sorties", 0)),
        "same_truck_recovery_sorties": int(sorties.get("same_truck_recovery_sorties", 0)),
        "cross_truck_recovery_sorties": int(sorties.get("cross_truck_recovery_sorties", 0)),
        "loaded_service_reconciled": bool(operation.get("all_drone_service_reconciled", False)),
        "missing_route_service": operation.get("missing_route_service", []),
        "unloaded_service": operation.get("unloaded_service", []),
        "priority_tardiness_by_class": {priority: customer[f"{priority}_priority_tardiness_minutes"] for priority in ("high", "medium", "low")},
        "objective_components": components,
        "diagnostic_counters": {key: value for key, value in solution.components.items() if key not in components},
        "customer_metrics": customer,
        "solution_payload": _solution_payload(solution),
    }
    return result


def _run_one(
    method: str,
    input_row: dict[str, str],
    m5_row: dict[str, str],
    config: Any,
) -> dict[str, Any]:
    started = time.perf_counter()
    instance = None
    routes: dict[int, list[int]] = {}
    initial_objective: float | None = None
    method_seed: int | None = None
    algorithm_wall_seconds: float | None = None
    try:
        instance, scenario_config, routes, initial_objective = _load_instance_context(input_row, m5_row, config)
        method_seed = int(m5_row["method_seed"]) if method in {"M6", "M7"} else int(scenario_config.heuristics.random_seed)
        algorithm_started = time.perf_counter()
        if method == "M2":
            solution = simple_drone_assignment_baseline(instance, scenario_config).solution
        elif method == "M6":
            solution = nils_no_local_search_baseline(
                instance, scenario_config, initial_truck_routes=routes, seed=method_seed
            ).solution
        elif method == "M7":
            solution = nils_no_perturbation_baseline(
                instance, scenario_config, initial_truck_routes=routes, seed=method_seed
            ).solution
        else:
            raise ValueError(f"Unsupported method: {method}")
        algorithm_wall_seconds = time.perf_counter() - algorithm_started
        result = _solution_metrics(
            instance,
            solution,
            method=method,
            method_seed=method_seed,
            instance_row=input_row,
            backbone_hash=_truck_routes_sha256(routes),
            initial_objective=float(initial_objective),
            elapsed_seconds=0.0,
            algorithm_wall_seconds=algorithm_wall_seconds,
        )
        result["elapsed_wall_seconds"] = time.perf_counter() - started
        return result
    except Exception as exception:
        elapsed = time.perf_counter() - started
        if instance is None:
            instance = load_instance_json(RAW_INSTANCES / f"{input_row['instance_id']}.json")
        return _solution_metrics(
            instance,
            None,
            method=method,
            method_seed=method_seed,
            instance_row=input_row,
            backbone_hash=_truck_routes_sha256(routes) if routes else "",
            initial_objective=float(initial_objective or 0.0),
            elapsed_seconds=elapsed,
            algorithm_wall_seconds=algorithm_wall_seconds,
            exception=repr(exception),
        )


def _write_source_manifest() -> None:
    files = []
    for root in (REPO / "src", REPO / "tests", REPO / "config"):
        if root.is_dir():
            files.extend(path for path in root.rglob("*") if path.is_file() and "__pycache__" not in path.parts)
    for name in ("pyproject.toml", "requirements.txt", "requirements-dev.txt", "pytest.ini"):
        path = REPO / name
        if path.is_file():
            files.append(path)
    rows = [{"path": path.relative_to(REPO).as_posix(), "sha256": _sha256_file(path), "size_bytes": path.stat().st_size} for path in sorted(set(files))]
    _write_csv(OUTPUT / "source_sha256_manifest.csv", rows)


def run_diagnostic() -> dict[str, Any]:
    manifest = _initialize_output()
    inputs, by_id, m5 = _load_frozen_inputs()
    sample = [by_id[instance_id] for instance_id in DIAGNOSTIC_IDS]
    if len(sample) != 8:
        raise RuntimeError("Diagnostic subset is incomplete")
    if {int(row["n"]) for row in sample} != set(SIZES):
        raise RuntimeError("Diagnostic sample must span all four problem sizes")
    if {int(row["number_of_drones"]) for row in sample} != {1, 2, 3}:
        raise RuntimeError("Diagnostic sample must span fleets of one, two, and three drones")
    config = build_search_config(yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8")))
    raw_rows = []
    for input_row in sample:
        m5_row = m5[input_row["instance_id"]]
        for method in METHODS:
            _append_log(f"DIAGNOSTIC start {input_row['instance_id']} {method}")
            result = _run_one(method, input_row, m5_row, config)
            result["run_id"] = manifest["run_id"]
            raw_rows.append(result)
            _append_line(OUTPUT / "diagnostic_raw_results.jsonl", result)
            _append_log(
                f"DIAGNOSTIC done {input_row['instance_id']} {method} valid={result.get('valid_run')} "
                f"failure={result.get('failure_type')} wall={result.get('elapsed_wall_seconds', 0):.2f}s"
            )
            print(
                f"DIAGNOSTIC {len(raw_rows)}/24 {input_row['instance_id']} {method} "
                f"valid={result.get('valid_run')} failure={result.get('failure_type')} "
                f"wall={result.get('elapsed_wall_seconds', 0):.1f}s",
                flush=True,
            )
    diagnostic_rows = []
    for row in raw_rows:
        diagnostic_rows.append({key: value for key, value in row.items() if key != "solution_payload"})
    _write_csv(OUTPUT / "preflight_diagnostic.csv", diagnostic_rows)

    by_method = {method: [row for row in raw_rows if row["method"] == method] for method in METHODS}
    exceptions = [row for row in raw_rows if row.get("failure_type") == "RUN_EXCEPTION"]
    m6_m7 = by_method["M6"] + by_method["M7"]
    m2_bad = [row for row in by_method["M2"] if not row.get("feasible")]
    objective_consistent = all(bool(row.get("objective_consistent")) for row in raw_rows if row.get("candidate_objective") is not None)
    flags_and_backbones = all(
        row.get("method_flags_valid") is True
        and row.get("same_initial_backbone_as_M5") is True
        and row.get("initial_backbone_reused")
        and row.get("backbone_routes_sha256") == by_id[row["instance_id"]]["backbone_routes_sha256"]
        for row in m6_m7
    )
    m5_rendezvous_cases = [
        row["instance_id"]
        for row in sample
        if int(m5[row["instance_id"]].get("u_sync_count", 0)) > 0
    ]
    category_coverage = {
        "all_sizes": len({int(row["n"]) for row in sample}) == 4,
        "drone_fleets_1_2_3": {int(row["number_of_drones"]) for row in sample} == {1, 2, 3},
        "eligibility_low_and_high": min(float(row["eligible_share_target"]) for row in sample) <= 0.25
        and max(float(row["eligible_share_target"]) for row in sample) >= 0.75,
        "spatial_uniform_and_clustered": {row["spatial_pattern"] for row in sample} == {"uniform", "clustered"},
        "truck_based_rendezvous_opportunity": bool(m5_rendezvous_cases),
    }
    gate_passed = (
        len(raw_rows) == 24
        and not exceptions
        and objective_consistent
        and flags_and_backbones
        and all(bool(row.get("valid_run")) for row in m6_m7)
        and all(bool(row.get("architecture_valid")) for row in by_method["M2"])
        and all(category_coverage.values())
    )
    gate_summary = {
        "status": "PASS_WITH_RECORDED_METHOD_FAILURES" if gate_passed and m2_bad else "PASS" if gate_passed else "BLOCKED",
        "gate_passed": gate_passed,
        "diagnostic_runs_expected": 24,
        "diagnostic_runs_completed": len(raw_rows),
        "sample_instances": DIAGNOSTIC_IDS,
        "sample_sizes": sorted({int(row["n"]) for row in sample}),
        "fleet_sizes": sorted({int(row["number_of_drones"]) for row in sample}),
        "eligibility_range": [min(float(row["eligible_share_target"]) for row in sample), max(float(row["eligible_share_target"]) for row in sample)],
        "spatial_patterns": sorted({row["spatial_pattern"] for row in sample}),
        "category_coverage": category_coverage,
        "M5_truck_based_rendezvous_opportunity_instances": m5_rendezvous_cases,
        "method_counts": {method: len(rows) for method, rows in by_method.items()},
        "method_valid_counts": {method: sum(bool(row.get("valid_run")) for row in rows) for method, rows in by_method.items()},
        "M2_infeasible_constructive_outcomes": [
            {"instance_id": row["instance_id"], "feasibility_errors": row.get("feasibility_errors", [])}
            for row in m2_bad
        ],
        "M2_failure_treatment": "A constructive benchmark failure is retained and not replaced; it is not treated as a software gate failure if the result is well-formed and architecture-valid.",
        "M6_M7_initial_backbone_seed_flags_passed": flags_and_backbones,
        "objective_components_consistent": objective_consistent,
        "software_exceptions": len(exceptions),
        "frozen_artifact_hashes_passed": manifest["frozen_artifact_verification"]["passed"],
        "objective_reference_note": "227 old M3/M4/M5 objectives differ on corrected re-evaluation; comparisons will use the read-only corrected references while immutable archived records remain untouched.",
        "full_2160_runs_started": False,
    }
    _atomic_json(OUTPUT / "preflight_gate_summary.json", gate_summary)
    manifest["preflight_gate"] = gate_summary
    manifest["status"] = "PRECHECK_PASSED" if gate_passed else "GATE_BLOCKED"
    _atomic_json(RUN_MANIFEST_PATH, manifest)
    _write_source_manifest()
    return gate_summary


def _checkpoint_rows() -> tuple[list[dict[str, Any]], set[tuple[str, str]]]:
    rows = _read_jsonl(RAW_JSONL)
    seen: set[tuple[str, str]] = set()
    for row in rows:
        if row.get("run_id") != json.loads(RUN_MANIFEST_PATH.read_text(encoding="utf-8"))["run_id"]:
            raise RuntimeError("Checkpoint record belongs to another run id")
        key = (str(row["instance_id"]), str(row["method"]))
        if key in seen:
            raise RuntimeError(f"Duplicate checkpoint record {key}")
        seen.add(key)
    return rows, seen


def run_full() -> dict[str, Any]:
    manifest = _initialize_output()
    gate_path = OUTPUT / "preflight_gate_summary.json"
    if not gate_path.is_file():
        raise RuntimeError("Run the diagnostic gate before the full 2,160 runs")
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    if not gate.get("gate_passed"):
        raise RuntimeError("Diagnostic gate is blocked; full experiment not started")
    inputs, _, m5 = _load_frozen_inputs()
    config = build_search_config(yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8")))
    completed_rows, completed = _checkpoint_rows()
    if len(completed) > EXPECTED_NEW_RUNS:
        raise RuntimeError("Checkpoint has more records than the frozen design")
    started_at = time.monotonic()
    manifest["status"] = "RUNNING"
    manifest["full_run_started_utc"] = manifest.get("full_run_started_utc", _now())
    manifest["full_expected_records"] = EXPECTED_NEW_RUNS
    manifest["full_completed_at_start"] = len(completed)
    _atomic_json(RUN_MANIFEST_PATH, manifest)
    _append_log(f"FULL start completed={len(completed)}/{EXPECTED_NEW_RUNS}")

    for input_index, input_row in enumerate(inputs, start=1):
        instance_id = input_row["instance_id"]
        m5_row = m5[instance_id]
        for method in METHODS:
            key = (instance_id, method)
            if key in completed:
                continue
            result = _run_one(method, input_row, m5_row, config)
            result["run_id"] = manifest["run_id"]
            result["run_order"] = len(completed) + 1
            _append_line(RAW_JSONL, result)
            completed.add(key)
            completed_rows.append(result)
            elapsed_total = time.monotonic() - started_at
            summary = {
                "status": "RUNNING",
                "completed": len(completed),
                "expected": EXPECTED_NEW_RUNS,
                "fraction_complete": len(completed) / EXPECTED_NEW_RUNS,
                "by_method": {method_code: sum(1 for row in completed_rows if row["method"] == method_code) for method_code in METHODS},
                "last_instance_id": instance_id,
                "last_method": method,
                "last_valid": result.get("valid_run", False),
                "last_failure_type": result.get("failure_type", ""),
                "last_wall_seconds": result.get("elapsed_wall_seconds", 0.0),
                "elapsed_seconds_this_process": elapsed_total,
                "updated_utc": _now(),
            }
            _atomic_json(PROGRESS_PATH, summary)
            _append_log(
                f"FULL {len(completed)}/{EXPECTED_NEW_RUNS} {instance_id} {method} "
                f"valid={result.get('valid_run')} failure={result.get('failure_type')} "
                f"wall={result.get('elapsed_wall_seconds', 0):.2f}s"
            )
            if len(completed) % 10 == 0 or len(completed) == 1:
                print(
                    f"FULL {len(completed)}/{EXPECTED_NEW_RUNS} ({len(completed) / EXPECTED_NEW_RUNS:.1%}) "
                    f"{instance_id} {method} valid={result.get('valid_run')} "
                    f"wall={result.get('elapsed_wall_seconds', 0):.1f}s",
                    flush=True,
                )

    if len(completed) != EXPECTED_NEW_RUNS:
        raise RuntimeError(f"Full run ended with {len(completed)}/{EXPECTED_NEW_RUNS} records")
    final_manifest = json.loads(RUN_MANIFEST_PATH.read_text(encoding="utf-8"))
    final_manifest["status"] = "RUNS_COMPLETE_SUMMARIZING"
    final_manifest["full_run_completed_utc"] = _now()
    final_manifest["full_run_elapsed_seconds_last_process"] = time.monotonic() - started_at
    _atomic_json(RUN_MANIFEST_PATH, final_manifest)
    return summarize_outputs()


def _m1_reference_rows() -> list[dict[str, Any]]:
    inputs, _, _ = _load_frozen_inputs()
    m1_ref = {row["instance_id"]: row for row in _read_csv(PRIOR_AUDIT / "frozen_720_m1_reference_revalidation.csv")}
    truck_runtime = {row["instance_id"]: row for row in _read_csv(LEGACY_TRUCK_ONLY / "truck_only_720_backbone_audit.csv")}
    config = build_search_config(yaml.safe_load(RUN_CONFIG.read_text(encoding="utf-8")))
    results = []
    for row in inputs:
        instance_id = row["instance_id"]
        instance = load_instance_json(RAW_INSTANCES / f"{instance_id}.json")
        path = REPO / row["backbone_file"]
        payload = json.loads(path.read_text(encoding="utf-8"))
        routes = {int(t): [int(node) for node in route] for t, route in payload["truck_routes"].items()}
        solution = _fill_solution_from_routes(instance, routes)
        objective, components = evaluate_solution(instance, solution)
        solution.objective = float(objective)
        solution.components = dict(components)
        findings = validate_solution(instance, solution)
        arch_errors = validate_drone_architecture(instance, solution, "M5")
        if abs(float(m1_ref[instance_id]["recomputed_objective"]) - float(objective)) > AUDIT_TOL:
            raise RuntimeError(f"M1 objective mismatch on {instance_id}")
        fake_row = dict(row)
        metrics = _solution_metrics(
            instance,
            solution,
            method="M1",
            method_seed=None,
            instance_row=fake_row,
            backbone_hash=_truck_routes_sha256(routes),
            initial_objective=float(objective),
            elapsed_seconds=float(truck_runtime[instance_id]["cpu_seconds"]),
        )
        metrics["valid_run"] = not any(item.severity == "ERROR" for item in findings) and not arch_errors
        metrics["objective"] = float(objective) if metrics["valid_run"] else None
        metrics["cpu_time_seconds"] = float(truck_runtime[instance_id]["cpu_seconds"])
        results.append(metrics)
    return results


def _existing_reference_rows() -> list[dict[str, Any]]:
    raw = _read_csv(PRIOR_AUDIT / "frozen_720_revalidation.csv")
    mapping = {"paired_baseline": "M3", "no_unpairing": "M4", "nils": "M5", "feasibility_aware_alns": "M8"}
    results = []
    for source in raw:
        method = mapping[source["method"]]
        result = {
            "method": method,
            "method_name": source["method"],
            "instance_id": source["instance_id"],
            "scenario_class_id": source["scenario_class_id"],
            "seed": None,
            "size": int(source["size"]),
            "eligible_share_target": _float(source["eligible_share_target"]),
            "eligible_share_realized": _float(source["eligible_share_realized"]),
            "number_of_drones": _float(source["number_of_drones"]),
            "endurance_level": source["endurance_level"],
            "speed_ratio": _float(source["speed_ratio"]),
            "handling_time_class": source["handling_time_class"],
            "spatial_pattern": source["spatial_pattern"],
            "priority_weight_profile": source["priority_weight_profile"],
            "valid_run": _bool(source["corrected_feasible"]) and _bool(source["corrected_architecture_valid"]),
            "feasible": _bool(source["corrected_feasible"]),
            "architecture_valid": _bool(source["corrected_architecture_valid"]),
            "objective": _float(source["corrected_evaluator_objective"]),
            "candidate_objective": _float(source["corrected_evaluator_objective"]),
            "archived_objective": _float(source["old_objective"]),
            "cpu_time_seconds": _float(source["cpu_time_seconds"]),
            "truck_cost": _float(source["truck_cost"]),
            "drone_cost": _float(source["drone_cost"]),
            "priority_tardiness_cost": _float(source["priority_tardiness_cost"]),
            "high_priority_tardiness": _float(source["high_priority_tardiness"]),
            "medium_priority_tardiness_minutes": _float(source["medium_priority_tardiness"]),
            "low_priority_tardiness_minutes": _float(source["low_priority_tardiness"]),
            "high_priority_on_time_rate": _float(source["high_priority_on_time_rate"]),
            "mean_high_priority_delivery_time": _float(source["mean_high_priority_delivery_time"]),
            "truck_distance": _float(source["truck_distance"]),
            "drone_distance": _float(source["drone_distance"]),
            "drone_customers_served": int(float(source["drone_served_customers"])),
            "drone_share": _float(source["drone_share"]),
            "active_drones_used": _float(source["active_drones_used"]),
            "physical_rendezvous_count": int(float(source["physical_rendezvous_count"])),
            "parcel_reload_count": int(float(source["parcel_reload_count"])),
            "battery_swap_count": int(float(source["battery_swap_count"])),
            "synchronization_waiting_minutes": _float(source["synchronization_waiting_minutes"]),
            "synchronization_handling_minutes": _float(source["synchronization_handling_minutes"]),
            "drone_sorties": int(float(source["drone_sorties"])),
            "depot_depot_sorties": int(float(source["depot_depot_sorties"])),
            "depot_truck_sorties": int(float(source["depot_truck_sorties"])),
            "truck_depot_sorties": int(float(source["truck_depot_sorties"])),
            "same_truck_recovery_sorties": int(float(source["same_truck_recovery_sorties"])),
            "cross_truck_recovery_sorties": int(float(source["cross_truck_recovery_sorties"])),
            "objective_source": "read-only corrected evaluator revalidation of frozen saved schedule",
        }
        results.append(result)
    return results


def _new_method_summary_rows() -> list[dict[str, Any]]:
    raw = _read_jsonl(RAW_JSONL)
    rows = []
    for record in raw:
        customer = record.get("customer_metrics", {})
        rows.append({
            "method": record["method"],
            "method_name": record.get("method_name", ""),
            "instance_id": record["instance_id"],
            "scenario_class_id": record.get("scenario_class_id"),
            "seed": record.get("seed"),
            "size": int(record["size"]),
            "eligible_share_target": record.get("eligible_share_target"),
            "eligible_share_realized": record.get("eligible_share_realized"),
            "number_of_drones": record.get("number_of_drones"),
            "endurance_level": record.get("endurance_level"),
            "speed_ratio": record.get("speed_ratio"),
            "handling_time_class": record.get("handling_time_class"),
            "spatial_pattern": record.get("spatial_pattern"),
            "priority_weight_profile": record.get("priority_weight_profile"),
            "valid_run": bool(record.get("valid_run", False)),
            "feasible": bool(record.get("feasible", False)),
            "architecture_valid": bool(record.get("architecture_valid", False)),
            "objective": _float(record.get("objective")),
            "candidate_objective": _float(record.get("candidate_objective")),
            "archived_objective": None,
            "cpu_time_seconds": _float(record.get("algorithm_wall_seconds")),
            "end_to_end_wall_seconds": _float(record.get("elapsed_wall_seconds")),
            "truck_cost": _float(record.get("truck_cost")),
            "drone_cost": _float(record.get("drone_cost")),
            "priority_tardiness_cost": _float(record.get("priority_tardiness_cost")),
            "high_priority_tardiness": customer.get("high_priority_tardiness_minutes"),
            "medium_priority_tardiness_minutes": customer.get("medium_priority_tardiness_minutes"),
            "low_priority_tardiness_minutes": customer.get("low_priority_tardiness_minutes"),
            "high_priority_on_time_rate": customer.get("high_priority_on_time_rate"),
            "mean_high_priority_delivery_time": customer.get("high_priority_priority_mean_delivery_time"),
            "truck_distance": _float(record.get("truck_distance")),
            "drone_distance": _float(record.get("drone_distance")),
            "drone_customers_served": int(record.get("drone_customers_served", 0)),
            "drone_share": _float(record.get("drone_share")),
            "active_drones_used": int(record.get("active_drones_used", 0)),
            "physical_rendezvous_count": int(record.get("physical_rendezvous_count", 0)),
            "parcel_reload_count": int(record.get("parcel_reload_count", 0)),
            "battery_swap_count": int(record.get("battery_swap_count", 0)),
            "synchronization_waiting_minutes": _float(record.get("synchronization_waiting_minutes")),
            "synchronization_handling_minutes": _float(record.get("synchronization_handling_minutes")),
            "drone_sorties": int(record.get("drone_sorties", 0)),
            "depot_depot_sorties": int(record.get("depot_depot_sorties", 0)),
            "depot_truck_sorties": int(record.get("depot_truck_sorties", 0)),
            "truck_depot_sorties": int(record.get("truck_depot_sorties", 0)),
            "same_truck_recovery_sorties": int(record.get("same_truck_recovery_sorties", 0)),
            "cross_truck_recovery_sorties": int(record.get("cross_truck_recovery_sorties", 0)),
            "iterations": record.get("iterations"),
            "failure_type": record.get("failure_type", ""),
            "objective_source": "new execution under current evaluator",
        })
    return rows


def _mean(values: list[float]) -> float | None:
    return statistics.mean(values) if values else None


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _summary_outputs() -> dict[str, Any]:
    existing = _existing_reference_rows()
    m1 = _m1_reference_rows()
    added = _new_method_summary_rows()
    by_key = {(row["method"], row["instance_id"]): row for row in existing + m1 + added}
    if len(by_key) != 8 * EXPECTED_RUNS:
        raise RuntimeError(f"Expected 5,760 all-method instance rows, got {len(by_key)}")
    methods = ["M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8"]
    m1_objectives = {instance_id: _float(by_key[("M1", instance_id)]["objective"]) for instance_id in {row["instance_id"] for row in m1}}
    all_summary = []
    by_size_rows = []
    size_indices: dict[str, dict[int, float | None]] = {method: {} for method in methods}
    for method in methods:
        rows = [row for (row_method, _), row in by_key.items() if row_method == method]
        valid = [row for row in rows if row.get("valid_run") and _float(row.get("objective")) is not None]
        ratios = [100.0 * float(row["objective"]) / float(m1_objectives[row["instance_id"]]) for row in valid if m1_objectives.get(row["instance_id"]) not in (None, 0.0)]
        for size in SIZES:
            size_rows = [row for row in rows if int(row["size"]) == size]
            size_valid = [row for row in size_rows if row.get("valid_run") and _float(row.get("objective")) is not None]
            size_ratios = [100.0 * float(row["objective"]) / float(m1_objectives[row["instance_id"]]) for row in size_valid if m1_objectives.get(row["instance_id"]) not in (None, 0.0)]
            size_indices[method][size] = _mean(size_ratios)
            by_size_rows.append({
                "method": method,
                "method_label": METHOD_LABELS[method],
                "size": size,
                "total_runs": len(size_rows),
                "valid_runs": len(size_valid),
                "invalid_or_unavailable_runs": len(size_rows) - len(size_valid),
                "feasibility_rate": len(size_valid) / len(size_rows) if size_rows else 0.0,
                "mean_objective": _mean([float(row["objective"]) for row in size_valid]),
                "median_objective": _median([float(row["objective"]) for row in size_valid]),
                "mean_cpu_seconds": _mean([float(row["cpu_time_seconds"]) for row in size_valid if _float(row.get("cpu_time_seconds")) is not None]),
                "median_cpu_seconds": _median([float(row["cpu_time_seconds"]) for row in size_valid if _float(row.get("cpu_time_seconds")) is not None]),
                "mean_relative_objective_index_vs_M1": _mean(size_ratios),
                "mean_high_priority_tardiness": _mean([float(row["high_priority_tardiness"]) for row in size_valid if _float(row.get("high_priority_tardiness")) is not None]),
                "mean_high_priority_on_time_rate": _mean([float(row["high_priority_on_time_rate"]) for row in size_valid if _float(row.get("high_priority_on_time_rate")) is not None]),
            })
        all_summary.append({
            "method": method,
            "method_label": METHOD_LABELS[method],
            "total_runs": len(rows),
            "valid_runs": len(valid),
            "invalid_or_unavailable_runs": len(rows) - len(valid),
            "feasibility_rate": len(valid) / len(rows) if rows else 0.0,
            "mean_objective": _mean([float(row["objective"]) for row in valid]),
            "median_objective": _median([float(row["objective"]) for row in valid]),
            "mean_cpu_seconds": _mean([float(row["cpu_time_seconds"]) for row in valid if _float(row.get("cpu_time_seconds")) is not None]),
            "median_cpu_seconds": _median([float(row["cpu_time_seconds"]) for row in valid if _float(row.get("cpu_time_seconds")) is not None]),
            "pooled_relative_objective_index_vs_M1": _mean(ratios),
            "size_balanced_relative_objective_index_vs_M1": _mean([value for value in size_indices[method].values() if value is not None]),
            "covered_size_classes": sum(value is not None for value in size_indices[method].values()),
            "mean_priority_tardiness_cost": _mean([float(row["priority_tardiness_cost"]) for row in valid if _float(row.get("priority_tardiness_cost")) is not None]),
            "mean_high_priority_tardiness_minutes": _mean([float(row["high_priority_tardiness"]) for row in valid if _float(row.get("high_priority_tardiness")) is not None]),
            "mean_high_priority_on_time_rate": _mean([float(row["high_priority_on_time_rate"]) for row in valid if _float(row.get("high_priority_on_time_rate")) is not None]),
        })
    ranked = sorted(all_summary, key=lambda row: (float("inf") if row["size_balanced_relative_objective_index_vs_M1"] is None else row["size_balanced_relative_objective_index_vs_M1"]))
    for rank, row in enumerate(ranked, start=1):
        row["size_balanced_rank"] = rank
    _write_csv(OUTPUT / "all_methods_M1_M8_summary.csv", sorted(ranked, key=lambda row: row["method"]))
    _write_csv(OUTPUT / "all_methods_by_size.csv", by_size_rows)

    operational_rows = []
    op_keys = (
        "drone_share", "drone_customers_served", "active_drones_used", "truck_distance", "drone_distance",
        "physical_rendezvous_count", "parcel_reload_count", "battery_swap_count", "synchronization_waiting_minutes",
        "synchronization_handling_minutes", "drone_sorties", "depot_depot_sorties", "depot_truck_sorties",
        "truck_depot_sorties", "same_truck_recovery_sorties", "cross_truck_recovery_sorties",
        "high_priority_tardiness", "medium_priority_tardiness_minutes", "low_priority_tardiness_minutes",
        "high_priority_on_time_rate", "mean_high_priority_delivery_time",
    )
    for method in methods:
        method_rows = [row for (m, _), row in by_key.items() if m == method]
        for scope in ("All", *SIZES):
            rows = [row for row in method_rows if scope == "All" or int(row["size"]) == scope]
            valid = [row for row in rows if row.get("valid_run")]
            record = {
                "method": method,
                "method_label": METHOD_LABELS[method],
                "size": scope,
                "total_runs": len(rows),
                "valid_runs": len(valid),
                "feasibility_rate": len(valid) / len(rows) if rows else 0.0,
            }
            for key in op_keys:
                values = [float(row[key]) for row in valid if _float(row.get(key)) is not None]
                record[f"mean_{key}"] = _mean(values)
                record[f"median_{key}"] = _median(values)
            operational_rows.append(record)
    _write_csv(OUTPUT / "all_methods_operational_indicators.csv", operational_rows)

    new_records = _read_jsonl(RAW_JSONL)
    simple_rows = []
    validation_rows = []
    for row in new_records:
        flat = {key: value for key, value in row.items() if key not in {"solution_payload", "objective_components", "diagnostic_counters", "customer_metrics", "feasibility_errors", "architecture_errors", "priority_tardiness_by_class"}}
        flat["priority_tardiness_by_class"] = json.dumps(row.get("priority_tardiness_by_class", {}), sort_keys=True)
        flat["diagnostic_counters"] = json.dumps(row.get("diagnostic_counters", {}), sort_keys=True)
        simple_rows.append(flat)
        validation_rows.append({
            "method": row["method"], "instance_id": row["instance_id"], "size": row["size"],
            "status": row.get("status"), "valid_run": row.get("valid_run", False),
            "feasible": row.get("feasible", False), "architecture_valid": row.get("architecture_valid", False),
            "objective_consistent": row.get("objective_consistent", False), "method_flags_valid": row.get("method_flags_valid", False),
            "failure_type": row.get("failure_type", ""),
            "feasibility_errors": json.dumps(row.get("feasibility_errors", []), sort_keys=True),
            "architecture_errors": json.dumps(row.get("architecture_errors", []), sort_keys=True),
            "error": row.get("error", ""),
            "input_sha256": row.get("input_sha256", ""),
            "backbone_routes_sha256": row.get("backbone_routes_sha256", ""),
        })
    _write_csv(OUTPUT / "new_method_results_2160.csv", simple_rows)
    _write_csv(OUTPUT / "new_method_validation.csv", validation_rows)

    comparisons = []
    service = []
    for ablation in ("M6", "M7"):
        for scope in ("All", *SIZES):
            m5_rows = {row["instance_id"]: row for row in by_key.values() if row["method"] == "M5" and row.get("valid_run") and (scope == "All" or int(row["size"]) == scope)}
            ablation_rows = {row["instance_id"]: row for row in by_key.values() if row["method"] == ablation and row.get("valid_run") and (scope == "All" or int(row["size"]) == scope)}
            paired_ids = sorted(set(m5_rows) & set(ablation_rows))
            excluded = (EXPECTED_RUNS if scope == "All" else EXPECTED_BY_SIZE[int(scope)]) - len(paired_ids)
            differences = [float(ablation_rows[i]["objective"]) - float(m5_rows[i]["objective"]) for i in paired_ids]
            improvements = [100.0 * (float(ablation_rows[i]["objective"]) - float(m5_rows[i]["objective"])) / max(abs(float(ablation_rows[i]["objective"])), 1e-12) for i in paired_ids]
            wins = sum(float(m5_rows[i]["objective"]) < float(ablation_rows[i]["objective"]) - AUDIT_TOL for i in paired_ids)
            losses = sum(float(m5_rows[i]["objective"]) > float(ablation_rows[i]["objective"]) + AUDIT_TOL for i in paired_ids)
            ties = len(paired_ids) - wins - losses
            if differences and any(abs(value) > AUDIT_TOL for value in differences):
                test = wilcoxon(
                    [float(m5_rows[i]["objective"]) for i in paired_ids],
                    [float(ablation_rows[i]["objective"]) for i in paired_ids],
                    zero_method="wilcox",
                    alternative="two-sided",
                    method="auto",
                )
                pvalue = float(test.pvalue)
                nonzero = [value for value in differences if abs(value) > AUDIT_TOL]
                ranks = rankdata([abs(value) for value in nonzero], method="average")
                rank_plus = float(sum(rank for rank, value in zip(ranks, nonzero) if value > 0))
                rank_minus = float(sum(rank for rank, value in zip(ranks, nonzero) if value < 0))
                rank_biserial = (rank_plus - rank_minus) / (rank_plus + rank_minus) if rank_plus + rank_minus else None
                dz = statistics.mean(differences) / statistics.stdev(differences) if len(differences) > 1 and statistics.stdev(differences) > 0 else None
                test_statistic = float(test.statistic)
            else:
                pvalue = test_statistic = rank_biserial = dz = None
            comparisons.append({
                "comparison": f"M5_vs_{ablation}", "size": scope,
                "expected_pairs": EXPECTED_RUNS if scope == "All" else EXPECTED_BY_SIZE[int(scope)],
                "valid_matched_pairs": len(paired_ids), "invalid_or_unavailable_pairs": excluded,
                "M5_wins": wins, f"{ablation}_wins": losses, "ties": ties,
                "mean_objective_difference_ablation_minus_M5": _mean(differences),
                "median_objective_difference_ablation_minus_M5": _median(differences),
                "mean_improvement_M5_percent": _mean(improvements), "median_improvement_M5_percent": _median(improvements),
                "wilcoxon_statistic": test_statistic, "wilcoxon_p": pvalue,
                "paired_rank_biserial_effect_M5_favorable_positive": rank_biserial,
                "cohens_dz_ablation_minus_M5": dz,
                "mean_cpu_M5_seconds": _mean([float(m5_rows[i]["cpu_time_seconds"]) for i in paired_ids]),
                f"mean_cpu_{ablation}_seconds": _mean([float(ablation_rows[i]["cpu_time_seconds"]) for i in paired_ids]),
            })
            hp5 = [float(m5_rows[i]["high_priority_tardiness"]) for i in paired_ids if _float(m5_rows[i].get("high_priority_tardiness")) is not None]
            hpab = [float(ablation_rows[i]["high_priority_tardiness"]) for i in paired_ids if _float(ablation_rows[i].get("high_priority_tardiness")) is not None]
            hp5rate = [float(m5_rows[i]["high_priority_on_time_rate"]) for i in paired_ids if _float(m5_rows[i].get("high_priority_on_time_rate")) is not None]
            hpab_rate = [float(ablation_rows[i]["high_priority_on_time_rate"]) for i in paired_ids if _float(ablation_rows[i].get("high_priority_on_time_rate")) is not None]
            service.append({
                "comparison": f"M5_vs_{ablation}", "size": scope, "valid_matched_pairs": len(paired_ids), "invalid_or_unavailable_pairs": excluded,
                "mean_high_priority_tardiness_M5": _mean(hp5), "mean_high_priority_tardiness_ablation": _mean(hpab),
                "mean_high_priority_tardiness_difference_ablation_minus_M5": _mean([b-a for a,b in zip(hp5,hpab)]) if len(hp5)==len(hpab) else None,
                "mean_high_priority_on_time_rate_M5": _mean(hp5rate), "mean_high_priority_on_time_rate_ablation": _mean(hpab_rate),
                "mean_high_priority_on_time_difference_ablation_minus_M5": _mean([b-a for a,b in zip(hp5rate,hpab_rate)]) if len(hp5rate)==len(hpab_rate) else None,
            })
    _write_csv(OUTPUT / "M5_vs_M6_M7_matched.csv", [row for row in comparisons if row["size"] == "All"])
    _write_csv(OUTPUT / "M5_vs_M6_M7_by_size.csv", [row for row in comparisons if row["size"] != "All"])
    _write_csv(OUTPUT / "M5_vs_M6_M7_priority_service.csv", service)
    _write_latex_tables(ranked, operational_rows, comparisons)
    _write_manuscript_actions(ranked, by_size_rows, comparisons, service)
    _write_ablation_report(ranked, by_size_rows, comparisons, service, new_records)
    validation_manifest = _build_validation_manifest(new_records, by_key)
    _atomic_json(OUTPUT / "validation_manifest.json", validation_manifest)
    return validation_manifest


def _fmt(value: Any, digits: int = 2) -> str:
    if value is None or value == "":
        return "--"
    try:
        return f"{float(value):,.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def _write_latex_tables(ranked: list[dict[str, Any]], operational_rows: list[dict[str, Any]], comparisons: list[dict[str, Any]]) -> None:
    summary_lines = [
        r"\begin{table}[htbp]", r"\centering", r"\caption{Matched objective performance across all eight methods}",
        r"\label{tab:all_methods_objective}", r"\small", r"\begin{tabular}{lrrrrrr}", r"\toprule",
        "Method & Valid runs & Mean objective & Median objective & Pooled index & Size-balanced index & Rank \\", r"\midrule",
    ]
    for row in sorted(ranked, key=lambda value: value["size_balanced_rank"]):
        summary_lines.append(
            f"{row['method']} & {row['valid_runs']}/720 & {_fmt(row['mean_objective'])} & {_fmt(row['median_objective'])} & "
            f"{_fmt(row['pooled_relative_objective_index_vs_M1'])} & {_fmt(row['size_balanced_relative_objective_index_vs_M1'])} & {row['size_balanced_rank']} \\\\"
        )
    summary_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    (OUTPUT / "table6_all_methods.tex").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    overall_operational = [row for row in operational_rows if row["size"] == "All"]
    op_lines = [
        r"\begin{table}[htbp]", r"\centering", r"\caption{Operational indicators for all eight methods}",
        r"\label{tab:all_methods_operational}", r"\scriptsize", r"\begin{tabular}{lrrrrrrrr}", r"\toprule",
        "Method & Valid & Drone share & Truck km & Drone km & Rendezvous & Reloads & Swaps & Wait (min) \\", r"\midrule",
    ]
    for row in overall_operational:
        op_lines.append(
            f"{row['method']} & {row['valid_runs']}/720 & {_fmt(row.get('mean_drone_share'),3)} & {_fmt(row.get('mean_truck_distance'))} & "
            f"{_fmt(row.get('mean_drone_distance'))} & {_fmt(row.get('mean_physical_rendezvous_count'))} & {_fmt(row.get('mean_parcel_reload_count'))} & "
            f"{_fmt(row.get('mean_battery_swap_count'))} & {_fmt(row.get('mean_synchronization_waiting_minutes'))} \\\\"
        )
    op_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    (OUTPUT / "table8_all_methods.tex").write_text("\n".join(op_lines) + "\n", encoding="utf-8")

    ablation = [row for row in comparisons if row["size"] == "All"]
    abl_lines = [
        r"\begin{table}[htbp]", r"\centering", r"\caption{Matched ablation comparisons with M5}",
        r"\label{tab:m5_ablation_comparison}", r"\small", r"\begin{tabular}{lrrrrrr}", r"\toprule",
        "Comparator & Pairs & M5 wins & Comparator wins & Ties & Median improvement (%) & Wilcoxon $p$ \\", r"\midrule",
    ]
    for row in ablation:
        comparator = row["comparison"].replace("M5_vs_", "")
        abl_lines.append(
            f"{comparator} & {row['valid_matched_pairs']} & {row['M5_wins']} & {row[f'{comparator}_wins']} & {row['ties']} & "
            f"{_fmt(row['median_improvement_M5_percent'])} & {_fmt(row['wilcoxon_p'],5)} \\\\"
        )
    abl_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    (OUTPUT / "ablation_comparison.tex").write_text("\n".join(abl_lines) + "\n", encoding="utf-8")


def _write_ablation_report(ranked: list[dict[str, Any]], by_size: list[dict[str, Any]], comparisons: list[dict[str, Any]], service: list[dict[str, Any]], new_records: list[dict[str, Any]]) -> None:
    lines = [
        "# M2/M6/M7 full-design results",
        "",
        f"Completed new method-instance runs: {len(new_records)}/{EXPECTED_NEW_RUNS}.",
        "The original M1/M3/M4/M5/M8 artifacts remain immutable; their objective values in this merged analysis are read-only corrected-evaluator re-evaluations.",
        "Invalid new runs remain in the raw records and are excluded from objective averages and paired wins/losses/ties.",
        "",
        "## M1-M8 size-balanced ranking",
        "",
        "| Rank | Method | Valid | Size-balanced objective index | Mean objective |",
        "|---:|---|---:|---:|---:|",
    ]
    for row in sorted(ranked, key=lambda value: value["size_balanced_rank"]):
        lines.append(f"| {row['size_balanced_rank']} | {row['method']} | {row['valid_runs']}/720 | {_fmt(row['size_balanced_relative_objective_index_vs_M1'])} | {_fmt(row['mean_objective'])} |")
    lines += ["", "## Feasibility by new method", ""]
    for method in METHODS:
        method_rows = [row for row in new_records if row["method"] == method]
        valid = sum(bool(row.get("valid_run")) for row in method_rows)
        failures = defaultdict(int)
        for row in method_rows:
            if not row.get("valid_run"):
                failures[str(row.get("failure_type") or "INVALID")] += 1
        lines.append(f"- {method}: {valid}/720 valid; failure types {dict(failures)}.")
    lines += ["", "## M5 ablations", "", "| Comparison | Size | Pairs | M5 wins | Ablation wins | Ties | Median M5 improvement (%) | Wilcoxon p | Rank-biserial |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in comparisons:
        lines.append(f"| {row['comparison']} | {row['size']} | {row['valid_matched_pairs']} | {row['M5_wins']} | {row.get('M6_wins', row.get('M7_wins'))} | {row['ties']} | {_fmt(row['median_improvement_M5_percent'])} | {_fmt(row['wilcoxon_p'],5)} | {_fmt(row['paired_rank_biserial_effect_M5_favorable_positive'],3)} |")
    lines += ["", "## Interpretation", "", "The ablations estimate finite-time component contributions under the frozen design. They do not establish universal causal effects. Report any M6/M7 improvements by size and operational regime; do not assume full NILS is superior.", "M2 is a constructive benchmark; infeasible constructions are retained and counted rather than replaced with truck-only schedules."]
    (OUTPUT / "ablation_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_manuscript_actions(ranked: list[dict[str, Any]], by_size: list[dict[str, Any]], comparisons: list[dict[str, Any]], service: list[dict[str, Any]]) -> None:
    size_ranks = {row["method"]: row["size_balanced_rank"] for row in ranked}
    comp_text = []
    for row in comparisons:
        if row["size"] != "All":
            continue
        comp = row["comparison"].replace("M5_vs_", "")
        comp_text.append(
            f"Against {comp}, M5 has {row['M5_wins']} wins, {row[f'{comp}_wins']} losses, and {row['ties']} ties among {row['valid_matched_pairs']} valid matched instances; median improvement is {_fmt(row['median_improvement_M5_percent'])}\\%."
        )
    all_rank = ", ".join(f"{row['method']} (index { _fmt(row['size_balanced_relative_objective_index_vs_M1']) })" for row in sorted(ranked, key=lambda item: item["size_balanced_rank"]))
    lines = [
        "# Manuscript actions: final computational verification",
        "",
        "## Evidence and scope",
        "",
        f"M1-M8 size-balanced ranking under corrected evaluator: {all_rank}.",
        "Use the corrected evaluator-derived references for the new matched comparison; immutable source outputs are untouched. The 20261117 audit found 227 archived M3/M4/M5 objectives change under readiness-based recomputation; this qualification must remain explicit.",
        "",
        "## Abstract / contribution statements",
        "",
        "KEEP the existing abstract statement on high-priority on-time service if still numerically supported by the matched customer-level audit. Do not claim uniform objective superiority for NILS if the new M1-M8 ranking or size-specific comparisons contradict it.",
        "",
        "## Section 5 and Tables 6/8",
        "",
        "REPLACE the objective/operational table bodies with `table6_all_methods.tex` and `table8_all_methods.tex`; retain existing table labels where possible and update references only as needed.",
        "Use the all-eight-method ranking and valid-run denominators from `all_methods_M1_M8_summary.csv`; report size-balanced and pooled indices separately.",
        "",
        "## NILS ablation paragraph and table",
        "",
        "ADD a concise M5-M6-M7 matched comparison using `ablation_comparison.tex` and `M5_vs_M6_M7_by_size.csv`.",
        "",
        "Copy-ready replacement paragraph:",
        "",
        "```latex",
        "The matched ablation results show that the contribution of local search and perturbation depends on problem size. " + " ".join(comp_text),
        "The corresponding size-specific results are reported in Table~\\ref{tab:m5_ablation_comparison}; invalid or unavailable runs are reported separately and are not counted as objective ties.",
        "```",
        "",
        "## Exact MILP validation",
        "",
        "KEEP the NILS 50-start calibration table. Add `table5_exact_validation.tex` as a separate exact-MILP audit. Only certified cases may be described as optimal; time-limited incumbents are not optima.",
        "The new exact-validation report retains the prior finding: one certified optimum out of seven, with n=7 rejected by the strict swap validator and n=9 having no feasible CBC incumbent. Exact waiting and swap semantics remain a qualification; do not claim full MILP/heuristic feasible-set equivalence.",
        "",
        "## Conclusions",
        "",
        "REVISE the method-ranking and algorithm-component statements to match the saved all-method and size-specific tables. Do not infer causal effects from the D-optimal design or equate drone share with service quality.",
        "",
        "The previous 20261117 action ledger contains exact unchanged corrections for Table 2, Table 3, Table 5 discussion, Tables 13-15, and the abstract. Recheck those against `Drone_paper (9).pdf` before final integration.",
    ]
    (OUTPUT / "MANUSCRIPT_ACTIONS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_validation_manifest(new_records: list[dict[str, Any]], all_method_rows: dict[tuple[str, str], dict[str, Any]]) -> dict[str, Any]:
    counts = {}
    failures = {}
    for method in METHODS:
        rows = [row for row in new_records if row["method"] == method]
        counts[method] = {
            "records": len(rows),
            "valid": sum(bool(row.get("valid_run")) for row in rows),
            "infeasible": sum(row.get("failure_type") == "INFEASIBLE_SCHEDULE" for row in rows),
            "architecture_invalid": sum(row.get("failure_type") == "ARCHITECTURE_INVALID" for row in rows),
            "run_exceptions": sum(row.get("failure_type") == "RUN_EXCEPTION" for row in rows),
            "objective_inconsistencies": sum(row.get("failure_type") == "IMPLEMENTATION_INCONSISTENCY" for row in rows),
        }
        failures[method] = {str(size): sum(1 for row in rows if int(row["size"]) == size and not row.get("valid_run")) for size in SIZES}
    return {
        "created_utc": _now(),
        "expected_new_records": EXPECTED_NEW_RUNS,
        "observed_new_records": len(new_records),
        "method_validation": counts,
        "invalid_counts_by_size": failures,
        "existing_reference_methods": {method: sum(1 for key in all_method_rows if key[0] == method and all_method_rows[key].get("valid_run")) for method in ("M1", "M3", "M4", "M5", "M8")},
        "strict_feasibility_and_architecture_required_for_objective_averages": True,
        "invalid_objectives_excluded_from_averages": True,
        "failures_retained_in_results": True,
    }


def summarize_outputs() -> dict[str, Any]:
    new_rows = _read_jsonl(RAW_JSONL)
    if len(new_rows) != EXPECTED_NEW_RUNS:
        raise RuntimeError(f"Cannot finalize summaries: {len(new_rows)}/{EXPECTED_NEW_RUNS} new runs")
    unique_keys = {(row["method"], row["instance_id"]) for row in new_rows}
    if len(unique_keys) != EXPECTED_NEW_RUNS:
        raise RuntimeError("New method records are not unique by method-instance")
    validation = _summary_outputs()
    manifest = json.loads(RUN_MANIFEST_PATH.read_text(encoding="utf-8"))
    manifest["status"] = "COMPLETED"
    manifest["completed_utc"] = _now()
    manifest["observed_new_records"] = len(new_rows)
    manifest["validation_summary"] = validation
    _atomic_json(RUN_MANIFEST_PATH, manifest)
    _write_source_manifest()
    _write_artifact_hash_manifest()
    _write_readme()
    return validation


def _write_artifact_hash_manifest() -> None:
    path = OUTPUT / "audit_artifact_sha256_manifest.csv"
    rows = []
    for file in sorted(p for p in OUTPUT.rglob("*") if p.is_file() and p != path and not p.name.endswith(".tmp")):
        rows.append({"path": file.relative_to(OUTPUT).as_posix(), "sha256": _sha256_file(file), "size_bytes": file.stat().st_size})
    _write_csv(path, rows)


def _write_readme() -> None:
    content = f"""# Final computational evidence audit\n\nOutput directory: `{OUTPUT.relative_to(REPO).as_posix()}`.\n\nThis package preserves the frozen M1/M3/M4/M5/M8 experiment and adds the M2/M6/M7 runs on the same 720 input instances. The new-run engine checkpoints each method-instance record to `new_method_raw_results.jsonl` and resumes without repeating completed records.\n\n## Reproduction\n\n```powershell\npython -m src.experiments.run_final_m2_m6_m7_720 --diagnostic\npython -m src.experiments.run_final_m2_m6_m7_720 --run-full\npython -m src.experiments.run_final_m2_m6_m7_720 --summarize\n```\n\nThe exact MILP tables and CBC logs are copied from the prior audited seven-case run; they are not relabeled as new solves. See `run_manifest.json` for execution commit, parameters, versions, and hashes.\n\nAll invalid results are retained and excluded from objective averages and matched comparisons. The previous 20261117 evaluator audit found 227 objective deltas in the immutable principal outputs; current comparisons use separately saved readiness-based reevaluations without overwriting those outputs.\n"""
    (OUTPUT / "README.md").write_text(content, encoding="utf-8")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--diagnostic", action="store_true")
    parser.add_argument("--run-full", action="store_true")
    parser.add_argument("--summarize", action="store_true")
    parser.add_argument("--prepare", action="store_true")
    args = parser.parse_args()
    if args.diagnostic:
        print(json.dumps(run_diagnostic(), indent=2))
    if args.run_full:
        print(json.dumps(run_full(), indent=2))
    if args.summarize:
        print(json.dumps(summarize_outputs(), indent=2))
    if args.prepare:
        print(json.dumps(_initialize_output(), indent=2))
    if not (args.diagnostic or args.run_full or args.summarize or args.prepare):
        parser.error("Select --diagnostic, --run-full, --summarize, or --prepare")


if __name__ == "__main__":
    main()
