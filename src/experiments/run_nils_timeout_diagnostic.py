from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from ..feasibility import is_feasible
from ..heuristics.baselines import truck_only_baseline
from ..heuristics.nils import _repair_infeasible_solution, evaluate_solution, run_nils, summarize_drone_usage
from ..instance_generator import load_instance_json
from ..parameters import load_and_build_config
from .heuristic_study import _scenario_config


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = ROOT / "outputs" / "full_revised_usync_20261101_paper_ready"
DEFAULT_OUTPUT = ROOT / "outputs" / "nils_timeout_diagnostic_corrected_20261105"
RUN_FIELDS = [
    "instance_id",
    "scenario_id",
    "scenario_class",
    "instance_seed",
    "nils_seed",
    "size",
    "baseline_objective",
    "rerun_objective",
    "objective_delta_rerun_minus_baseline",
    "baseline_drone_share",
    "rerun_drone_share",
    "baseline_launches",
    "rerun_launches",
    "baseline_cpu_seconds",
    "rerun_cpu_seconds",
    "nils_runtime_seconds",
    "baseline_feasible",
    "rerun_feasible",
    "nils_status_before_wrapper",
    "final_solution_status",
    "iterations",
    "customers_scanned",
    "sortie_options_generated",
    "local_screen_rejections",
    "full_candidates_evaluated",
    "strict_feasibility_checks",
    "terminated_by_deadline",
    "status",
    "error",
]


def _read_source_rows(source_dir: Path, sizes: list[int], per_size: int) -> list[dict[str, str]]:
    source_csv = source_dir / "tables" / "heuristic_study_runs.csv"
    with source_csv.open(newline="", encoding="utf-8-sig") as handle:
        rows = [
            row
            for row in csv.DictReader(handle)
            if row.get("method_name") == "nils" and int(row.get("n", 0)) in sizes
        ]

    selected: list[dict[str, str]] = []
    for size in sizes:
        size_rows = sorted(
            (row for row in rows if int(row["n"]) == size),
            key=lambda row: (row["scenario_id"], row["instance_id"]),
        )
        class_representatives: dict[str, dict[str, str]] = {}
        for row in size_rows:
            class_representatives.setdefault(row["scenario_id"], row)
        representatives = list(class_representatives.values())
        target = min(per_size, len(size_rows))
        if target <= len(representatives):
            if target == 1:
                picks = [len(representatives) // 2]
            else:
                picks = sorted(
                    {round(index * (len(representatives) - 1) / (target - 1)) for index in range(target)}
                )
            chosen = [representatives[index] for index in picks]
        else:
            chosen = list(representatives)

        chosen_ids = {row["instance_id"] for row in chosen}
        for row in size_rows:
            if len(chosen) >= target:
                break
            if row["instance_id"] not in chosen_ids:
                chosen.append(row)
                chosen_ids.add(row["instance_id"])
        selected.extend(chosen[:target])
    return selected


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def _write_run_row(path: Path, row: dict[str, Any]) -> None:
    is_new = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RUN_FIELDS)
        if is_new:
            writer.writeheader()
        writer.writerow(row)
        handle.flush()
        import os

        os.fsync(handle.fileno())


def _write_summary(path: Path, records: list[dict[str, str]], sizes: list[int]) -> None:
    fields = [
        "size",
        "runs",
        "baseline_feasible_runs",
        "rerun_feasible_runs",
        "matched_feasible_pairs",
        "rerun_wins",
        "rerun_losses",
        "ties",
        "baseline_mean_objective",
        "rerun_mean_objective",
        "mean_objective_delta_rerun_minus_baseline",
        "baseline_mean_drone_share",
        "rerun_mean_drone_share",
        "baseline_mean_cpu_seconds",
        "rerun_mean_cpu_seconds",
        "deadline_terminated_runs",
        "mean_sortie_options_generated",
        "mean_full_candidates_evaluated",
    ]
    rows = []
    for size in sizes:
        group = [row for row in records if int(row["size"]) == size and row["status"] == "completed"]
        if not group:
            continue
        paired = [
            row
            for row in group
            if row["baseline_feasible"].lower() == "true" and row["rerun_feasible"].lower() == "true"
        ]
        deltas = [float(row["objective_delta_rerun_minus_baseline"]) for row in paired]
        baseline_objectives = [float(row["baseline_objective"]) for row in paired]
        rerun_objectives = [float(row["rerun_objective"]) for row in paired]
        rows.append(
            {
                "size": size,
                "runs": len(group),
                "baseline_feasible_runs": sum(row["baseline_feasible"].lower() == "true" for row in group),
                "rerun_feasible_runs": sum(row["rerun_feasible"].lower() == "true" for row in group),
                "matched_feasible_pairs": len(paired),
                "rerun_wins": sum(delta < -1e-9 for delta in deltas),
                "rerun_losses": sum(delta > 1e-9 for delta in deltas),
                "ties": sum(abs(delta) <= 1e-9 for delta in deltas),
                "baseline_mean_objective": statistics.mean(baseline_objectives) if paired else "",
                "rerun_mean_objective": statistics.mean(rerun_objectives) if paired else "",
                "mean_objective_delta_rerun_minus_baseline": statistics.mean(deltas) if paired else "",
                "baseline_mean_drone_share": statistics.mean(float(row["baseline_drone_share"]) for row in paired) if paired else "",
                "rerun_mean_drone_share": statistics.mean(float(row["rerun_drone_share"]) for row in paired) if paired else "",
                "baseline_mean_cpu_seconds": statistics.mean(float(row["baseline_cpu_seconds"]) for row in group),
                "rerun_mean_cpu_seconds": statistics.mean(float(row["rerun_cpu_seconds"]) for row in group),
                "deadline_terminated_runs": sum(row["terminated_by_deadline"].lower() == "true" for row in group),
                "mean_sortie_options_generated": statistics.mean(float(row["sortie_options_generated"]) for row in group),
                "mean_full_candidates_evaluated": statistics.mean(float(row["full_candidates_evaluated"]) for row in group),
            }
        )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Matched NILS timeout-salvage diagnostic with per-instance checkpoints.")
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sizes", type=int, nargs="+", default=[50, 75, 100])
    parser.add_argument("--instances-per-size", type=int, default=10)
    parser.add_argument("--instance-ids", nargs="+")
    args = parser.parse_args()

    source_dir = args.source_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    source_csv = source_dir / "tables" / "heuristic_study_runs.csv"
    source_raw_dir = source_dir / "raw"
    provenance_path = source_dir / "run_provenance.json"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    base_config = load_and_build_config(source_dir / "config" / "main_config.yaml")
    experiment_config = dict(base_config.experiment or {})
    force_feasible_outputs = bool(experiment_config.get("study_force_feasible_outputs", True))
    output_repair_steps = max(1, int(experiment_config.get("study_output_repair_max_steps", 8)))
    selected = _read_source_rows(source_dir, args.sizes, args.instances_per_size)
    if args.instance_ids:
        requested_ids = set(args.instance_ids)
        selected = [row for row in _read_source_rows(source_dir, args.sizes, 10_000) if row["instance_id"] in requested_ids]
        missing_ids = requested_ids - {row["instance_id"] for row in selected}
        if missing_ids:
            raise ValueError(f"Requested IDs are missing from the matched baseline: {sorted(missing_ids)}")
    runs_path = output_dir / "nils_timeout_diagnostic_runs.csv"
    metadata_path = output_dir / "run_provenance.json"
    summary_path = output_dir / "nils_timeout_diagnostic_by_size.csv"

    if runs_path.exists():
        with runs_path.open(newline="", encoding="utf-8-sig") as handle:
            existing = list(csv.DictReader(handle))
    else:
        existing = []
    completed_ids = {row["instance_id"] for row in existing if row.get("status") == "completed"}

    def git_value(*parts: str) -> str:
        try:
            return subprocess.check_output(parts, cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
        except Exception:
            return "unavailable"

    nils_path = ROOT / "src" / "heuristics" / "nils.py"
    metadata: dict[str, Any] = {
        "status": "running",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "baseline_output_dir": str(source_dir),
        "baseline_results_csv": str(source_csv),
        "baseline_results_sha256": hashlib.sha256(source_csv.read_bytes()).hexdigest(),
        "baseline_execution_commit": provenance.get("source_git_commit"),
        "diagnostic_execution_commit": git_value("git", "rev-parse", "HEAD"),
        "diagnostic_worktree_status": git_value("git", "status", "--short"),
        "nils_source_sha256": hashlib.sha256(nils_path.read_bytes()).hexdigest(),
        "nils_time_limit_seconds": int(provenance.get("configured_nils_time_limit_seconds", 120)),
        "nils_max_outer_iterations": int(provenance.get("configured_nils_max_outer_iterations", 25)),
        "nils_max_no_improve": int(provenance.get("configured_nils_max_no_improve", 8)),
        "nils_random_seed": int(base_config.heuristics.random_seed),
        "strict_candidate_check_cap": 12,
        "instance_seed_source": "random_seed column in baseline heuristic_study_runs.csv",
        "nils_seed_source": "heuristics.nils.random_seed in config/main_config.yaml",
        "force_feasible_outputs": force_feasible_outputs,
        "output_repair_steps_lower_bound": output_repair_steps,
        "instances_per_size_requested": args.instances_per_size,
        "selected_instance_ids": [row["instance_id"] for row in selected],
        "completed_instances_before_resume": sorted(completed_ids),
        "instances_total": len(selected),
    }
    _atomic_json(metadata_path, metadata)

    time_limit = metadata["nils_time_limit_seconds"]
    max_iter = metadata["nils_max_outer_iterations"]
    max_no_improve = metadata["nils_max_no_improve"]
    pending = [row for row in selected if row["instance_id"] not in completed_ids]
    try:
        for index, baseline in enumerate(pending, start=1):
            instance_id = baseline["instance_id"]
            started = time.monotonic()
            common = {
                "instance_id": instance_id,
                "scenario_id": baseline["scenario_id"],
                "scenario_class": baseline["scenario_class"],
                "instance_seed": int(baseline["random_seed"]),
                "nils_seed": int(base_config.heuristics.random_seed),
                "size": int(baseline["n"]),
                "baseline_objective": float(baseline["objective_total"]),
                "baseline_drone_share": float(baseline["drone_service_share"]),
                "baseline_launches": int(float(baseline["number_of_drone_launches"])),
                "baseline_cpu_seconds": float(baseline["cpu_time_seconds"]),
                "baseline_feasible": str(baseline["final_solution_feasibility_flag"].lower() == "true").lower(),
            }
            try:
                instance = load_instance_json(source_raw_dir / f"{instance_id}.json")
                scenario = {
                    "n": int(baseline["n"]),
                    "eligible_share": float(baseline["eligible_share_target"]),
                    "drones_available": int(baseline["number_of_drones"]),
                    "endurance": baseline["endurance_level"],
                    "speed_ratio": float(baseline["speed_ratio"]),
                    "handling_time": baseline["launch_retrieval_time_class"],
                    "spatial_pattern": baseline["spatial_pattern"],
                    "priority_weight_profile": baseline["priority_weight_profile"],
                    "battery_slack_ratio": float(baseline["battery_slack_ratio"]),
                }
                scenario_config = _scenario_config(base_config, scenario)
                solution = run_nils(
                    instance,
                    seed=common["nils_seed"],
                    max_iter=max_iter,
                    max_no_improve=max_no_improve,
                    time_limit=time_limit,
                )
                nils_status_before_wrapper = str(solution.status)
                if force_feasible_outputs and not is_feasible(instance, solution):
                    repaired = _repair_infeasible_solution(
                        instance,
                        solution,
                        max_steps=max(output_repair_steps, int(instance.num_customers)),
                    )
                    if repaired is not None and is_feasible(instance, repaired):
                        solution = repaired
                        solution.objective, solution.components = evaluate_solution(instance, solution)
                        solution.status = f"{solution.status}|repaired_output"
                    else:
                        solution = truck_only_baseline(instance, scenario_config).solution
                        solution.objective, solution.components = evaluate_solution(instance, solution)
                        solution.status = "nils|fallback_truck_only"
                usage = summarize_drone_usage(solution, instance)
                drone_served = len(solution.served_by_drone())
                if drone_served == 0:
                    drone_served = sum(1 for route in solution.drone_routes.values() for node in route if node != 0)
                components = solution.components or {}
                elapsed = time.monotonic() - started
                objective = float(solution.objective)
                row = {
                    **common,
                    "rerun_objective": objective,
                    "objective_delta_rerun_minus_baseline": objective - common["baseline_objective"],
                    "rerun_drone_share": drone_served / max(1, instance.num_customers),
                    "rerun_launches": int(usage.get("drone_launches", 0)),
                    "rerun_cpu_seconds": elapsed,
                    "nils_runtime_seconds": solution.run_time_seconds,
                    "rerun_feasible": str(bool(is_feasible(instance, solution))).lower(),
                    "nils_status_before_wrapper": nils_status_before_wrapper,
                    "final_solution_status": str(solution.status),
                    "iterations": components.get("iterations", 0),
                    "customers_scanned": components.get("displacement_customers_scanned", 0),
                    "sortie_options_generated": components.get("displacement_sortie_options_generated", 0),
                    "local_screen_rejections": components.get("local_screen_rejections", 0),
                    "full_candidates_evaluated": components.get("candidate_reassignments_evaluated", 0),
                    "strict_feasibility_checks": components.get("strict_feasibility_checks", 0),
                    "terminated_by_deadline": str(
                        bool(components.get("displacement_deadline_terminated", 0))
                    ).lower(),
                    "status": "completed",
                    "error": "",
                }
            except Exception as error:
                row = {
                    **common,
                    "rerun_objective": "",
                    "objective_delta_rerun_minus_baseline": "",
                    "rerun_drone_share": "",
                    "rerun_launches": "",
                    "rerun_cpu_seconds": time.monotonic() - started,
                    "nils_runtime_seconds": "",
                    "rerun_feasible": "",
                    "nils_status_before_wrapper": "",
                    "final_solution_status": "",
                    "iterations": "",
                    "customers_scanned": "",
                    "sortie_options_generated": "",
                    "local_screen_rejections": "",
                    "full_candidates_evaluated": "",
                    "strict_feasibility_checks": "",
                    "terminated_by_deadline": "",
                    "status": "error",
                    "error": f"{type(error).__name__}: {error}",
                }
            _write_run_row(runs_path, row)
            existing.append({key: str(value) for key, value in row.items()})
            _write_summary(summary_path, existing, args.sizes)
            metadata["completed_instances"] = sum(row.get("status") == "completed" for row in existing)
            metadata["error_instances"] = sum(row.get("status") == "error" for row in existing)
            metadata["last_completed_or_failed_instance"] = instance_id
            _atomic_json(metadata_path, metadata)
            print(
                f"[{index}/{len(pending)}] n={common['size']} {instance_id} {row['status']} "
                f"cpu={float(row['rerun_cpu_seconds']):.2f}s "
                f"drone_share={row.get('rerun_drone_share', 'NA')} objective={row.get('rerun_objective', 'NA')}",
                flush=True,
            )
    except KeyboardInterrupt:
        metadata["status"] = "interrupted_checkpointed"
        metadata["interrupted_at_utc"] = datetime.now(timezone.utc).isoformat()
        _atomic_json(metadata_path, metadata)
        raise

    metadata["status"] = "complete" if not any(row.get("status") == "error" for row in existing) else "complete_with_errors"
    metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    metadata["completed_instances"] = sum(row.get("status") == "completed" for row in existing)
    metadata["error_instances"] = sum(row.get("status") == "error" for row in existing)
    _atomic_json(metadata_path, metadata)


if __name__ == "__main__":
    main()
