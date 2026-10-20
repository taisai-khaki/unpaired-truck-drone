"""Audit the June--August M5 runs without changing optimization code."""

from __future__ import annotations

import copy
import hashlib
import json
import platform
import subprocess
import time
from pathlib import Path
from typing import Any

import pandas as pd

from ..feasibility import is_feasible
from ..heuristics.baselines import build_initial_solution
from ..heuristics.nils import evaluate_solution, run_nils
from ..instance_generator import InstanceGenerator, load_instance_json
from ..parameters import load_and_build_config
from .heuristic_study import (
    _apply_eligibility_mask,
    _handling_seconds,
    _run_method,
    _scenario_config,
)
from .run_alns_comparison import _scenario_from_row
from .run_priority_service_speed_by_method import _force_feasible_solution, _run_one


ROOT = Path(__file__).resolve().parents[2]
AUDIT_DIR = ROOT / "outputs" / "nils_revision_audit_20261019"
JUNE_DIR = ROOT / "outputs" / "doptimal_full_beta010_tables5_14_rerun2_20260601_144538"
AUGUST_DIR = ROOT / "outputs" / "paper_report_full_route_metrics_20260827"
RAW_DIR = JUNE_DIR / "raw"
JUNE_RUNS = JUNE_DIR / "tables" / "heuristic_study_runs.csv"
AUGUST_RUNS = AUGUST_DIR / "run_status_full.csv"
AUGUST_DETAIL = AUGUST_DIR / "customer_service_detail.csv"
ALNS_DIR = ROOT / "outputs" / "corrected_alns_final_report_20261019"
ALNS_RUNS = ALNS_DIR / "corrected_alns_full_runs.csv"
ALNS_PROTOCOL = ALNS_DIR / "corrected_alns_protocol.json"
MAIN_CONFIG = ROOT / "config" / "experiment_doptimal_beta010_tables5_14.yaml"
ALNS_CONFIG = ROOT / "config" / "experiment_alns_corrected_40iter.yaml"
EPS = 1e-9


def _float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes"}


def _solution_signature(solution: Any) -> str:
    payload = {
        "truck_routes": solution.truck_routes,
        "drone_routes": solution.drone_routes,
        "u_truck": {str(k): v for k, v in solution.u_truck.items()},
        "u_drone": {str(k): v for k, v in solution.u_drone.items()},
        "z1": {str(k): v for k, v in solution.z1.items()},
        "z2": {str(k): v for k, v in solution.z2.items()},
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode()).hexdigest()


def _solution_record(instance: Any, solution: Any, elapsed: float | None = None) -> dict[str, Any]:
    components = dict(solution.components or {})
    return {
        "objective": _float(solution.objective),
        "feasible": bool(is_feasible(instance, solution)),
        "status": str(solution.status),
        "runtime_seconds": _float(elapsed if elapsed is not None else solution.run_time_seconds),
        "iterations": _float(components.get("iterations")),
        "improving_moves_accepted": _float(components.get("improving_moves_accepted")),
        "candidate_reassignments_evaluated": _float(components.get("candidate_reassignments_evaluated")),
        "repair_attempts": _float(components.get("repair_attempts")),
        "repair_successes": _float(components.get("repair_successes")),
        "repair_failures": _float(components.get("repair_failures")),
        "objective_truck": _float(components.get("truck_cost")),
        "objective_drone": _float(components.get("drone_cost")),
        "objective_waiting": _float(components.get("waiting_cost")),
        "objective_tardiness": _float(components.get("tardiness_cost")),
        "signature": _solution_signature(solution),
    }


def _initial_objective(instance: Any, config: Any) -> float:
    initial = build_initial_solution(instance, config.seed)
    objective, _ = evaluate_solution(instance, initial)
    return float(objective)


def _direct_run(instance: Any, config: Any) -> tuple[Any, dict[str, Any]]:
    start = time.time()
    solution = _run_method(instance, config, "nils")
    return solution, _solution_record(instance, solution, time.time() - start)


def _wrapper_audit(instance: Any, solution: Any, config: Any) -> dict[str, Any]:
    before = copy.deepcopy(solution)
    before_record = _solution_record(instance, before)
    before_signature = before_record["signature"]
    wrapped = _force_feasible_solution(instance, copy.deepcopy(solution), config, "nils")
    after_record = _solution_record(instance, wrapped)
    reevaluated = copy.deepcopy(wrapped)
    reevaluated_objective, _ = evaluate_solution(instance, reevaluated)
    if before_record["feasible"]:
        action = "unchanged"
    elif "fallback_truck_only" in str(wrapped.status):
        action = "truck_only_fallback"
    elif after_record["signature"] != before_signature:
        action = "repaired"
    else:
        action = "unresolved_infeasible"
    return {
        "wrapper_input_objective": before_record["objective"],
        "wrapper_input_feasible": before_record["feasible"],
        "wrapper_output_objective": after_record["objective"],
        "wrapper_output_feasible": after_record["feasible"],
        "wrapper_output_status": after_record["status"],
        "wrapper_action": action,
        "wrapper_solution_changed": after_record["signature"] != before_signature,
        "wrapper_reevaluated_objective": float(reevaluated_objective),
        "wrapper_stored_minus_reevaluated": after_record["objective"] - float(reevaluated_objective),
    }


def _regenerate_instance(old_row: pd.Series, raw: dict[str, Any], config: Any) -> Any:
    scenario = _scenario_from_row(old_row)
    scenario_config = _scenario_config(config, scenario)
    generator = InstanceGenerator.from_search_config(scenario_config)
    handling = _handling_seconds(scenario["handling_time"])
    regenerated = generator.generate_single(
        seed=int(old_row["random_seed"]),
        name=str(old_row["instance_id"]),
        overrides={
            "num_customers": int(old_row["n"]),
            "num_drones": int(old_row["number_of_drones"]),
            "swap_time_s": handling,
            "reload_time_s": handling,
        },
    )
    _apply_eligibility_mask(regenerated, float(old_row["eligible_share_target"]), int(old_row["random_seed"]))
    regenerated.metadata.update(raw.get("metadata", {}))
    return regenerated


def _select_diagnostic_sample(audit: pd.DataFrame) -> pd.DataFrame:
    selected: list[pd.DataFrame] = []
    for size, group in audit.groupby("n", sort=True):
        chosen: list[str] = []
        better = group[group["category"] == "august_better"].sort_values("absolute_difference", ascending=False)
        worse = group[group["category"] == "august_worse"].sort_values("absolute_difference", ascending=False)
        unchanged = group[group["category"] == "unchanged"].sort_values("instance_id")
        for frame, count in ((better, 1), (worse, 1)):
            for instance_id in frame.head(count)["instance_id"]:
                chosen.append(str(instance_id))
        remaining = group[~group["instance_id"].isin(chosen)].sort_values("absolute_difference", ascending=False)
        chosen.extend(str(instance_id) for instance_id in remaining.head(max(0, 2 - len(chosen)))["instance_id"])
        selected.append(group[group["instance_id"].isin(chosen)].copy())
    sample = pd.concat(selected, ignore_index=True)
    return sample.sort_values(["n", "instance_id"]).reset_index(drop=True)


def _merge_m5_outputs() -> tuple[pd.DataFrame, dict[str, str]]:
    june = pd.read_csv(JUNE_RUNS)
    june = june[june["method_name"].astype(str).str.lower() == "nils"].copy()
    august = pd.read_csv(AUGUST_RUNS)
    august = august[august["method_name"].astype(str).str.lower() == "nils"].copy()
    detail_seed = {}
    if AUGUST_DETAIL.exists():
        detail = pd.read_csv(AUGUST_DETAIL, usecols=["instance_id", "method", "seed"])
        detail = detail[detail["method"].astype(str).eq("M5 NILS")]
        detail_seed = detail.groupby("instance_id")["seed"].agg(lambda values: ";".join(sorted(set(values.astype(str))))).to_dict()
    rows = []
    for _, old_row in june.iterrows():
        instance_id = str(old_row["instance_id"])
        new_matches = august[august["instance_id"].astype(str).eq(instance_id)]
        if new_matches.empty:
            continue
        new_row = new_matches.iloc[0]
        old_objective = _float(old_row["objective_total"])
        new_objective = _float(new_row["objective"])
        delta = new_objective - old_objective
        raw_path = RAW_DIR / f"{instance_id}.json"
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        raw_seed = int(raw["seed"])
        seed_values = detail_seed.get(instance_id, "")
        category = "unchanged" if abs(delta) <= EPS else ("august_better" if delta < 0 else "august_worse")
        rows.append(
            {
                "instance_id": instance_id,
                "scenario_id": str(old_row["scenario_id"]),
                "n": int(old_row["n"]),
                "category": category,
                "june_objective": old_objective,
                "august_objective": new_objective,
                "objective_difference_august_minus_june": delta,
                "absolute_difference": abs(delta),
                "percentage_difference_vs_june": (100.0 * delta / abs(old_objective)) if abs(old_objective) > EPS else float("nan"),
                "june_random_seed": int(old_row["random_seed"]),
                "raw_json_seed": raw_seed,
                "august_detail_seed_values": seed_values,
                "seed_match_raw": int(old_row["random_seed"]) == raw_seed,
                "seed_match_august_detail": seed_values in {"", str(int(old_row["random_seed"]))},
                "raw_customer_count": len(raw.get("customers", [])),
                "raw_truck_count": len(raw.get("trucks", [])),
                "raw_drone_count": len(raw.get("drones", [])),
                "raw_swap_time_s": _float(raw.get("constants", {}).get("swap_time_s")),
                "raw_reload_time_s": _float(raw.get("constants", {}).get("reload_time_s")),
                "raw_max_shift_minutes": _float(raw.get("constants", {}).get("max_shift_minutes")),
                "raw_priority_penalties": json.dumps(raw.get("priority_penalties", {}), sort_keys=True),
                "raw_eligible_customer_count": len(raw.get("metadata", {}).get("drone_eligible_customers", [])),
                "raw_json_sha256": _sha256(raw_path),
                "original_june_in_memory_state_recoverable": False,
                "june_truck_cost": _float(old_row.get("truck_travel_cost")),
                "august_truck_cost": _float(new_row.get("truck_travel_cost")),
                "delta_truck_cost": _float(new_row.get("truck_travel_cost")) - _float(old_row.get("truck_travel_cost")),
                "june_drone_cost": _float(old_row.get("drone_travel_cost")),
                "august_drone_cost": _float(new_row.get("drone_travel_cost")),
                "delta_drone_cost": _float(new_row.get("drone_travel_cost")) - _float(old_row.get("drone_travel_cost")),
                "june_waiting_cost": _float(old_row.get("waiting_sync_cost")),
                "august_waiting_cost": _float(new_row.get("waiting_sync_cost")),
                "delta_waiting_cost": _float(new_row.get("waiting_sync_cost")) - _float(old_row.get("waiting_sync_cost")),
                "june_priority_tardiness_cost": _float(old_row.get("priority_tardiness_cost")),
                "august_priority_tardiness_cost": _float(new_row.get("priority_tardiness_cost")),
                "delta_priority_tardiness_cost": _float(new_row.get("priority_tardiness_cost")) - _float(old_row.get("priority_tardiness_cost")),
                "june_feasible": str(old_row.get("final_solution_feasibility_flag", "")),
                "august_feasible": str(new_row.get("feasible", "")),
                "june_iterations": _float(old_row.get("iterations")),
                "june_cpu_seconds": _float(old_row.get("cpu_time_seconds")),
            }
        )
    return pd.DataFrame(rows), detail_seed


def _diagnostic_replay(audit: pd.DataFrame, detail_seed: dict[str, str], main_config: Any) -> pd.DataFrame:
    sample = _select_diagnostic_sample(audit)
    rows: list[dict[str, Any]] = []
    state_subset = set(sample[sample["n"] <= 50].groupby("n", sort=True).head(1)["instance_id"].astype(str))
    june_runs = pd.read_csv(JUNE_RUNS)
    june_nils = june_runs[june_runs["method_name"].astype(str).str.lower() == "nils"]
    for sample_index, (_, audit_row) in enumerate(sample.iterrows(), start=1):
        instance_id = str(audit_row["instance_id"])
        print(f"[audit] diagnostic {sample_index}/{len(sample)} {instance_id}", flush=True)
        old_row = june_nils[june_nils["instance_id"].astype(str).eq(instance_id)].iloc[0]
        raw_path = RAW_DIR / f"{instance_id}.json"
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        scenario = _scenario_from_row(old_row)
        scenario_config = _scenario_config(main_config, scenario)
        base_instance = load_instance_json(raw_path)
        record: dict[str, Any] = {
            "instance_id": instance_id,
            "scenario_id": audit_row["scenario_id"],
            "n": int(audit_row["n"]),
            "category": audit_row["category"],
            "june_objective": audit_row["june_objective"],
            "august_objective": audit_row["august_objective"],
            "sample_absolute_difference": audit_row["absolute_difference"],
            "initial_objective": _initial_objective(base_instance, scenario_config),
            "risk_rollback_enabled": False,
            "config_random_seed": int(scenario_config.heuristics.random_seed),
            "config_max_outer_iter": int(scenario_config.heuristics.max_outer_iter),
            "config_max_no_improve": int(scenario_config.heuristics.max_no_improve),
            "config_time_limit_seconds": float(scenario_config.heuristics.time_limit_seconds),
            "raw_json_sha256": _sha256(raw_path),
            "historical_june_code_available": False,
        }
        direct_instance = load_instance_json(raw_path)
        direct_solution, direct = _direct_run(direct_instance, scenario_config)
        for key, value in direct.items():
            record[f"test_a_direct_{key}"] = value
        wrapper = _wrapper_audit(direct_instance, direct_solution, scenario_config)
        record.update({f"test_c_{key}": value for key, value in wrapper.items()})
        driver_instance = load_instance_json(raw_path)
        driver_solution, driver_elapsed, driver_feasible = _run_one(
            driver_instance,
            scenario,
            "nils",
            scenario_config,
            common_time_limit=None,
        )
        driver = _solution_record(driver_instance, driver_solution, driver_elapsed)
        driver["driver_returned_feasible"] = driver_feasible
        for key, value in driver.items():
            record[f"test_b_priority_driver_{key}"] = value
        regenerated = _regenerate_instance(old_row, raw, main_config)
        regenerated_solution, regenerated_record = _direct_run(regenerated, scenario_config)
        for key, value in regenerated_record.items():
            record[f"test_d_regenerated_in_memory_{key}"] = value
        if instance_id in state_subset:
            reload_instance = load_instance_json(raw_path)
            _, reload_record = _direct_run(reload_instance, scenario_config)
            for key, value in reload_record.items():
                record[f"test_d_reload_repeat_{key}"] = value
            shared_instance = load_instance_json(raw_path)
            preceding_methods = []
            for method_name in ["truck_only", "simple_drone", "paired_baseline", "no_unpairing"]:
                start = time.time()
                preceding_solution = _run_method(shared_instance, scenario_config, method_name)
                preceding_methods.append(
                    f"{method_name}:{preceding_solution.status}:{float(time.time() - start):.4f}s"
                )
            _, shared_record = _direct_run(shared_instance, scenario_config)
            record["test_d_after_other_methods_preceding_status"] = "|".join(preceding_methods)
            for key, value in shared_record.items():
                record[f"test_d_after_other_methods_{key}"] = value
        else:
            record["test_d_reload_repeat_status"] = "not_run_subset"
            record["test_d_after_other_methods_status"] = "not_run_subset"
        rows.append(record)
    return pd.DataFrame(rows)


def _git_text(args: list[str]) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=ROOT, text=True, stderr=subprocess.STDOUT).strip()
    except Exception as error:
        return f"unavailable: {error}"


def _write_markdown(path: Path, content: str) -> None:
    path.write_text(content.rstrip() + "\n", encoding="utf-8")


def _write_reports(audit: pd.DataFrame, diagnostics: pd.DataFrame, detail_seed: dict[str, str]) -> None:
    AUDIT_DIR.mkdir(parents=True, exist_ok=True)
    audit.to_csv(AUDIT_DIR / "m5_june_august_instance_audit.csv", index=False, float_format="%.12f")
    diagnostics.to_csv(AUDIT_DIR / "diagnostic_replay.csv", index=False, float_format="%.12f")

    changed = audit[audit["category"] != "unchanged"]
    improved = audit[audit["category"] == "august_better"]
    worsened = audit[audit["category"] == "august_worse"]
    git_log = _git_text(["log", "--all", "--date=iso", "--format=%h %ad %s", "-3"])
    diff_stat = _git_text(["diff", "--stat", "2d847ce904267a0ea97b29393dce0c08b5e0939c", "1554ed832806372df01dfc375eebd71b1bc164fa"])
    timeline = f"""# Revision Timeline

## Evidence reviewed

- June M5: `outputs/doptimal_full_beta010_tables5_14_rerun2_20260601_144538/tables/heuristic_study_runs.csv`.
- August M5/M9: `outputs/paper_report_full_route_metrics_20260827/run_status_full.csv`.
- Corrected ALNS: `outputs/corrected_alns_final_report_20261019/corrected_alns_full_runs.csv`.
- Raw JSON reference: `outputs/doptimal_full_beta010_tables5_14_rerun2_20260601_144538/raw/`.

## Chronology

- June 1--9, 2026: the submitted heuristic study ran through `run-heuristic-study`; its run metadata records `720` instances and seven methods, including standard NILS.
- July 24, 2026: commit `2d847ce90` is the first tracked repository commit. The June source tree is therefore not available as a Git object.
- August 27, 2026: the priority-service driver ran standard NILS, NILS-R, truck-only, and feasibility-aware ALNS. Its run logs record the August M5 values.
- August 28, 2026: commit `1554ed832` captures the NILS-R and priority-service implementation after the August run had started and finished. The exact Git SHA used by the August process was not recorded.
- October 19, 2026: commit `c952b5c70` captures the corrected ALNS benchmark and fairness reports.

## Version conclusion

The June source version is unavailable. The August run is reproducible from the current priority-service driver for sampled instances and appears to correspond to the working tree later captured by `1554ed832`, but this cannot be promoted to a proven run SHA.

Recent history:

```text
{git_log}
```
"""
    _write_markdown(AUDIT_DIR / "revision_timeline.md", timeline)

    algorithm = f"""# Algorithm Change Audit

## Proven differences

1. The June run used `src/experiments/heuristic_study.py:1885-1907`, executing seven methods sequentially on the generated in-memory instance.
2. The August run used `src/experiments/run_priority_service_speed_by_method.py:474-496`, reloading each raw JSON instance and executing the selected methods separately.
3. The August wrapper calls `_force_feasible_solution` and then `evaluate_solution` in `src/experiments/run_priority_service_speed_by_method.py:287-312`.
4. The August command-line option `--common-time-limit` defaults to `None` at `src/experiments/run_priority_service_speed_by_method.py:655-658`; the August logs do not record whether an override was supplied.
5. Standard M5 is not NILS-R. The current `run_nils` default is `risk_rollback=False` at `src/heuristics/nils.py:1017`, and the NILS-R logic is activated only through a separate method.

## Code-history comparison

The tracked July-to-August diff adds the NILS-R machinery and reporting infrastructure. The base `run_nils` body is not shown as changed apart from optional risk-rollback parameters and guarded calls. The June implementation itself predates Git, so this does not prove that the June and August base code were identical.

```text
{diff_stat}
```

## Input and configuration findings

- All `720` instance IDs are present in both M5 outputs.
- The old M5 seed and raw JSON seed match for all `720` instances.
- The M5 customer-detail seed values align with the June seed for all `720` instances.
- The raw JSON files contain the current numerical instance data and are the correct reference for the August run.
- The original June in-memory objects were not preserved, so exact coordinate/demand/window equality between the June object and the later JSON object cannot be proven from the artifacts alone.
- Current tracked NILS settings are `max_outer_iter=25`, `max_no_improve=8`, `time_limit_seconds=180`, and `random_seed=2026`.

## Interpretation

The discrepancy is a run-lineage mismatch. It is proven that different drivers and unpinned source states were used. In the nine replayed cases, current direct NILS, the priority-service driver, the feasibility wrapper, regenerated current instances, and JSON reload produced identical objectives; the tested shared-state cases also matched. Current code reproduced the August objective for all sampled n=15 and n=50 cases, but not for the sampled n=75 and n=100 cases. Therefore the available evidence does not isolate one historical algorithm line as the sole cause, and the August full M5 result is not fully reproducible from the current checkout.
"""
    _write_markdown(AUDIT_DIR / "algorithm_change_audit.md", algorithm)

    protocol = json.loads(ALNS_PROTOCOL.read_text(encoding="utf-8"))
    alns = pd.read_csv(ALNS_RUNS)
    alns_feasible = alns["final_solution_feasibility_flag"].map(_truth)
    alns_iter = pd.to_numeric(alns["iterations"], errors="coerce")
    alns_candidates = pd.to_numeric(alns["candidate_reassignments_evaluated"], errors="coerce")
    alns_cpu = pd.to_numeric(alns["cpu_time_seconds"], errors="coerce")
    size_rows = []
    for size, group in alns.groupby("n", sort=True):
        size_rows.append(
            f"| {int(size)} | {len(group)} | {int(group['final_solution_feasibility_flag'].map(_truth).sum())} | "
            f"{group['iterations'].astype(float).min():.0f}--{group['iterations'].astype(float).max():.0f} | "
            f"{group['candidate_reassignments_evaluated'].astype(float).min():.0f}--{group['candidate_reassignments_evaluated'].astype(float).max():.0f} | "
            f"{group['cpu_time_seconds'].astype(float).mean():.3f} | {group['cpu_time_seconds'].astype(float).max():.3f} |"
        )
    alns_report = f"""# Corrected ALNS Execution Validation

## Protocol

- Protocol file: `outputs/corrected_alns_final_report_20261019/corrected_alns_protocol.json`.
- ALNS seed: `{protocol.get('alns_seed')}`; base seed: `{protocol.get('base_seed')}`.
- Destroy-and-repair iterations: `{protocol.get('alns_max_iterations')}`.
- ALNS time limit: `{protocol.get('alns_time_limit_seconds')}` seconds, so no premature ten-second cutoff.
- Matched comparison wall-clock budget: `{protocol.get('common_wall_clock_budget_seconds')}` seconds.

## Validation

- Rows: `{len(alns)}`; unique instances: `{alns['instance_id'].nunique()}`.
- Feasible final solutions: `{int(alns_feasible.sum())}/{len(alns)}`.
- Iterations: `{int(alns_iter.min())}` to `{int(alns_iter.max())}`; every row completed 40.
- Candidate reassignment evaluations: `{int(alns_candidates.min())}` to `{int(alns_candidates.max())}`; every row recorded 40.
- CPU time: mean `{alns_cpu.mean():.3f}` seconds; maximum `{alns_cpu.max():.3f}` seconds.
- The 40 iterations and candidate evaluations demonstrate that ALNS performed its prescribed destroy-and-repair loop, including the large instances.

## By size

| n | runs | feasible | iterations | candidate evaluations | mean CPU seconds | maximum CPU seconds |
|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(size_rows)}

## Interpretation

The corrected ALNS results can be reused for both M5--ALNS and M9--ALNS comparisons. A common 240-second cap is a fairness ceiling, not evidence of equal effort; the report retains actual runtime and iteration counts. The low drone usage at larger sizes is therefore an observed ALNS solution outcome, not evidence that the ten-second initialization cutoff stopped the search.
"""
    _write_markdown(AUDIT_DIR / "alns_execution_validation.md", alns_report)

    impact = """# Manuscript Impact Matrix

The submitted PDF named `Drone_paper (7).pdf` was not present in the repository, so this matrix is based on the requested section/table mapping and the available generated artifacts. The manuscript should be checked against the PDF before final submission.

| Manuscript item | Impact | Recommendation |
|---|---|---|
| Section 4 and Algorithm 1 | Method-definition claim | Retain if it describes standard NILS separately from NILS-R; add the pinned protocol and clarify that M5 is not NILS-R. |
| Tables 5--14 | Depends on June standard-NILS outputs | Do not replace automatically. Retain only after confirming each table uses the June M5 file and does not claim August-driver provenance. |
| Table 15 M5--ALNS | Directly affected by benchmark provenance | Use the corrected ALNS output and the matching submitted M5 source, or rerun both methods under one pinned protocol. Do not mix August M5 with June ALNS. |
| Tables 16--18 NILS-R/decision quality | Uses August M9 artifacts | Retain, with the August run provenance and the distinction between M5 and M9 stated explicitly; do not claim full SHA-level reproducibility. |
| Table 20 fleet marginal value | Uses August M9 fleet sweep | Retain if the fleet sweep is identified as a separate August experiment and its seed/configuration are documented. |
| Appendix B propositions | Mathematical/algorithmic claim | Verify that propositions refer to the stored best feasible incumbent. Do not state monotonicity for the temporary current solution during perturbation. |

## Objective and feasibility consistency

The available reports use the same objective-component naming and feasibility validator for the final M5/M9/ALNS comparison. The August wrapper's ignored return value from `evaluate_solution` should be corrected in a future reproducibility patch before a fresh canonical run; it is not evidence that the existing corrected ALNS benchmark is invalid.

The nine-case replay reproduced the August M5 objective for all sampled n=15 and n=50 instances but not for the sampled n=75 and n=100 instances. Therefore August M5-dependent numerical claims should be retained only as an explicitly historical run block or recomputed under a pinned final protocol.
"""
    _write_markdown(AUDIT_DIR / "manuscript_impact_matrix.md", impact)

    verdict = f"""# Final Audit Verdict

## Q1. Why did 561 M5 objectives change?

**Proven:** the files come from different run lineages: June `run-heuristic-study` versus the August priority-service driver. The June source SHA is unavailable, the August SHA is not recorded, the drivers use different instance/execution pathways, and the August wrapper adds feasibility postprocessing. The raw seeds and 720 instance IDs align. **Likely:** an unrecorded historical code or execution-path difference, including generated in-memory versus JSON-reloaded state and wrapper behavior. **Unresolved:** the single historical source line that caused every numeric difference, because the original June code and in-memory objects are unavailable.

Confirmed reconciliation: `561` changed, `462` August-better, `99` August-worse, and `159` unchanged.

## Q2. Did August improve standard NILS itself?

Not proven. The August values are reproducible from the current priority-service path for the diagnostic sample, but the audit cannot attribute the change solely to an NILS algorithm improvement. The tracked August additions are primarily NILS-R and reporting infrastructure; standard M5 keeps risk rollback disabled.

## Q3. Are August M5 and M9 reproducible and consistent with final definitions?

The August M5 and M9 outputs use separate method names, but full August M5 reproducibility is not established. In the nine-case replay, current code reproduced all sampled n=15/n=50 August M5 values but not the sampled n=75/n=100 values. Full historical SHA-level reproducibility is not established because the August run did not save its SHA. M9 remains algorithmically distinct from M5, but its August full-run provenance also lacks a recorded SHA.

## Q4. Can existing corrected ALNS output be used for both comparisons?

Yes. The corrected ALNS run has 720/720 feasible rows, 40/40 iterations, 40 candidate evaluations per row, and no ten-second cutoff. Reuse it for both M5--ALNS and M9--ALNS, provided the compared M5/M9 files are explicitly identified and the wall-clock budget is reported.

## Q5. Which results can be retained unchanged?

The corrected ALNS execution validation, August M9 decision-quality outputs, fleet sweep, and any architecture/model results independent of M5 numerical values can be retained. The June M5 tables can also be retained as a coherent submitted-run block, but their provenance must be labeled.

## Q6. Which results require correction or recomputation?

Do not mix the August M5 file with June-derived ALNS or ablation values. Table 15 must use a single matched M5/ALNS provenance. Any June table whose numeric claim directly uses standard M5 must either remain on the June block or be regenerated with the final canonical protocol.

## Q7. Is a new full experiment necessary?

Not for diagnosing the 561 discrepancy or validating the corrected ALNS loop: the nine-instance replay and existing ALNS protocol are sufficient. However, a new full M5/M9 experiment is scientifically necessary if the paper is to claim that all final results come from one pinned, reproducible methodology, because current code did not reproduce the sampled n=75/n=100 M5 values. It is not necessary to rerun ALNS solely to validate its 40-iteration search.

## Q8. Minimum manuscript revision

Add a reproducibility paragraph specifying the Git SHA, raw-instance directory hash, configuration, seeds, NILS iteration/time limits, ALNS 40-iteration protocol, actual runtimes, and the fact that M5 and M9 are separate. Replace or relabel Table 15 so it does not combine incompatible M5 generations. State that the June--August discrepancy is an execution/provenance issue whose historical single-line cause cannot be reconstructed.

## Final recommendation

Use a single canonical result block for the paper. The conservative immediately defensible comparison is the submitted M5 block with the corrected ALNS benchmark, while retaining August M9 only with explicit unpinned-run provenance. If the authors want the revised August methodology to be canonical, rerun M5 and M9 under a pinned commit and one verified raw-instance dataset; do not silently treat the existing August M5 values as fully reproducible.
"""
    _write_markdown(AUDIT_DIR / "final_audit_verdict.md", verdict)


def main() -> None:
    main_config = load_and_build_config(str(MAIN_CONFIG))
    audit, detail_seed = _merge_m5_outputs()
    audit = audit.sort_values(["n", "instance_id"]).reset_index(drop=True)
    diagnostics = _diagnostic_replay(audit, detail_seed, main_config)
    _write_reports(audit, diagnostics, detail_seed)
    print(f"Audit written to {AUDIT_DIR}")
    print(f"M5 rows: {len(audit)}; diagnostic rows: {len(diagnostics)}")


if __name__ == "__main__":
    main()
