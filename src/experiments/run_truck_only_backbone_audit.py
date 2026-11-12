from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
import subprocess
import time
from copy import deepcopy
from pathlib import Path
from typing import Any

from ..data_models import InstanceData, SolutionData
from ..drone_state import route_intersection_events
from ..feasibility import validate_solution
from ..heuristics.nils import _sync_route_assignments, evaluate_solution
from ..instance_generator import load_instance_json


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _routes_sha256(routes: dict[int, list[int]]) -> str:
    payload = json.dumps(
        {str(truck): route for truck, route in sorted(routes.items())},
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _load_backbone(path: Path) -> tuple[dict[int, list[int]], dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    routes = {
        int(truck): [int(node) for node in route]
        for truck, route in payload["truck_routes"].items()
    }
    if payload.get("truck_routes_sha256") != _routes_sha256(routes):
        raise RuntimeError(f"Persisted route hash mismatch: {path}")
    if payload.get("initialization_feasible") is not True:
        raise RuntimeError(f"Persisted backbone not marked feasible: {path}")
    return routes, payload


def _truck_reference(instance: InstanceData, routes: dict[int, list[int]]) -> SolutionData:
    solution = SolutionData(
        instance_name=instance.name,
        status="truck_only_shared_backbone_reference",
        objective=0.0,
        components={},
        truck_routes=deepcopy(routes),
        drone_routes={drone_id: [0, 0] for drone_id in instance.drone_ids()},
    )
    _sync_route_assignments(solution, instance)
    solution.objective, solution.components = evaluate_solution(instance, solution)
    return solution


def _atomic_json(path: Path, payload: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    columns = list(dict.fromkeys(key for row in rows for key in row))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def run_audit(repo: Path, source_dir: Path, output_dir: Path) -> dict[str, Any]:
    source_dir = source_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = source_dir / "raw"
    backbone_dir = raw_dir / "initial_truck_backbones"
    instance_paths = {path.stem: path for path in raw_dir.glob("S*.json")}
    backbone_paths = {path.stem: path for path in backbone_dir.glob("S*.json")}
    if len(instance_paths) != 720 or len(backbone_paths) != 720:
        raise RuntimeError(
            f"Expected exactly 720 saved instances and backbones; found "
            f"{len(instance_paths)} and {len(backbone_paths)}"
        )
    if set(instance_paths) != set(backbone_paths):
        raise RuntimeError("The frozen instance and backbone ID sets do not match")

    checkpoint_path = output_dir / "truck_only_720_checkpoint.json"
    if checkpoint_path.is_file():
        checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))
    else:
        checkpoint = {"rows": []}
    rows = checkpoint.setdefault("rows", [])
    completed = {row["instance_id"] for row in rows}
    source_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True
    ).strip()
    started = time.monotonic()

    for instance_id in sorted(instance_paths):
        if instance_id in completed:
            continue
        instance_path = instance_paths[instance_id]
        backbone_path = backbone_paths[instance_id]
        instance = load_instance_json(instance_path)
        routes, backbone_payload = _load_backbone(backbone_path)
        if set(routes) != set(instance.truck_ids()):
            raise RuntimeError(f"Backbone truck IDs do not match instance {instance_id}")
        run_started = time.monotonic()
        solution = _truck_reference(instance, routes)
        findings = validate_solution(instance, deepcopy(solution))
        errors = [
            f"{item.code}: {item.message}"
            for item in findings
            if item.severity == "ERROR"
        ]
        rows.append(
            {
                "instance_id": instance_id,
                "size": instance.num_customers,
                "seed": instance.seed,
                "scenario_class_id": instance.metadata.get("scenario_id"),
                "method": "truck_only_shared_backbone",
                "objective": float(solution.objective),
                "objective_components": json.dumps(solution.components, sort_keys=True),
                "feasible": not errors,
                "feasibility_error_count": len(errors),
                "feasibility_errors": json.dumps(errors),
                "route_intersection_count": len(route_intersection_events(solution)),
                "backbone_routes_sha256": backbone_payload["truck_routes_sha256"],
                "backbone_file_sha256": _sha256(backbone_path),
                "instance_file_sha256": _sha256(instance_path),
                "cpu_seconds": time.monotonic() - run_started,
            }
        )
        completed.add(instance_id)
        checkpoint.update(
            {
                "execution_commit": source_commit,
                "source_dir": str(source_dir),
                "completed_instances": len(rows),
                "expected_instances": 720,
                "updated_at_elapsed_seconds": time.monotonic() - started,
            }
        )
        _atomic_json(checkpoint_path, checkpoint)
        _write_csv(output_dir / "truck_only_720_backbone_audit.csv", rows)
        if len(rows) % 25 == 0 or len(rows) == 720:
            print(
                f"truck-only reference audit: {len(rows)}/720 complete; "
                f"feasible={sum(bool(row['feasible']) for row in rows)}; "
                f"elapsed={time.monotonic() - started:.1f}s",
                flush=True,
            )

    rows.sort(key=lambda row: row["instance_id"])
    by_size: list[dict[str, Any]] = []
    for size in sorted({int(row["size"]) for row in rows}):
        group = [row for row in rows if int(row["size"]) == size]
        objectives = [float(row["objective"]) for row in group]
        by_size.append(
            {
                "size": size,
                "instances": len(group),
                "feasible": sum(bool(row["feasible"]) for row in group),
                "infeasible": sum(not bool(row["feasible"]) for row in group),
                "mean_objective": statistics.fmean(objectives) if objectives else None,
                "median_objective": statistics.median(objectives) if objectives else None,
                "mean_cpu_seconds": statistics.fmean(float(row["cpu_seconds"]) for row in group),
            }
        )
    failures = [row for row in rows if not row["feasible"]]
    report = {
        "execution_commit": source_commit,
        "source_dir": str(source_dir),
        "output_dir": str(output_dir.resolve()),
        "instance_count": len(instance_paths),
        "backbone_count": len(backbone_paths),
        "completed_count": len(rows),
        "feasible_count": sum(bool(row["feasible"]) for row in rows),
        "infeasible_count": len(failures),
        "all_720_feasible": len(rows) == 720 and not failures,
        "full_method_rerun_authorized_by_this_script": False,
        "status": "PASS_REFERENCE_AUDIT" if len(rows) == 720 and not failures else "STOP_INFEASIBLE_REFERENCE",
        "failures": failures,
        "by_size": by_size,
        "total_cpu_seconds": sum(float(row["cpu_seconds"]) for row in rows),
        "wall_seconds": time.monotonic() - started,
    }
    _atomic_json(output_dir / "truck_only_720_backbone_report.json", report)
    _write_csv(output_dir / "truck_only_720_backbone_audit.csv", rows)
    _write_csv(output_dir / "truck_only_720_by_size.csv", by_size)
    lines = [
        "# Matched truck-only backbone audit",
        "",
        f"- Execution commit: `{source_commit}`",
        f"- Matched instances/backbones: {len(rows)}/720",
        f"- Feasible: {report['feasible_count']}; infeasible: {report['infeasible_count']}",
        f"- Wall time: {report['wall_seconds']:.2f} seconds",
        f"- Status: **{report['status']}**",
        "- This audit does not run M3/M4/M5 or ALNS.",
        "",
        "| Size | Instances | Feasible | Infeasible | Mean objective | Median objective |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for row in by_size:
        lines.append(
            f"| {row['size']} | {row['instances']} | {row['feasible']} | {row['infeasible']} | "
            f"{row['mean_objective']:.6f} | {row['median_objective']:.6f} |"
        )
    if failures:
        lines.extend(["", "## Infeasible references", ""])
        for row in failures:
            lines.append(f"- `{row['instance_id']}`: {row['feasibility_errors']}")
    (output_dir / "truck_only_720_backbone_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument(
        "--source",
        type=Path,
        default=Path("outputs/full_m3_m4_m5_alns_720_frozen_20261106"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/manuscript_semantics_reconciliation_20261112/truck_only_720"),
    )
    args = parser.parse_args()
    repo = args.repo.resolve()
    source = args.source if args.source.is_absolute() else repo / args.source
    output = args.output if args.output.is_absolute() else repo / args.output
    print(json.dumps(run_audit(repo, source, output), indent=2))


if __name__ == "__main__":
    main()
