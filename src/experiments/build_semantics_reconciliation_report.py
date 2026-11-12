from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Any


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def _json_list(value: str | None) -> list[Any]:
    if not value:
        return []
    try:
        parsed = json.loads(value)
    except (json.JSONDecodeError, TypeError):
        return []
    return parsed if isinstance(parsed, list) else []


def _as_bool(value: str | None) -> bool:
    return str(value).lower() in {"true", "1", "yes"}


def build_report(
    old_diagnostic_dir: Path,
    targeted_dir: Path,
    truck_only_dir: Path,
    output_dir: Path,
) -> dict[str, Any]:
    old_methods = _read_csv(old_diagnostic_dir / "method_summary.csv")
    old_counterfactuals = _read_csv(old_diagnostic_dir / "support_counterfactuals.csv")
    new_methods = _read_csv(targeted_dir / "method_summary.csv")
    new_sorties = _read_csv(targeted_dir / "sorties.csv")
    new_rendezvous = _read_csv(targeted_dir / "rendezvous_events.csv")
    new_support_events = _read_csv(targeted_dir / "support_events.csv")
    diagnostic = json.loads(
        (targeted_dir / "event_semantics_diagnostic.json").read_text(encoding="utf-8")
    )
    truck_report = json.loads(
        (truck_only_dir / "truck_only_720_backbone_report.json").read_text(encoding="utf-8")
    )

    old_nils_family = [row for row in old_methods if row["method"] in {
        "paired_baseline", "no_unpairing", "nils"
    }]
    old_forced = {
        method: {
            "z1_forced_by_launch_label": sum(
                int(float(row.get("forced_launch_label_z1_count") or 0))
                for row in old_methods if row["method"] == method
            ),
            "z2_forced_by_recovery_label": sum(
                int(float(row.get("forced_recovery_label_z2_count") or 0))
                for row in old_methods if row["method"] == method
            ),
        }
        for method in sorted({row["method"] for row in old_methods})
    }
    changed_objective = [
        row for row in old_counterfactuals
        if abs(float(row.get("objective_delta") or 0.0)) > 1e-8
    ]
    changed_feasibility = [
        row for row in old_counterfactuals
        if str(row.get("feasible_before")) != str(row.get("feasible_after"))
    ]
    deltas = [float(row.get("objective_delta") or 0.0) for row in old_counterfactuals]

    new_feasible = [row for row in new_methods if row["full_feasible"].lower() == "true"]
    architecture_pass = [
        row for row in new_methods if not _json_list(row.get("architecture_violations"))
    ]
    u_sync_matches = sum(
        not _json_list(row.get("intersection_events_without_explicit_u_sync"))
        and not _json_list(row.get("explicit_u_sync_without_route_intersection"))
        for row in new_methods
    )
    nils_family_new = [row for row in new_methods if row["method"] in {
        "paired_baseline", "no_unpairing", "nils"
    }]
    new_forced_z1 = sum(
        int(float(row.get("forced_launch_label_z1_count") or 0))
        for row in nils_family_new
    )
    new_forced_z2 = sum(
        int(float(row.get("forced_recovery_label_z2_count") or 0))
        for row in nils_family_new
    )
    alns_methods = [row for row in new_methods if row["method"] == "feasibility_aware_alns"]
    alns_sortie_types = Counter(
        row["sortie_type"] for row in new_sorties
        if row["method"] == "feasibility_aware_alns"
    )
    sortie_types_by_method = {
        method: dict(sorted(Counter(
            row["sortie_type"] for row in new_sorties if row["method"] == method
        ).items()))
        for method in sorted({row["method"] for row in new_methods})
    }
    rendezvous_by_method = {}
    for method in sorted({row["method"] for row in new_methods}):
        events = [row for row in new_rendezvous if row["method"] == method]
        rendezvous_by_method[method] = {
            "events": len(events),
            "without_reload_or_swap": sum(
                not _as_bool(row.get("z1")) and not _as_bool(row.get("z2"))
                for row in events
            ),
            "with_reload": sum(_as_bool(row.get("z1")) for row in events),
            "with_swap": sum(_as_bool(row.get("z2")) for row in events),
            "handling_minutes": sum(float(row.get("handling_minutes") or 0.0) for row in events),
            "truck_wait_minutes": sum(float(row.get("truck_wait_minutes") or 0.0) for row in events),
            "drone_wait_minutes": sum(float(row.get("drone_wait_minutes") or 0.0) for row in events),
        }
    support_actions_by_method = {
        method: dict(sorted(Counter(
            row["event_type"] for row in new_support_events if row["method"] == method
        ).items()))
        for method in sorted({row["method"] for row in new_methods})
    }
    nils_backbone_runs = [
        row for row in new_methods
        if row["method"] in {"paired_baseline", "no_unpairing", "nils"}
    ]
    by_method: dict[str, dict[str, Any]] = {}
    for method in sorted({row["method"] for row in new_methods}):
        group = [row for row in new_methods if row["method"] == method]
        by_method[method] = {
            "runs": len(group),
            "feasible": sum(row["full_feasible"].lower() == "true" for row in group),
            "architecture_pass": sum(not _json_list(row.get("architecture_violations")) for row in group),
            "mean_objective": sum(float(row["recomputed_objective"]) for row in group) / len(group),
            "mean_cpu_seconds": sum(float(row["run_time_seconds"]) for row in group) / len(group),
            "forced_z1_count": sum(int(float(row.get("forced_launch_label_z1_count") or 0)) for row in group),
            "forced_z2_count": sum(int(float(row.get("forced_recovery_label_z2_count") or 0)) for row in group),
        }

    report = {
        "purpose": "Targeted implementation validation and matched truck-only reference audit; not the full method experiment.",
        "historical_fixed_route_support_counterfactual": {
            "source_directory": str(old_diagnostic_dir.resolve()),
            "old_target_method_runs": len(old_methods),
            "nils_family_runs": len(old_nils_family),
            "forced_support_events_by_method": old_forced,
            "nils_family_forced_z1_total": sum(
                int(float(row.get("forced_launch_label_z1_count") or 0))
                for row in old_nils_family
            ),
            "nils_family_forced_z2_total": sum(
                int(float(row.get("forced_recovery_label_z2_count") or 0))
                for row in old_nils_family
            ),
            "objective_changed_runs": len(changed_objective),
            "objective_changed_runs_nils_family": sum(row["method"] != "feasibility_aware_alns" for row in changed_objective),
            "feasibility_changed_runs": len(changed_feasibility),
            "objective_delta_sum": sum(deltas),
            "objective_delta_min": min(deltas, default=0.0),
            "objective_delta_max": max(deltas, default=0.0),
            "interpretation": "This comparison holds the previously saved routes fixed and changes only z1/z2 to state-required actions; it isolates the operating-convention correction from search changes.",
        },
        "new_targeted_run": {
            "source_directory": str(targeted_dir.resolve()),
            "method_runs": len(new_methods),
            "feasible_runs": len(new_feasible),
            "architecture_validator_passes": len(architecture_pass),
            "route_intersection_u_sync_exact_matches": u_sync_matches,
            "nils_family_runs": len(nils_family_new),
            "nils_family_forced_z1_count": new_forced_z1,
            "nils_family_forced_z2_count": new_forced_z2,
            "alns_runs": len(alns_methods),
            "alns_all_m5_validator_pass": bool(alns_methods) and all(
                not _json_list(row.get("architecture_violations")) for row in alns_methods
            ),
            "alns_sortie_type_distribution": dict(sorted(alns_sortie_types.items())),
            "sortie_types_by_method": sortie_types_by_method,
            "rendezvous_event_accounting_by_method": rendezvous_by_method,
            "support_action_counts_by_method": support_actions_by_method,
            "nils_family_shared_initial_backbone_reused_runs": sum(
                _as_bool(row.get("initial_backbone_reused")) for row in nils_backbone_runs
            ),
            "nils_family_runs": len(nils_backbone_runs),
            "reconstructed_u_sync_invalid_or_discontinuous_runs": diagnostic[
                "reconstructed_u_sync_invalid_or_discontinuous_runs"
            ],
            "by_method": by_method,
        },
        "truck_only_720_backbone_audit": truck_report,
        "full_720_method_experiment_run": False,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "manuscript_semantics_reconciliation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Manuscript Semantics Reconciliation",
        "",
        "This report covers targeted implementation validation and the matched truck-only reference audit. It does not report a full 720-instance M3/M4/M5/ALNS rerun.",
        "",
        "## Historical Support-Action Counterfactual",
        "",
        "The saved historical routes are held fixed; only state-required `z1`/`z2` actions are substituted.",
        f"- Historical target runs: {len(old_methods)}; NILS-family runs: {len(old_nils_family)}.",
        f"- NILS-family `z1` events forced by launch labeling: {report['historical_fixed_route_support_counterfactual']['nils_family_forced_z1_total']}.",
        f"- NILS-family `z2` events forced by recovery labeling: {report['historical_fixed_route_support_counterfactual']['nils_family_forced_z2_total']}.",
        f"- Objective changed in {len(changed_objective)} runs; feasibility changed in {len(changed_feasibility)} runs.",
        f"- Sum of objective deltas: {sum(deltas):.6f} (diagnostic-only arithmetic total).",
        "",
        "## Updated Targeted Runs",
        "",
        f"- Runs: {len(new_methods)}; feasible: {len(new_feasible)}; architecture-validator passes: {len(architecture_pass)}.",
        f"- Exact route-intersection / `u_sync` match: {u_sync_matches}/{len(new_methods)}.",
        f"- NILS-family unnecessary launch reloads: {new_forced_z1}; unnecessary recovery swaps: {new_forced_z2}.",
        f"- ALNS runs pass M5 validator: {report['new_targeted_run']['alns_all_m5_validator_pass']}.",
        f"- ALNS sortie-type counts: `{json.dumps(dict(sorted(alns_sortie_types.items())), sort_keys=True)}`.",
        f"- Reconstructed `u_sync` invalid/discontinuous runs: {diagnostic['reconstructed_u_sync_invalid_or_discontinuous_runs']}.",
        f"- M3/M4/M5 reused the same saved initial backbone: {report['new_targeted_run']['nils_family_shared_initial_backbone_reused_runs']}/{len(nils_backbone_runs)} runs.",
        f"- Rendezvous events without reload or swap: {sum(row['without_reload_or_swap'] for row in rendezvous_by_method.values())}/{sum(row['events'] for row in rendezvous_by_method.values())}.",
        f"- Rendezvous handling charged: {sum(row['handling_minutes'] for row in rendezvous_by_method.values()):.3f} minutes total; readiness waiting is recorded per event in `rendezvous_events.csv`.",
        "- Sortie-type counts by method:",
        *[
            f"  - {method}: `{json.dumps(counts, sort_keys=True)}`."
            for method, counts in sortie_types_by_method.items()
        ],
        "",
        "| Method | Runs | Feasible | Architecture pass | Mean objective | Mean CPU (s) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for method, row in by_method.items():
        lines.append(
            f"| {method} | {row['runs']} | {row['feasible']} | {row['architecture_pass']} | "
            f"{row['mean_objective']:.6f} | {row['mean_cpu_seconds']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Matched Truck-Only Reference",
            "",
            f"- Evaluated saved backbones: {truck_report['completed_count']}/720.",
            f"- Feasible: {truck_report['feasible_count']}; infeasible: {truck_report['infeasible_count']}.",
            f"- Audit status: **{truck_report['status']}**.",
            "- No full M3/M4/M5/ALNS experiment was run.",
            "",
            "| Size | Instances | Feasible | Infeasible | Mean objective | Median objective |",
            "|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in truck_report["by_size"]:
        lines.append(
            f"| {row['size']} | {row['instances']} | {row['feasible']} | {row['infeasible']} | "
            f"{row['mean_objective']:.6f} | {row['median_objective']:.6f} |"
        )
    (output_dir / "manuscript_semantics_reconciliation_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--old-diagnostic", type=Path, required=True)
    parser.add_argument("--targeted", type=Path, required=True)
    parser.add_argument("--truck-only", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            build_report(args.old_diagnostic, args.targeted, args.truck_only, args.output),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
