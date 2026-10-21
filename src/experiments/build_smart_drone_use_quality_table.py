from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd


METHOD_ORDER = ["M5", "M8", "M9"]
SIZE_ORDER = ["All", "15", "50", "75", "100"]
METHOD_MAP = {
    "M5 NILS": "M5",
    "M8 Feasibility-aware ALNS": "M8",
    "M9 NILS-R": "M9",
}


def _read_source(path: Path, source_name: str) -> pd.DataFrame:
    frame = pd.read_csv(path)
    frame = frame[frame["method"].isin(METHOD_MAP)].copy()
    if frame.empty:
        raise ValueError(f"No M5/M8/M9 rows found in {source_name}")
    frame["method"] = frame["method"].map(METHOD_MAP)
    frame["size"] = frame["size"].astype(str)
    return frame


def build_table(use_path: Path, speed_path: Path, output_dir: Path) -> pd.DataFrame:
    use = _read_source(use_path, "drone-use table")
    speed = _read_source(speed_path, "priority-speed table")
    use_columns = [
        "method",
        "size",
        "feasible_runs",
        "mean_active_drones_used",
        "mean_drone_share",
        "mean_high_priority_served_by_drone_share",
    ]
    speed_columns = [
        "method",
        "size",
        "high_priority_on_time_rate",
        "mean_high_priority_delivery_time",
    ]
    merged = use[use_columns].merge(
        speed[speed_columns],
        on=["method", "size"],
        how="inner",
        validate="one_to_one",
    )
    expected = {(method, size) for method in METHOD_ORDER for size in SIZE_ORDER}
    observed = set(zip(merged["method"], merged["size"]))
    missing = sorted(expected - observed)
    extra = sorted(observed - expected)
    if missing or extra:
        raise ValueError(f"Unexpected table keys; missing={missing}, extra={extra}")
    merged["method_order"] = merged["method"].map({method: index for index, method in enumerate(METHOD_ORDER)})
    merged["size_order"] = merged["size"].map({size: index for index, size in enumerate(SIZE_ORDER)})
    merged = merged.sort_values(["size_order", "method_order"]).drop(columns=["method_order", "size_order"])
    output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = output_dir / "table_smart_drone_use_quality_by_size.csv"
    merged.to_csv(output_csv, index=False, float_format="%.10f")
    _write_latex(merged, output_dir / "table_smart_drone_use_quality_by_size.tex")
    provenance = {
        "drone_use_source": str(use_path),
        "priority_speed_source": str(speed_path),
        "methods": METHOD_ORDER,
        "sizes": SIZE_ORDER,
        "aggregation": "Values are copied from the feasible-run aggregates and joined by method and size.",
        "m8_provenance": "August route-metrics ALNS run used by the submitted n=100 diagnostic table; not the corrected October 40-iteration ALNS benchmark.",
    }
    (output_dir / "table_smart_drone_use_quality_by_size_provenance.json").write_text(
        json.dumps(provenance, indent=2), encoding="utf-8"
    )
    return merged


def _percent(value: float) -> str:
    return f"{100.0 * value:.1f}\\%"


def _write_latex(frame: pd.DataFrame, path: Path) -> None:
    lines = [
        r"\begin{table}[htbp]",
        r"\centering",
        r"\caption{Decision-quality diagnostics for drone-use intelligence by instance size}",
        r"\label{tab:smart_drone_use_quality_by_size}",
        r"\footnotesize",
        r"\begin{tabular}{lrrrrr}",
        r"\toprule",
        r"Method & Active drones & Drone share & HP drone share & HP on-time & Mean HP time \\",
        r"\midrule",
    ]
    for size in SIZE_ORDER:
        size_label = "Full design" if size == "All" else rf"$n={size}$"
        lines.append(rf"\multicolumn{{6}}{{c}}{{{size_label}}} \\")
        lines.append(r"\cmidrule(lr){1-6}")
        subset = frame[frame["size"].eq(size)]
        for row in subset.itertuples(index=False):
            lines.append(
                f"{row.method} & {row.mean_active_drones_used:.2f} & "
                f"{_percent(row.mean_drone_share)} & "
                f"{_percent(row.mean_high_priority_served_by_drone_share)} & "
                f"{_percent(row.high_priority_on_time_rate)} & "
                f"{row.mean_high_priority_delivery_time:.2f} \\\\" 
            )
        if size != SIZE_ORDER[-1]:
            lines.append(r"\addlinespace")
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}",
            r"\begin{flushleft}",
            r"\footnotesize",
            r"\textit{Note}: HP denotes high-priority customers. Each entry is averaged over feasible runs for the indicated size; the exact feasible-run counts and unrounded values are in the accompanying CSV. The M8 rows use the August route-metrics ALNS run that reproduces the submitted $n=100$ diagnostic row.",
            r"\end{flushleft}",
            r"\end{table}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone-use-source", type=Path, required=True)
    parser.add_argument("--priority-speed-source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    frame = build_table(args.drone_use_source, args.priority_speed_source, args.output_dir)
    print(frame.to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
