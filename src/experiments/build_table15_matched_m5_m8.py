"""Build a full matched M5-NILS versus M8-ALNS Table 15."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from .statistics import paired_comparison


M5 = "nils"
M8 = "feasibility_aware_alns"
TIE_TOLERANCE = 1e-6
EXPECTED_SIZES = (15, 50, 75, 100)


def _as_bool(values: pd.Series) -> pd.Series:
    if values.dtype == bool:
        return values.fillna(False)
    return values.astype(str).str.strip().str.lower().isin({"1", "true", "t", "yes"})


def _finite_mean(values: pd.Series) -> float:
    numeric = pd.to_numeric(values, errors="coerce").to_numpy(dtype=float)
    numeric = numeric[np.isfinite(numeric)]
    return float(np.mean(numeric)) if numeric.size else float("nan")


def _finite_median(values: pd.Series) -> float:
    numeric = pd.to_numeric(values, errors="coerce").to_numpy(dtype=float)
    numeric = numeric[np.isfinite(numeric)]
    return float(np.median(numeric)) if numeric.size else float("nan")


def _pair_frame(run_status: pd.DataFrame) -> pd.DataFrame:
    required = {
        "instance_id", "scenario_id", "n", "method_name", "feasible", "error", "objective",
        "cpu_time_seconds", "high_priority_tardiness", "number_of_drone_launches", "drone_share",
    }
    missing = sorted(required - set(run_status.columns))
    if missing:
        raise ValueError(f"Run status is missing columns: {missing}")
    frame = run_status.copy()
    frame["feasible"] = _as_bool(frame["feasible"])
    if frame["error"].fillna("").astype(str).str.strip().ne("").any():
        raise ValueError("Run status contains failed method executions")
    frame = frame[frame["method_name"].isin({M5, M8})].copy()
    counts = frame.groupby("method_name")["instance_id"].nunique().to_dict()
    if counts != {M5: 720, M8: 720}:
        raise ValueError(f"Expected 720 runs per method; found {counts}")
    if frame.duplicated(["instance_id", "method_name"]).any():
        raise ValueError("Duplicate instance-method records found")
    index = ["instance_id", "scenario_id", "n"]
    m5 = frame[frame["method_name"].eq(M5)].set_index(index)
    m8 = frame[frame["method_name"].eq(M8)].set_index(index)
    pairs = m5[
        ["feasible", "objective", "cpu_time_seconds", "high_priority_tardiness", "number_of_drone_launches", "drone_share"]
    ].join(
        m8[
            ["feasible", "objective", "cpu_time_seconds", "high_priority_tardiness", "number_of_drone_launches", "drone_share"]
        ],
        how="outer",
        lsuffix="_M5",
        rsuffix="_M8",
        validate="one_to_one",
    ).reset_index()
    pairs["pair_feasible"] = pairs["feasible_M5"] & pairs["feasible_M8"]
    pairs["objective_difference_M5_minus_M8"] = pairs["objective_M5"] - pairs["objective_M8"]
    pairs["objective_improvement_M5_vs_M8_pct"] = 100.0 * (
        pairs["objective_M8"] - pairs["objective_M5"]
    ) / pairs["objective_M8"].replace(0.0, np.nan)
    pairs["HPT_difference_M5_minus_M8"] = (
        pairs["high_priority_tardiness_M5"] - pairs["high_priority_tardiness_M8"]
    )
    return pairs


def _summarize(pairs: pd.DataFrame, size: str) -> dict[str, object]:
    subset = pairs if size == "All" else pairs[pairs["n"].eq(int(size))]
    feasible = subset[subset["pair_feasible"]].copy()
    objective_diff = feasible["objective_difference_M5_minus_M8"]
    hpt_diff = feasible["HPT_difference_M5_minus_M8"]
    valid_hpt = hpt_diff.notna()
    test = paired_comparison(feasible["objective_M5"], feasible["objective_M8"])
    return {
        "size": size,
        "pairs": int(len(feasible)),
        "M5_objective_wins": int((objective_diff < -TIE_TOLERANCE).sum()),
        "M8_objective_wins": int((objective_diff > TIE_TOLERANCE).sum()),
        "objective_ties": int((objective_diff.abs() <= TIE_TOLERANCE).sum()),
        "mean_Z_M5": _finite_mean(feasible["objective_M5"]),
        "mean_Z_M8": _finite_mean(feasible["objective_M8"]),
        "median_Z_M5": _finite_median(feasible["objective_M5"]),
        "median_Z_M8": _finite_median(feasible["objective_M8"]),
        "mean_improvement_M5_vs_M8_pct": _finite_mean(feasible["objective_improvement_M5_vs_M8_pct"]),
        "median_improvement_M5_vs_M8_pct": _finite_median(feasible["objective_improvement_M5_vs_M8_pct"]),
        "wilcoxon_test": test.test,
        "wilcoxon_p": float(test.p_value),
        "mean_cpu_M5": _finite_mean(subset["cpu_time_seconds_M5"]),
        "mean_cpu_M8": _finite_mean(subset["cpu_time_seconds_M8"]),
        "feasible_runs_M5": int(subset["feasible_M5"].sum()),
        "feasible_runs_M8": int(subset["feasible_M8"].sum()),
        "feasibility_rate_M5": float(subset["feasible_M5"].mean()),
        "feasibility_rate_M8": float(subset["feasible_M8"].mean()),
        "mean_HPT_M5": _finite_mean(feasible["high_priority_tardiness_M5"]),
        "mean_HPT_M8": _finite_mean(feasible["high_priority_tardiness_M8"]),
        "HPT_M5_wins": int((hpt_diff[valid_hpt] < -TIE_TOLERANCE).sum()),
        "HPT_M8_wins": int((hpt_diff[valid_hpt] > TIE_TOLERANCE).sum()),
        "HPT_ties": int((hpt_diff[valid_hpt].abs() <= TIE_TOLERANCE).sum()),
        "mean_launches_M5": _finite_mean(feasible["number_of_drone_launches_M5"]),
        "mean_launches_M8": _finite_mean(feasible["number_of_drone_launches_M8"]),
        "mean_drone_share_M5_pct": 100.0 * _finite_mean(feasible["drone_share_M5"]),
        "mean_drone_share_M8_pct": 100.0 * _finite_mean(feasible["drone_share_M8"]),
    }


def _fmt(value: object, digits: int = 2) -> str:
    if pd.isna(value):
        return "--"
    if isinstance(value, (int, np.integer)):
        return f"{int(value)}"
    return f"{float(value):,.{digits}f}"


def _write_latex(summary: pd.DataFrame, path: Path) -> None:
    lines = [
        r"\begin{table*}[htbp]",
        r"\centering",
        r"\caption{Full matched comparison of NILS (M5) and feasibility-aware ALNS (M8)}",
        r"\label{tab:matched_m5_m8_full}",
        r"\scriptsize",
        r"\textbf{Panel A: Objective, statistical test, runtime, and feasibility}",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{lrrrrrrrrrrrrrr}",
        r"\toprule",
        r"Size & Pairs & M5 wins & M8 wins & Ties & Mean $Z_{M5}$ & Mean $Z_{M8}$ & Median $Z_{M5}$ & Median $Z_{M8}$ & Mean imp. (\%) & Median imp. (\%) & Wilcoxon $p$ & CPU M5 (s) & CPU M8 (s) & Feas. M5/M8 \\",
        r"\midrule",
    ]
    for row in summary.itertuples(index=False):
        feasibility = f"{100 * row.feasibility_rate_M5:.1f}/{100 * row.feasibility_rate_M8:.1f}\\%"
        p_value = "<0.001" if row.wilcoxon_p < 0.001 else _fmt(row.wilcoxon_p, 3)
        values = [
            row.size, str(row.pairs), str(row.M5_objective_wins), str(row.M8_objective_wins),
            str(row.objective_ties), _fmt(row.mean_Z_M5), _fmt(row.mean_Z_M8),
            _fmt(row.median_Z_M5), _fmt(row.median_Z_M8), _fmt(row.mean_improvement_M5_vs_M8_pct),
            _fmt(row.median_improvement_M5_vs_M8_pct), p_value, _fmt(row.mean_cpu_M5),
            _fmt(row.mean_cpu_M8), feasibility,
        ]
        lines.append(" & ".join(values) + r" \\")
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}}",
            r"\vspace{0.5em}",
            r"\textbf{Panel B: Priority service and drone utilization}",
            r"\resizebox{\textwidth}{!}{%",
            r"\begin{tabular}{lrrrrrrrrr}",
            r"\toprule",
            r"Size & Mean HPT M5 & Mean HPT M8 & HPT wins M5 & HPT wins M8 & HPT ties & Launches M5 & Launches M8 & Drone share M5 (\%) & Drone share M8 (\%) \\",
            r"\midrule",
        ]
    )
    for row in summary.itertuples(index=False):
        values = [
            row.size, _fmt(row.mean_HPT_M5), _fmt(row.mean_HPT_M8), str(row.HPT_M5_wins),
            str(row.HPT_M8_wins), str(row.HPT_ties), _fmt(row.mean_launches_M5),
            _fmt(row.mean_launches_M8), _fmt(row.mean_drone_share_M5_pct),
            _fmt(row.mean_drone_share_M8_pct),
        ]
        lines.append(" & ".join(values) + r" \\")
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}}",
            r"\begin{flushleft}",
            r"\footnotesize",
            r"\textit{Notes}: Each pair is run on the same instance. Positive improvement $100(Z_{M8}-Z_{M5})/Z_{M8}$ favors M5. Objective ties use tolerance $10^{-6}$. The Wilcoxon signed-rank test uses matched objective values. CPU is recorded runtime under the configured cooperative limits. Feasibility is reported separately; objective and service summaries use matched feasible runs. HPT is total high-priority tardiness in minutes.",
            r"\end{flushleft}",
            r"\end{table*}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build(run_status_path: Path, output_dir: Path) -> pd.DataFrame:
    run_status = pd.read_csv(run_status_path)
    pairs = _pair_frame(run_status)
    sizes = ["All", *[str(size) for size in EXPECTED_SIZES]]
    summary = pd.DataFrame([_summarize(pairs, size) for size in sizes])
    output_dir.mkdir(parents=True, exist_ok=True)
    summary.to_csv(output_dir / "table_15_matched_m5_m8.csv", index=False, float_format="%.12g")
    pairs.reset_index(drop=True).to_csv(output_dir / "table_15_matched_m5_m8_instance_detail.csv", index=False, float_format="%.12g")
    _write_latex(summary, output_dir / "table_15_matched_m5_m8.tex")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-status", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(build(args.run_status, args.output_dir).to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
