# Final computational evidence audit

**Status: BLOCKED for an unqualified exact-model reconciliation.** The M2/M6/M7 computation and all-method ablation analysis are complete; material MILP/evaluator and manuscript consistency issues remain. This is not a claim of submission readiness.

## Results at a glance

- M2, M6, and M7 ran on all 720 frozen instances: M2 has 487 valid schedules and 233 retained infeasible constructions; M6 and M7 each have 720 valid schedules.
- Lower-is-better size-balanced objective index ranking: M8 (35.25), M4 (39.41), M5 (41.29), M3 (44.24), M7 (52.38), M6 (57.00), M2 (97.62; conditional on 487 valid runs), M1 (100.00).
- Matched M5/M6: 563/151 wins and 6 ties; median M5 improvement 17.80%. The n=100 difference is not significant (100/190 versus 90/190; Wilcoxon p=0.496).
- Matched M5/M7: 523/46 wins and 151 ties; median M5 improvement 12.39%; M5 has more wins in each size class.
- M5 mean high-priority tardiness is 807.92, versus 982.96 for M6 and 996.64 for M7. These are finite-time ablation results, not causal estimates.
- Exact MILP audit: only 1/7 cases is certified optimal (n=3). The n=7 incumbent fails strict validation; n=9 has no feasible incumbent. The remaining feasible incumbents are time-limited and uncertified.

See `MANUSCRIPT_ACTIONS.md` for exact edit locations, existing/replacement wording, copy-ready LaTeX, evidence references, and unresolved blockers. It also records the 227 readiness-evaluator objective changes in archived M3/M4/M5 records; source schedules remain immutable.

## Reproducibility

`run_manifest.json` records the execution commit, manuscript/input provenance, environment, seeds, parameters, runtime/checkpoint history, and frozen-artifact hashes. `validation_manifest.json` records validation status. The completed raw file has 2,160 unique method-instance records with no duplicates. The principal frozen M1/M3/M4/M5/M8 results were not rerun or overwritten.

To reproduce summaries from the checkpointed raw results:

```powershell
python -m src.experiments.run_final_m2_m6_m7_720 --summarize
```

The full run commands are retained in `run_manifest.json`; do not launch `--run-full` unless intentionally repeating the long computation. Exact CBC logs and certificates in this package are reused from the prior seven-case audit, not new solves.

## Primary artifacts

- Full M1-M8 tables: `all_methods_M1_M8_summary.csv`, `all_methods_by_size.csv`, `all_methods_operational_indicators.csv`.
- Ablations: `M5_vs_M6_M7_matched.csv`, `M5_vs_M6_M7_by_size.csv`, `M5_vs_M6_M7_priority_service.csv`, `ablation_comparison.tex`, `ablation_report.md`.
- Manuscript tables/actions: `table5_exact_validation.tex`, `table6_all_methods.tex`, `table8_all_methods.tex`, `MANUSCRIPT_ACTIONS.md`.
- Raw new runs and diagnostics: `new_method_raw_results.jsonl`, `new_method_results_2160.csv`, `new_method_validation.csv`, `run_progress.json`.
- Exact audit: `exact_results.csv`, `exact_solver_certificates.csv`, `exact_model_code_matrix.csv`, `exact_objective_reconciliation.csv`, `exact_validation_report.md`, and `exact_solver_logs/`.

All invalid runs are retained and excluded from objective averages and matched win/loss/tie counts. M2 means and rankings use only its valid runs and must be labeled conditional; do not replace failures with truck-only schedules.
