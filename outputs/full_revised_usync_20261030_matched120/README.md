# Full Matched 120-Second Study

## Run scope

- Design: 720 instances (72 scenarios, 10 replications each).
- Methods: `truck_only`, `nils`, `feasibility_aware_alns`, and `nils_r`.
- Run window: 2026-10-30 08:38:50 through 2026-11-01 00:15:47, America/Mexico_City.
- Configuration: `config/experiment_full_revised_usync_20261030_budgeted.yaml`.
- NILS and NILS-R: maximum 25 outer iterations, 8 non-improving iterations, and a configured 120-second time limit.
- ALNS: maximum 40 iterations and a configured 120-second time limit.
- NILS-R reconstruction: at most one attempt, stagnation trigger 8, rollback size cap 3.

The time limits are cooperative deadlines checked during search; they are not external process-kill limits. Some runs exceed 120 seconds slightly while an operation already in progress finishes. Use the recorded CPU times when reporting computational effort.

## Completeness and feasibility

- `tables/heuristic_study_runs.csv` contains 2,880 data rows: 720 instances for each of four methods.
- The checkpoint contains the same 2,880 method rows.
- Feasible solutions: NILS 720/720, feasibility-aware ALNS 720/720, NILS-R 720/720, and truck-only 483/720.
- The 237 infeasible truck-only outcomes are retained and reported as infeasible; feasible-only aggregate comparisons exclude them. See `tables/table_03_overall_performance.csv` and `tables/table_04_performance_by_size.csv`.

## Contents

- `raw/`: generated instance data.
- `checkpoints/heuristic_study_rows.jsonl`: append-only method-level results.
- `logs/`: runner, progress, and command summaries.
- `tables/` and `figures/`: generated aggregate outputs.

## Provenance note

The run was not stamped with a Git commit SHA at launch. The matched-120 configuration, source code, logs, raw instances, checkpoint, and results are included in the repository publication for audit and reproduction. The supervisory chunk size changed during execution; solver settings and method list did not change. The older `full_revised_usync_20261021` and partial `full_revised_usync_20261030_budgeted` output folders are intentionally excluded because they are not the final matched-120 run.

This run generates aggregate study tables. It does not contain the separate customer-level delivery and accepted-drone-move diagnostic datasets requested for additional practitioner analyses.
