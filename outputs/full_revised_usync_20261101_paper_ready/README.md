# Matched Paper-Ready Rerun

## Status

This is a fresh, sequential rerun of the 720-instance design. It is required because schedules from the earlier run were not persisted, and replaying time-limited NILS/NILS-R runs can produce different objectives. Do not combine this run's customer records with the earlier aggregate results.

## Important interpretation before manuscript use

- The fresh matched M5–M8 replacement for Table 15 is in `tables/table_15_matched_m5_m8.tex` and `.csv`; the old submitted table is not reproduced. See `paper_results_reconciliation.md` for the row-by-row comparison and required interpretation.
- Both methods had a configured 120-second limit, but the copied configuration sets NILS to at most 25 outer iterations and ALNS to at most 40 destroy/repair iterations. Their mean realized runtimes also differ materially (82.62 seconds for M5 versus 18.53 seconds for M8 overall). This is a matched-instance comparison, not a matched-realized-runtime comparison; do not make equal-computational-effort claims from it.
- The exporter overwrote search diagnostics while refreshing objective components, so `iterations` and related search counters in the historical aggregate file are zero and cannot be recovered from this run. The exporter has been fixed for future runs; that fix does not retroactively repair the saved counters.
- The separate available-fleet sweep is not part of this fresh run. Older August sweep files were excluded because they were produced under an earlier implementation and have not been validated against this run's timing convention.
- The NILS-R decision log records NILS-R rollback/admission decisions; it is not a trace of every candidate considered by M5 or M8.

## Design and methods

- 720 saved instances reused from `outputs/full_revised_usync_20261030_matched120/raw/`.
- Methods: M1 truck-only, M5 NILS, M8 feasibility-aware ALNS, and M9 NILS-R.
- Uses `config/experiment_full_revised_usync_20261030_budgeted.yaml` for both method configurations.
- NILS and NILS-R use a 120-second cooperative time limit; ALNS uses the configured 40 iterations and 120-second cooperative limit.
- Run source commit and configuration hashes are recorded in `run_provenance.json`.

## Recovery

The runner writes completed method rows and customer records after each instance. If interrupted, rerun the same command from the repository root; successful instance-method pairs are skipped and incomplete/error pairs are retried:

```powershell
python -u -m src.experiments.run_priority_service_speed_by_method `
  --main-config config/experiment_full_revised_usync_20261030_budgeted.yaml `
  --alns-config config/experiment_full_revised_usync_20261030_budgeted.yaml `
  --source-output-dir outputs/full_revised_usync_20261030_matched120 `
  --output-dir outputs/full_revised_usync_20261101_paper_ready `
  --methods truck_only nils feasibility_aware_alns nils_r
```

## Deliverables

- `customer_service_detail.csv`, `drone_decision_log.csv`, and `nils_r_reconstruction_log.csv` hold schedule-level diagnostics. The decision log records NILS-R's priority-risk rollback/admission decisions; it is not a trace of every internal candidate considered by M5 or M8.
- `table_priority_service_speed_by_method.csv` and the `tables/` folder hold summary results and regenerated paper tables.
- `replay_objective_audit.csv` compares this run with the earlier aggregate run; mismatches are expected for methods that hit their cooperative time limit.
- `run_status_partial.csv` and `paper_method_rows_partial.csv` are resumable checkpoints.
- `raw/` contains a copy of each input instance used.

After completion, generate the matched size-wise M9 comparisons with:

```powershell
python -m src.experiments.aggregate_nils_r_comparisons `
  --run-status outputs/full_revised_usync_20261101_paper_ready/run_status_partial.csv `
  --output-dir outputs/full_revised_usync_20261101_paper_ready/tables
```
