# Final legacy-semantics refresh

This directory contains only the refreshed small exact-calibration and real-data lambda evidence. The definitive 720-instance experiment is untouched.

Run status: complete_with_integrity_findings. Final integrity failures: 4; recorded findings: 7.

## Reproduction

Run from the repository root with the recorded execution commit:

    python -m src.experiments.refresh_legacy_semantics

To regenerate derived tables, the claim audit, and the manifest from the already saved raw JSONL without rerunning any optimization, use:

    python -m src.experiments.refresh_legacy_semantics --finalize-existing

The runner checkpoints each NILS restart, exact solve, and real-data method to JSONL under raw/. It resumes completed records without replacing them. Raw solution payloads, frozen small-instance inputs, the 80-row real-data sample, and all eight scaled real-case instances are included.

## Semantics and validation

All new candidate solutions are evaluated with the current objective and checked with validate_solution plus validate_drone_architecture. M3 uses the fixed default drone-to-truck map, M4 uses the no-cross-truck validator, and M1/M2/M5/M6/M7 use the M5 feasibility architecture check. Only candidates passing both validators appear in the validated objective columns; rejected candidates and diagnostics remain in raw records.

The final timing evaluator recomputes synchronization waiting from readiness times. The current implementation treats u_sync as physical rendezvous, charges two handling events per truck rendezvous, and keeps parcel reload and battery swap as separate state-dependent events.

## Important audit finding

The archived small-calibration inputs have 3 trucks and 3 drones, even though the latest manuscript calls the cases single-truck/single-drone. The report uses the exact frozen files associated with the historical Table 5 values and flags this manuscript wording. It does not regenerate inputs.

The exact-gap table is not paper-ready: none of the seven exact optima is certified, and n=7, n=8, and n=9 have additional validation/objective/timeout blockers. The real-data rows at lambda=0.150 and 0.200 remain infeasible under the shift constraint, matching the historical feasibility flags.

## Files

- exact_calibration_results.csv: one row for each n=3,...,9, including exact/NILS objectives, gaps, runtimes, event counts, and validator results.
- exact_calibration_summary.csv: exact-match and gap summary.
- real_data_lambda_results.csv: one row for each of eight lambda values and M1-M7.
- historical_reconciliation.csv: explicit historical-versus-new value comparisons; historical values are never mixed into new result tables.
- claim_audit.md: manuscript claim classifications and the required MANUSCRIPT ACTIONS.
- run_manifest.json: execution commit, versions, commands, settings, input hashes, timestamps, and output inventory.
- raw/: append-only raw solver and heuristic outputs, including serialized solutions.
- inputs/: packaged frozen inputs and lambda-specific real-case instances.

Execution commit: dd9a85a05fa4a8badbc00275a15ce67622c5d220
Finalization commit: e334980e0a86da9eadca6c6a5c95311c81c9aa5f
Branch: codex/utdrp-latest
