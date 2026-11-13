# Manuscript Semantics Reconciliation

This report covers targeted implementation validation and the matched truck-only reference audit. It does not report a full 720-instance M3/M4/M5/ALNS rerun.

## Historical Support-Action Counterfactual

The saved historical routes are held fixed; only state-required `z1`/`z2` actions are substituted.
- Historical target runs: 32; NILS-family runs: 24.
- NILS-family `z1` events forced by launch labeling: 26.
- NILS-family `z2` events forced by recovery labeling: 66.
- Objective changed in 18 runs; feasibility changed in 0 runs.
- Sum of objective deltas: -34025.812905 (diagnostic-only arithmetic total).

## Updated Targeted Runs

- Runs: 32; feasible: 32; architecture-validator passes: 32.
- Exact route-intersection / `u_sync` match: 32/32.
- Counts across these runs: `u_sync`=89, `z1`=22, `z2`=1.
- State-unnecessary support: `z1`=0, `z2`=0; missing required support: `z1`=0, `z2`=0.
- M3 unpaired-event violations: 0; M4 cross-truck-recovery violations: 0.
- M5 cross-truck sorties by method: `{"feasibility_aware_alns": 0, "nils": 9}`.
- NILS-family unnecessary launch reloads: 0; unnecessary recovery swaps: 0.
- ALNS runs pass M5 validator: True.
- ALNS sortie-type counts: `{"depot-depot": 4}`.
- Reconstructed `u_sync` invalid/discontinuous runs: 0.
- M3/M4/M5 reused the same saved initial backbone: 24/24 runs.
- Per-instance M3/M4/M5 backbone hashes match the persisted input: 8/8 instances.
- Rendezvous events without reload or swap: 67/89.
- Synchronization handling/waiting: 163.000/7928.428 minutes total; event-level readiness times are in `rendezvous_events.csv`.
- Targeted pre-rerun gate: **PASS**.
- Sortie-type counts by method:
  - feasibility_aware_alns: `{"depot-depot": 4}`.
  - nils: `{"cross-truck": 9, "depot-truck": 7, "same-truck": 9, "truck-depot": 5}`.
  - no_unpairing: `{"depot-depot": 1, "depot-truck": 7, "same-truck": 15, "truck-depot": 1}`.
  - paired_baseline: `{"depot-truck": 5, "same-truck": 9, "truck-depot": 2}`.

| Method | Runs | Feasible | Architecture pass | Mean objective | Mean CPU (s) |
|---|---:|---:|---:|---:|---:|
| feasibility_aware_alns | 8 | 8 | 8 | 52127.447405 | 68.20 |
| nils | 8 | 8 | 8 | 81056.663816 | 43.56 |
| no_unpairing | 8 | 8 | 8 | 72557.097229 | 20.88 |
| paired_baseline | 8 | 8 | 8 | 76543.674599 | 13.03 |

## Matched Truck-Only Reference

- Evaluated saved backbones: 720/720.
- Feasible: 720; infeasible: 0.
- Audit status: **PASS_REFERENCE_AUDIT**.
- No full M3/M4/M5/ALNS experiment was run.

| Size | Instances | Feasible | Infeasible | Mean objective | Median objective |
|---:|---:|---:|---:|---:|---:|
| 15 | 230 | 230 | 0 | 6196.523223 | 1462.150377 |
| 50 | 210 | 210 | 0 | 123678.707931 | 95350.344623 |
| 75 | 90 | 90 | 0 | 126827.793032 | 51979.798276 |
| 100 | 190 | 190 | 0 | 342742.403262 | 324347.862664 |
