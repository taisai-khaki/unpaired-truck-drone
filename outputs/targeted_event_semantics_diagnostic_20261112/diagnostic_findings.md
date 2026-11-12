# Targeted Event-Semantics Diagnostic

**Date:** 2026-11-12
**Scope:** Eight purposively selected frozen instances; M3, M4, M5/NILS, ALNS, and a truck-only reference. This is an operational audit, not a performance sample. No 720-instance rerun was started.

## Design and provenance

- Instances: two each at (n=15,50,75,100), stratified by eligible share near 0.25 and 0.75. The M3/M4/M5 runs reused the exact persisted truck backbone for each instance; all 24 reused-backbone hashes match within instance. ALNS used its independent initializer.
- Runs: 32 method runs (8 instances × 4 methods), plus 8 truck-only evaluations on the saved backbone. Configured limits were NILS 120 s / 25 outer iterations / 8 no-improvement iterations / candidate caps 100/100, and ALNS 120 s / 40 iterations.
- Algorithm source commit: `cff0112f2eb2cc01212589ace2c9ada8b17d725e`. Config SHA-256: `3643d5e95a62d850319e627319b546b5254d3da68a7c85ac7b352caa14883924`. Diagnostic script SHA-256: `06649bac432b15b9b2f25e4e430de20909f8129397286a4e47102ee1516cc905`. No NILS, ALNS, evaluator, or feasibility implementation was changed for this audit.

## Findings

- All 32 returned solutions passed the full feasibility checker, the repository architecture validator, and the separate frozen-semantics architecture/continuity audit. The 24 M3/M4/M5 solutions had 92 explicit `u_sync` events; all 92 exactly matched common truck/drone route visits. No reconstructed event was invalid, ambiguous, or discontinuous.
- Architecture behavior was coherent on this subset: M3 had no unpaired-truck event; M4 had zero cross-truck recoveries; M5 had four valid cross-truck sorties among 40 sorties. Every recorded multi-sortie sequence continued from the prior recovery node/truck.
- Current NILS code forced `z1` at launch and `z2` at recovery. Among M5/NILS runs, 9 of 31 active `z1` events were not required by parcel state, and all 34 of 34 active `z2` events were not required by the battery-reserve state. Across M3/M4/M5 variants, the corresponding counts were 26 of 58 `z1` events and 66 of 66 `z2` events.
- Replacing those support flags with state-required flags while keeping the routes fixed changed no feasibility result. It changed objective in 18 of 24 M3/M4/M5 runs, always through tardiness cost; the largest change was (-12,225.24) for M5 at `S0068_R08`. Across these 24 distinct diagnostic runs, the arithmetic sum of objective deltas was (-34,025.81); this is not an estimate of a population effect. Readiness-based synchronization waiting was recomputed from the resulting schedules.
- The current architecture validator has an unconditional-reload rule at every nondepot launch. After the state correction, it flags 15 of 24 M3/M4/M5 counterfactuals for a missing reload even though all remain fully feasible and pass the frozen state-aware audit. This is a validator/code-paper mismatch that must be corrected before a full run.
- ALNS passed the current M5 validator on all 8 instances. It produced only 3 sorties total, all single-customer depot–depot sorties; it produced no truck rendezvous, `z1`, or `z2` events. These sorties are physically valid under the frozen convention, but the current ALNS insertion/repair search does not exercise truck-based or cross-truck M5 operations. Thus the sampled ALNS outputs are architecture-valid but its search coverage is narrower than the full M5 operating set; the result does not establish an equivalent M5 search space.
- The current-evaluator truck-only reference on each saved M3/M4/M5 backbone was feasible on all 8 instances. Rerunning the current `truck_only_baseline` constructor reproduced the historical objective and feasibility status to numerical precision on all 8. Its route hash matches the shared backbone at (n=15,50), but differs at (n=75,100). This isolates the discrepancy to the route reference, not the evaluator: historical rows use the constructor route, while a matched reference evaluates the shared persisted backbone. The historical constructor is infeasible at (n=75,100); the shared-backbone reference is feasible in these selected cases. A fresh matched truck-only reference is required for the paper tables and can be evaluated from all saved backbones without rerunning NILS/ALNS.

## Required before 720

1. Make NILS `z1`/`z2` decisions depend on parcel and battery states, not launch/recovery labels.
2. Change the common architecture validator to check parcel continuity rather than requiring a reload at every nondepot launch; explicitly check M3 truck identity for `u_sync`, `z1`, and `z2`.
3. Extend ALNS destroy/repair to construct and preserve feasible M5 truck-based sortie sequences, or label its benchmark explicitly as depot-only/restricted. Do not treat the current 8/8 validator pass as evidence that it searches the full M5 operating set.
4. Repeat targeted state/event tests, then create a current-evaluator truck-only reference for the 720 saved backbones before any full comparative rerun.

## Files

- `event_semantics_diagnostic.json`: run-level findings, schedules, counterfactuals, hashes.
- `method_summary.csv`: method feasibility, objective components, support counts, validator results.
- `sorties.csv`: sortie sequences, launch/recovery nodes and trucks, continuity.
- `rendezvous_events.csv`: every `u_sync` event, readiness, handling and realized waiting.
- `support_events.csv`: every active `z1`/`z2` event and state-based reason.
- `drone_legs.csv`: loaded/empty state and battery use on every drone leg.
- `support_counterfactuals.csv`: objective and feasibility changes after state-based support correction.
- `truck_only_backbone_reference.csv`: current matched truck-only evaluations and historical comparison.
- `truck_only_constructor_audit.csv`: current constructor reproduction of historical truck-only values.
- `diagnostic_checkpoint.json`: resumable per-method checkpoint.
