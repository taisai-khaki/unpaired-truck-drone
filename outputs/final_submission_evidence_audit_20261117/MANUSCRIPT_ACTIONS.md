# Final manuscript action ledger

**Controlling version:** `C:\Users\L03128674\Downloads\Drone_paper (9).pdf` (35 pages; SHA-256 `EF3AE96764DB89A44CA71EE4BF3579B846A04B366F265D9C2DE41FE13F1A3247`). Earlier PDFs were not used as the controlling manuscript.

**Decision:** **BLOCKED — not ready for submission.** The exact-MILP audit certifies only 1/7 cases; the M2/M6/M7 gate failed and the 2,160 runs were not started; 227 frozen principal objectives change under corrected readiness-based evaluation; and Table 2's 20-km range does not determine the battery capacity used by code.

## Action matrix

| Manuscript location | Existing claim/content | Action | Evidence / type |
|---|---|---|---|
| Abstract | NILS on-time high-priority rate is 35.92% versus ALNS 30.89%. | REPLACE 35.92% with 35.97%; retain the qualitative scale-dependent objective ranking. | Corrected customer-level record: 2,869/7,977 vs 2,464/7,977; scientific numeric correction. |
| Table 2, drone speed | 65 km/h is shown as the representative cruise speed. | REPLACE with explicit distinction: nominal/default 65 km/h; the principal D-optimal design overrides it to 43.75/70 km/h from 35 km/h truck speed × ratios 1.25/2.00. | Frozen run configuration and `src/experiments/heuristic_study.py`; scientific reproducibility correction. |
| Table 2, drone range / battery | 20 km per charge is described as calibrating battery-feasibility regimes. | REPLACE with implemented base capacity 1,200 Wh and low/medium/high capacities 840/1,200/1,560 Wh; state that the current code does not derive 1,200 Wh from 20 km. Keep the 20-km assumption only if authors provide a consistent calibration and rerun the affected study. | `config/default.yaml`, endurance multiplier, energy tests; material model-parameter mismatch. |
| Table 2 and Appendix A.3, aerodynamic source | Table 2 cites Moshref-Javadi et al. (2021); Appendix A.3 attributes the displayed formula to Liu et al. (2017). | REPLACE/RECONCILE the inconsistent attribution. The implementation and Appendix A.3 formula correspond internally to the Liu et al. citation; confirm the intended external source before final typesetting. | Internal citation inconsistency; scientific source attribution. |
| Table 3 note | Handling-time levels are described as per-event launch/retrieval burden. | ADD a note that each selected level sets `tau_hand`, `t_rel`, and `t_swap` in this implementation; rendezvous handling is charged independently from state-dependent reload and swap. | Scenario configuration; methodological transparency. |
| Table 5 and following paragraph | Table 5 is NILS 50-start feasibility calibration; text says no exact optimality gaps are certified. | KEEP Table 5 and its concept. ADD the separate exact-MILP audit table from `exact_table5.tex`; do not replace the NILS calibration with MILP results. Replace the paragraph as shown below. | Table 5 values are still verified; only n=3 is certified exact. |
| Tables 6–14 / associated analysis | These contain historical aggregates from the frozen M1/M3/M4/M5/M8 experiment. | REPLACE numeric bodies and dependent exact counts with `corrected_tables_principal.tex`; preserve manuscript's original labels or update all references consistently. These are corrected evaluator derivatives, not new optimization runs. | Current evaluator re-evaluated 2,880 saved schedules; 227 objective values change, none becomes infeasible. Scientific numeric correction. |
| Methods/experiment coverage, M2/M6/M7 | The paper says the main comparison is M3/M4/M5/M8 with M1 as reference; M2/M6/M7 are not pooled into that principal comparison. | KEEP the distinction. Do not imply that a new full 720-instance M2/M6/M7 study exists. No such 2,160-run study was performed because the diagnostic gate failed. | 12-run diagnostic: M2 is infeasible on one n=100 case; full runs not started. |
| Table 13 text | M5/M3 wins are 452/267/1 and overall p-values are 0.00117 and 2.77e-5. | REPLACE with M5/M3 453/266/1, M5/M4 292/379/49, median improvements and p-values from the corrected table. Preserve the statement that more flexibility does not guarantee better finite-time heuristic outcomes. | Paired re-evaluation of frozen M1/M3/M4/M5/M8 records. |
| Table 14 and associated priority text | Means, wins, customer-level counts, and on-time percentages reflect earlier evaluator output. | REPLACE with corrected Table 14 in `corrected_tables_principal.tex`; use the added runtime/feasibility table. Do not claim NILS is uniformly superior: ALNS has the lower pooled objective, and NILS has a modest overall on-time advantage while ALNS delivers more high-priority customers earlier. | Corrected records: M5 wins 325, M8 395; 7,977 matched high-priority deliveries. Scientific numeric correction. |
| Table 15 status at lambda 0.150/0.200 | Status is labeled “Infeasible”; prose says constructed instances violate route duration. | REPLACE “Infeasible” with “No feasible schedule found” (or “Shift-violating schedules”) and add that these heuristic outcomes are not mathematical infeasibility certificates. Keep the numeric feasible rows; all 56 real-data objectives re-evaluate unchanged. | 14 saved schedules fail the 480-minute shift check; mathematical infeasibility was not proved. |
| Conclusion | “NILS is more competitive at smaller problem sizes, while ALNS obtains lower mean objectives and high-priority tardiness at the two largest sizes.” | KEEP. The corrected re-evaluation supports this size-dependent statement; do not add a universal priority-service or ablation claim. | Corrected M5/M8 results by size; no full M2/M6/M7 comparison exists. |

## Optional clustered service interval

The paired instance-cluster bootstrap estimates the M5-minus-M8 high-priority on-time-rate difference as **5.08 percentage points**, with a 95% percentile interval of **[3.85, 6.35] percentage points** (10,000 resamples of 720 instance clusters; seed 20261117). It uses the 7,977 matched customer pairs and does not resample customers independently. This supplemental estimate is not needed for the descriptive manuscript replacement text above and should not be interpreted as population-level validation of the D-optimal design.

## Verbatim controlling-manuscript excerpts

These excerpts are transcribed from the controlling 35-page PDF; they identify the existing claims that the actions above address.

- **Abstract:** “Customer-level analysis of 7,977 high-priority deliveries provides a complementary service perspective: although ALNS delivers more high-priority customers earlier overall, NILS places 35.92% of high-priority deliveries on time compared with 30.89% for ALNS.”
- **Table 2:** “Drone speed $v_D$ | 65 km/h | Representative operational cruise speed”; “Drone range | 20 km per charge | Base endurance setting used to calibrate battery-feasibility regimes.”
- **Table 3:** “Handling time $\tau_{hand}$ | {short, medium, long} | Per-event launch/retrieval handling burden.”
- **Table 5 discussion:** “These experiments establish small-instance solution feasibility but do not provide certified optimality gaps; an exact comparison requires reconciliation of the MILP formulation, solver certificates, and solution evaluator.”
- **Table 13 narrative:** “Across all 720 instances, ALNS wins 396 objective comparisons and NILS wins 324.” The table reports M5-versus-M3 as 452 wins, 267 losses, and one tie; size-specific M5-versus-M4 counts include 112/97/1 at $n=50$.
- **Table 14 narrative:** “Across 7,977 matched high-priority customer deliveries, NILS delivers 3,468 customers earlier, ALNS delivers 3,923 earlier, and 586 are tied. NILS nevertheless places 2,865 high-priority deliveries on time (35.92%), compared with 2,464 (30.89%) for ALNS.”
- **Table 15:** The rows at $\lambda=0.150$ and $0.200$ are labeled “Infeasible”; the note states, “Infeasibility at the two largest scaling values is associated with the route-duration constraint.”
- **Conclusion:** “NILS is more competitive at smaller problem sizes, while ALNS obtains lower mean objectives and high-priority tardiness at the two largest sizes.”

## Copy-ready replacement text

### Abstract sentence

Replace the on-time clause with:

```latex
Customer-level analysis of 7,977 high-priority deliveries provides a complementary service perspective: although ALNS delivers more high-priority customers earlier overall, NILS places 35.97\% of high-priority deliveries on time, compared with 30.89\% for ALNS.
```

### Table 2 rows

Replace the speed, range/capacity, and aerodynamic-source rows with these truthful rows (adapt column widths to the source table):

```latex
Drone speed $v_D$ & 65 km/h nominal; 43.75 or 70 km/h in the principal design & The 720-instance design uses $v_D/v_K\in\{1.25,2.00\}$ with $v_K=35$ km/h \\
Drone battery capacity $B_d$ & 1,200 Wh base; 840/1,200/1,560 Wh for low/medium/high endurance & Endurance multipliers are 0.70/1.00/1.30; the implementation does not derive these capacities from a 20-km range \\
Aerodynamic power model & Liu et al. (2017), subject to source confirmation & $c_1=2.8037$, $c_2=0.3177$, $c_4=0.0296$, $c_5=0.0279$, $g=9.8\,\mathrm{m/s^2}$, $\alpha=10^\circ$; speed is converted to m/s \\
```

Add to the Table 3 note:

```latex
The selected handling-time level sets the physical rendezvous handling time and the reload and swap durations in the computational implementation. Physical rendezvous handling is charged independently of whether a parcel reload or battery swap is required.
```

### Table 5 discussion and exact-MILP addition

Keep the NILS calibration table unchanged. Replace the sentence beginning “These experiments establish small-instance solution feasibility...” with:

```latex
The 50-start results in Table~5 establish small-instance NILS feasibility but are not an exact comparison. We separately solved the archived $n=3,\ldots,9$ instances with CBC and retained the native solver logs. Only the $n=3$ case is certified optimal. Time-limited cases $n=4$--$6$ and $n=8$ have validated incumbents but large nonzero gaps; the $n=7$ incumbent fails the final battery-swap validator, and the $n=9$ run found no feasible CBC incumbent within the time limit. Accordingly, no optimality gaps against NILS are reported except for the certified $n=3$ case. Table~\ref{tab:exact_milp_audit} reports the complete audit.
```

The exact table is an **additional** audit table, not a replacement for the NILS feasibility-calibration Table 5. The n=8 historical mismatch was a timing-evaluator error: the old evaluator added 1,368.8880 weighted tardiness units from 13.6889 minutes of stale high-priority waiting; the corrected readiness-based evaluator reproduces the archived MILP objective 59.3834694.

### Tables 13–14 narrative

Replace the Table 13 count sentence with:

```latex
Across 720 instances, M5 produces 292 wins, 379 losses, and 49 ties against M4, and 453 wins, 266 losses, and one tie against M3. M5 uses 420 cross-truck sorties among 1,812 drone sorties. Thus, the additional recovery flexibility is exercised, but the expanded coordination set does not guarantee better finite-time heuristic solutions.
```

Replace the Table 14 aggregate/customer summary with:

```latex
Across all 720 instances, M5 has lower objective values in 325 comparisons and ALNS in 395. M5's mean objective is 80,081.14, compared with 50,263.42 for ALNS; the scale-dependent pattern remains, with M5 winning 132/230 instances at $n=15$ and 157/210 at $n=50$, while ALNS wins 58/90 at $n=75$ and 186/190 at $n=100$. Across 7,977 matched high-priority deliveries, M5 delivers 3,475 customers earlier, ALNS delivers 3,916 earlier, and 586 are tied. M5 places 2,869 deliveries on time (35.97\%), compared with 2,464 (30.89\%) for ALNS; there are 1,350 M5-only and 945 ALNS-only on-time deliveries. These measures distinguish objective value, earlier delivery, total high-priority tardiness, and on-time service rather than implying uniform dominance by either method.
```

Use the corrected CPU/feasibility table supplied with the table fragments. The 120-second budget is nominal; M5/M8 are feasible and architecture-valid on 720/720 runs each.

### Table 15 status sentence

Replace the sentence that begins “At $\\lambda=0.150$ and 0.200...” with:

```latex
At $\lambda=0.150$ and $0.200$, none of the seven saved method schedules satisfies the 480-minute route-duration constraint under the fixed fleet and operating assumptions. These are unsuccessful heuristic schedules, not certificates that the underlying routing instances are mathematically infeasible.
```

## Consistency matrix

| Major claim | Verified source | Status |
|---|---|---|
| 720 frozen instances; 72 classes × 10 replications; M3/M4/M5/M8 totals 2,880 | `frozen_720_revalidation_summary.json`, immutable `input_manifest.csv` | Inputs and artifacts unchanged; 2,880 re-evaluated |
| M1 truck-only reference valid and reproduced | `frozen_720_m1_reference_revalidation.csv`, `reference_revalidation_summary.json` | 720/720 |
| Principal M3/M4/M5/M8 feasibility | `frozen_720_revalidation.csv` | 720/720 per method; no formerly accepted solution invalidated |
| Principal objective summaries and matched tests | `frozen_720_corrected_overall_summary.csv`, `frozen_720_corrected_summary_by_method_size.csv`, `frozen_720_corrected_matched_comparisons.csv` | 227 historical objective values change under corrected evaluator; table numbers require revision |
| M5/M8 high-priority service | `frozen_720_corrected_priority_service_summary.csv`, `frozen_720_corrected_M5_M8_customer_comparison.csv`, `frozen_720_corrected_hpt_comparisons.csv` | 7,977 paired customer records |
| Exact MILP statuses, bounds, and validation | `exact_results.csv`, `exact_certificates.csv`, `exact_solver_logs/`, `exact_validation_summary.md` | 1/7 proven; n=7 invalid; n=9 no feasible incumbent |
| M2/M6/M7 full-design results | `ablation_gate_diagnostics.csv`, `ablation_gate_summary.json` | Diagnostic only; full 2,160-run comparison not performed |
| Table 15 real-data results | `real_data_revalidation.csv`, `real_data_corrected_table15.csv` | 56 rows; no objective changes; no valid schedules at lambda 0.150/0.200 |
| Battery capacity and speed values | `config/default.yaml`, `src/experiments/heuristic_study.py`, `tests/test_energy_calibration.py` | Code uses 1,200-Wh base and 0.70/1.00/1.30 multipliers; nominal 20-km claim does not determine capacity |

## Manuscript readiness blockers

1. Resolve the 20-km / 1,200-Wh battery calibration and speed-row interpretation in Table 2; if the intended numerical capacity changes, the frozen battery-feasibility experiment is not manuscript-ready without a controlled rerun.
2. Resolve optional `z2` swaps and exact readiness-wait equality in the MILP, or revise the model/paper conventions and rerun the seven-case exact audit.
3. Decide whether to repair M2's n=100 shift-violating construction and rerun the diagnostic; do not silently replace it with a feasible fallback.
4. Resolve the 227 corrected-evaluator objective differences with the authors' reporting policy before replacing manuscript aggregates. The frozen result artifacts themselves remain immutable.
5. Complete the NILS-to-MILP fixed-route compatibility check if the manuscript continues to make a complete MILP/heuristic formulation-equivalence claim.
