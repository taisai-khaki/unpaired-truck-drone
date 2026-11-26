# Final manuscript action ledger

**Controlling manuscript:** Drone_paper (9).pdf, 35 pages, SHA-256 EF3AE96764DB89A44CA71EE4BF3579B846A04B366F265D9C2DE41FE13F1A3247.

**Overall status: BLOCKED.** The requested M2/M6/M7 experiment and all-method tables are complete, but exact-MILP/evaluator semantics remain unreconciled. Only one of seven MILP instances is certified optimal; the n=7 incumbent fails strict validation and n=9 has no feasible incumbent. The paper's 20-km range does not determine the 1,200-Wh base battery used by the code. Separately, 227 archived M3/M4/M5 objective values change when saved routes are re-evaluated with readiness-based waiting. Do not state that the paper is computationally ready or that the MILP and heuristic feasible sets are equivalent.

## Evidence and scope

- Size-balanced relative-objective index, lower is better: M8 35.25, M4 39.41, M5 41.29, M3 44.24, M7 52.38, M6 57.00, M2 97.62, M1 100.00. The M2 value is conditional on 487 feasible runs and must not be treated as a complete 720-instance comparison.
- Pooled mean objectives: M8 50,263.42; M4 76,023.80; M5 80,081.14; M3 81,735.49; M6 95,629.47; M7 97,469.49; M1 144,351.79. M2's mean 67,258.02 is calculated only on its 487 valid runs.
- M2: 487/720 valid; 233 retained infeasible schedules (0/230 at n=15, 9/210 at n=50, 36/90 at n=75, and 188/190 at n=100). No M2 architecture-invalid outcomes, run exceptions, or objective inconsistencies. M6 and M7 each have 720/720 valid schedules and valid ablation flags.
- Original M1/M3/M4/M5/M8 routes and outputs were not rerun or overwritten. Their corrected-evaluator derivatives are separate; the earlier evaluator audit found 227 M3/M4/M5 objective differences.

## Required manuscript edits

### Abstract — customer-level service sentence

**Existing text:** “Customer-level analysis of 7,977 high-priority deliveries provides a complementary service perspective: although ALNS delivers more high-priority customers earlier overall, NILS places 35.92% of high-priority deliveries on time compared with 30.89% for ALNS.”

**Action: REPLACE** the on-time percentage and punctuation with:

~~~latex
Customer-level analysis of 7,977 high-priority deliveries provides a complementary service perspective: although ALNS delivers more high-priority customers earlier overall, NILS places 35.97\% of high-priority deliveries on time, compared with 30.89\% for ALNS.
~~~

**Evidence:** Readiness-based customer comparison; 2,869/7,977 M5 high-priority deliveries on time versus 2,464/7,977 for M8. On-time service remains a post-solution diagnostic, not an objective term.

### Table 2 and Table 3 — operating parameters

**Existing text:** Table 2 lists 65 km/h as representative drone speed and says “Drone range | 20 km per charge | Base endurance setting used to calibrate battery-feasibility regimes.” Table 3 calls handling time a “Per-event launch/retrieval handling burden.”

**Action: REPLACE** the speed/range rows and **ADD** the operational note:

~~~latex
Drone speed $v_D$ & 65 km/h nominal; 43.75 or 70 km/h in the principal design & The design uses $v_D/v_K\in\{1.25,2.00\}$ with $v_K=35$ km/h \\
Drone battery capacity $B_d$ & 1,200 Wh base; 840/1,200/1,560 Wh for low/medium/high endurance & The implementation does not derive these capacities from a 20-km range \\
Aerodynamic power model & Liu et al. (2017), subject to source confirmation & $c_1=2.8037$, $c_2=0.3177$, $c_4=0.0296$, $c_5=0.0279$, $g=9.8\,\mathrm{m/s^2}$, $\alpha=10^\circ$; speed is converted to m/s \\
\textit{Note:} The selected handling-time level sets physical rendezvous handling and the reload and swap durations in the computational implementation. Physical rendezvous handling is charged independently of whether parcel reload or battery swap is required.
~~~

**Also reconcile citations:** Table 2 attributes aerodynamic parameters to Moshref-Javadi et al. (2021), while Appendix A.3 attributes the displayed formula to Liu et al. (2017). Confirm the intended source before typesetting.

### Table 5 and exact-MILP discussion

**Action: KEEP** the 50-start NILS feasibility table. **ADD** table5_exact_validation.tex as a separate exact audit; do not replace the NILS table.

**Replacement paragraph:**

~~~latex
The 50-start results in Table~5 establish small-instance NILS feasibility but are not an exact comparison. We separately audited the archived $n=3,\ldots,9$ instances with CBC and retained the native solver logs. Only the $n=3$ case is certified optimal, with objective 27.0782893. The $n=4$--$6$ and $n=8$ cases have feasible, validated, time-limited incumbents with large nonzero gaps; the $n=7$ incumbent fails the strict battery-swap validator, and the $n=9$ run has no feasible CBC incumbent. Accordingly, no optimality gaps against NILS are reported except for the certified $n=3$ case. Table~\ref{tab:exact_milp_audit} reports the audit.
~~~

The old n=8 evaluator discrepancy (59.3835 versus 1,428.2715) was due to stale synchronization waiting in the old evaluator; readiness-based re-evaluation reconciles the archived objective. The fresh time-limited n=8 incumbent is 59.418475 and is not an optimum. The exact model remains qualified: readiness waiting is not constrained to equal the exact maximum, optional battery swaps conflict with strict validation at n=7, fixed-route NILS-to-MILP representation was not verified, and the implemented global big-M of 10,000 is not reconciled with Appendix A.5's calibrated bounds. Do not claim complete feasible-set equivalence.

### Section 5 — method coverage and experiment design

**Location:** Section 5, paragraph beginning “The eight methods serve distinct experimental purposes.”

**Existing text:** “The principal 720-instance analysis covers M3, M4, M5, and M8, with M1 used for normalization. The refreshed illustrative real-data analysis covers M1–M7. Consequently, conclusions about the incremental contributions of local search and perturbation are not inferred from the M3–M5–M8 comparison alone.”

**Action: REPLACE** the method-coverage paragraph with:

~~~latex
The eight methods serve distinct experimental purposes. M1 is the truck-only reference; M2 is the constructive simple-drone benchmark; M3--M5 compare coordination architectures; M6 and M7 ablate post-improvement local search and perturbation within NILS; and M8 is the independent feasibility-aware ALNS comparator. All eight methods are evaluated on the same 720 frozen instances. The 2,160 new M2, M6, and M7 records use the frozen inputs and operating assumptions; M6 and M7 reuse the persisted M5 truck backbone. M2 returns a valid schedule on 487/720 instances, whereas M6 and M7 are valid on 720/720 each. M2 failures are retained and are not replaced by truck-only solutions. The original M1/M3/M4/M5/M8 schedules remain unchanged; their corrected-evaluator re-evaluations are reported separately.
~~~

Update nearby references from the four-method principal comparison to the complete M1–M8 comparison. Preserve distinct method roles and M8's independent initialization.

### Tables 6–8 — all-method summaries

**Action: REPLACE** Table 6 with table6_all_methods.tex and Table 8 with table8_all_methods.tex. Table 6 reports pooled and size-balanced indices and valid-run denominators. Its M2 mean/index is conditional on only 487 valid schedules. Table 8 reports valid runs, CPU, high-priority tardiness and delivery time, high-priority on-time rate, drone share, truck/drone distance, sorties, rendezvous, and synchronization waiting. Full priority-class tardiness, active drones, sortie types, reloads, swaps, and handling are in all_methods_operational_indicators.csv.

**Action: KEEP WITH CORRECTED VALUES** the M5-specific factor summaries in outputs/final_submission_evidence_audit_20261117/corrected_tables_principal.tex for Tables 7 and 9–12. Do not restore pre-re-evaluation values. Interpret D-optimal factor comparisons descriptively, not causally.

### Tables 13–14 — coordination and ALNS comparisons

**Action: REPLACE** stale counts/values with the corrected-evaluator tables in corrected_tables_principal.tex:

- Table 13: M5 versus M4 is 292 wins, 379 losses, 49 ties; M5 versus M3 is 453 wins, 266 losses, one tie. Keep the explanation that greater coordination flexibility does not guarantee a better finite-time heuristic solution.
- Table 14: M5 has lower objectives in 325 comparisons and M8 in 395; mean objectives are 80,081.14 and 50,263.42. M5 wins 132/230 at n=15 and 157/210 at n=50; M8 wins 58/90 at n=75 and 186/190 at n=100. Across 7,977 matched high-priority deliveries, M5 delivers earlier for 3,475, M8 for 3,916, and 586 tie; on-time rates are 35.97% versus 30.89%.

The frozen schedules were not reoptimized. Identify these as corrected-evaluator derivatives because 227 historical M3/M4/M5 objective values changed on readiness-based re-evaluation.

### New ablation table and real-data table numbering

**Location:** Section 5.5, after the current Table 14 discussion and before the real-data subsection.

**Action: ADD** ablation_comparison.tex as a new Table 15, then renumber the current real-data Table 15 and its citations to Table 16.

**Copy-ready paragraph:**

~~~latex
Across 720 matched instances, M5 has lower objective values than M6 in 563 cases (M6 wins 151; 6 ties; median improvement 17.80%; Wilcoxon $p<0.001$) and than M7 in 523 cases (M7 wins 46; 151 ties; median improvement 12.39%; Wilcoxon $p<0.001$). The M5--M6 difference is not significant at $n=100$ (100/190 versus 90/190 wins; $p=0.496$); the M5--M7 comparison favors M5 in each size class. These are finite-time ablation results, not causal estimates. Mean high-priority tardiness is 807.92 minutes for M5, 982.96 for M6, and 996.64 for M7. On the 710 instances with a defined paired high-priority on-time rate, the rates are 51.69\%, 44.52\%, and 41.91\%, respectively.
The size-specific matched counts, effect sizes, two-sided Wilcoxon tests, and CPU times are reported in Table~\ref{tab:m5_ablation_comparison}. All 720 M6 and all 720 M7 schedules pass feasibility and architecture validation. High-priority on-time service is an evaluation diagnostic, not part of the optimized objective.
~~~

The full-size ablation table reports All and n=15, 50, 75, and 100 rows. At n=15, M5's mean percentage improvement versus M6 is negative despite 193 wins and a positive median improvement; retain both mean and median rather than presenting only the favorable statistic. M5 versus M6 at n=100 is not significant. CPU averages are 33.29 seconds for M5, 29.22 for M6, and 31.63 for M7.

### Real-data Table 16 — status terminology

**Existing text:** Table 15 labels lambda=0.150 and 0.200 “Infeasible” and attributes this to the route-duration constraint.

**Action: REPLACE** both status labels with “No feasible schedule found” and replace the claim with:

~~~latex
At $\lambda=0.150$ and $0.200$, none of the seven saved method schedules satisfies the 480-minute route-duration constraint under the fixed fleet and operating assumptions. These are unsuccessful heuristic schedules, not certificates that the underlying routing instances are mathematically infeasible.
~~~

Keep the real-data ranking explicitly limited to M1–M7; M8 is not included.

### Conclusion

**Action: KEEP** the current size-dependent M5–M8 objective statement, but do not claim overall NILS dominance. **ADD** the qualified ablation summary above. The size-balanced all-method ranking is M8 first, M4 second, and M5 third. Local search's M5–M6 objective difference is not significant at n=100; do not claim that either NILS component improves every size class.

## M5 ablation results by size

| Comparison | n | Valid pairs | M5 wins | Ablation wins | Ties | Median M5 improvement | Wilcoxon p |
|---|---:|---:|---:|---:|---:|---:|---:|
| M5 vs M6 | 15 | 230/230 | 193 | 32 | 5 | 17.80% | <0.001 |
| M5 vs M6 | 50 | 210/210 | 199 | 10 | 1 | 67.63% | <0.001 |
| M5 vs M6 | 75 | 90/90 | 71 | 19 | 0 | 25.24% | <0.001 |
| M5 vs M6 | 100 | 190/190 | 100 | 90 | 0 | 0.83% | 0.496 |
| M5 vs M7 | 15 | 230/230 | 118 | 8 | 104 | 0.82% | <0.001 |
| M5 vs M7 | 50 | 210/210 | 159 | 19 | 32 | 23.50% | <0.001 |
| M5 vs M7 | 75 | 90/90 | 64 | 12 | 14 | 6.99% | <0.001 |
| M5 vs M7 | 100 | 190/190 | 182 | 7 | 1 | 13.69% | <0.001 |

## Model-to-code and evidence files

- exact_model_code_matrix.csv maps formulation families to code and records discrepancies; exact_results.csv and exact_solver_certificates.csv provide the seven exact statuses, primal/dual evidence, gaps, and validation.
- exact_objective_reconciliation.csv and exact_validation_report.md document objective/evaluator and feasibility outcomes; exact_solver_logs contains native CBC logs.
- new_method_raw_results.jsonl has 2,160 unique records. new_method_results_2160.csv and new_method_validation.csv retain outcomes and failure diagnostics.
- all_methods_M1_M8_summary.csv, all_methods_by_size.csv, and all_methods_operational_indicators.csv support the full comparison.
- M5_vs_M6_M7_matched.csv, M5_vs_M6_M7_by_size.csv, and M5_vs_M6_M7_priority_service.csv support matched ablation conclusions.
- run_manifest.json records execution provenance; input_manifest.csv, source_sha256_manifest.csv, and audit_artifact_sha256_manifest.csv record hashes and integrity.

## Remaining blockers

1. Resolve the exact MILP readiness-wait equality and optional battery-swap semantics; n=7 must not be accepted by weakening validation.
2. Complete the fixed-route NILS-to-MILP compatibility audit and reconcile global M=10,000 with Appendix A.5 bounds.
3. Reconcile the Table 2 nominal 20-km range with the implemented 1,200-Wh base capacity and confirm the aerodynamic citation.
4. Apply corrected-evaluator values consistently in the manuscript; do not overwrite immutable source schedules or conceal the 227 objective differences.
5. Report M2's 233 invalid schedules and the conditional denominator for its objective metrics.

**Do not claim PASS or submission readiness until the material model/evaluator blockers above are resolved.**
