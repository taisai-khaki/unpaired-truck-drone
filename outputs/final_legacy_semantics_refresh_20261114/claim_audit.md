# Final legacy-semantics refresh claim audit

## Scope and provenance

- The archived calibration inputs and matching historical table values use 3 trucks and 3 drones; the latest manuscript describes the Table 5 cases as single-truck/single-drone. The refresh uses the frozen files that generated the table and marks that manuscript description for correction.
- The real-data sample is the exact same 80-row sample as the archived lambda run. Lambda values are unchanged: 0.075, 0.100, 0.115, 0.120, 0.125, 0.130, 0.150, and 0.200.
- Real-data comparison includes M1–M7 because Table 16 and its surrounding claims rank/reference M3, M5, M6, and M7 and use M1 as the truck-only denominator. M8 is not in the existing Table 16 lambda comparison and is not added here.
- Every objective shown as a result is gated on both the final feasibility validator and the method's architecture validator. Failed candidates remain in raw JSONL diagnostics and are not presented as feasible objectives.
- The small calibration config uses 50 NILS starts per instance, 80 outer iterations, 18 no-improvement iterations, 45 seconds per start, beta_slack=0.10, and unlimited candidate caps (0/0). The current MILP solve uses the saved 120-second, 0.001-gap, one-thread settings with CBC.

## Exact calibration

- Exact matches: 0/7.
- Certified exact optima: 0/7; valid exact comparisons: 0/7.
- Mean percentage gap: unavailable; no valid exact comparisons.
- Median percentage gap: unavailable; no valid exact comparisons.
- Maximum percentage gap: unavailable; no valid exact comparisons.
- Every returned NILS solution passes both final validators: True.
- Percentage gap is exactly 100 * (Z_NILS - Z_exact) / Z_exact. If Z_exact is zero, the result is 0% only when both objectives are zero; otherwise the percentage is undefined and explicitly flagged.
- Historical claim classification for the exact-gap headline: REMOVE.
- Historical consistency issue: the displayed Table 5 gaps (one 1.05% gap and six 0.00% gaps) average 0.15%, not the stated 0.17%. The archived 50-start file also reports a 10.599056 NILS objective at n=9, below the archived exact objective 10.697707; the paper instead prints 10.697707 for NILS at n=9.
- The exact solver objective is compared against the evaluator-recomputed objective. A mismatch, a non-proven solver status, or a failed final validator leaves the exact optimum/gap blank rather than silently using a fallback.
- CBC proof limitation: the configured relative gap is 0.001. The current PuLP adapter reports zero gap and sets the best bound equal to the incumbent when PuLP returns `Optimal`; those adapter fields do not independently certify a zero-gap optimum. The refresh therefore does not treat numeric status 2 as a proof.

## Exact-case diagnostics

- n=3: no certified proof (solver status 2).
- n=4: no certified proof (solver status 2).
- n=5: no certified proof (solver status 2).
- n=6: no certified proof (solver status 2).
- n=7: no certified proof (solver status 2); UNNECESSARY_BATTERY_SWAP: Drone 3 has an unnecessary battery swap at rendezvous node 5; Drone 3 has an unnecessary battery swap at rendezvous node 5.
- n=8: no certified proof (solver status 2); MILP objective/evaluator mismatch (59.383469350916734 vs 1428.2715178718215).
- n=9: no certified proof (solver status 9); UNSERVED: Customer 4 is not served; UNSERVED: Customer 6 is not served; UNSERVED: Customer 7 is not served; UNSERVED: Customer 8 is not served; UNSERVED: Customer 9 is not served; SYNC_EXTRA: u_sync at node 1 for drone 2 and truck 3 lacks a joint route visit; SYNC_EXTRA: u_sync at node 5 for drone 2 and truck 3 lacks a joint route visit; DRONE_SERVICE_ROUTE: Drone 1 serves customer 2 but the customer is absent from its route; DRONE_SERVICE_ROUTE: Drone 2 serves customer 3 but the customer is absent from its route; UNMATCHED_RELOAD: Drone 2 has a parcel reload without a rendezvous at node 1; UNMATCHED_RELOAD: Drone 2 has a parcel reload without a rendezvous at node 5; UNNECESSARY_RELOAD: Drone 2 has an unnecessary parcel reload at rendezvous node 1; UNNECESSARY_RELOAD: Drone 2 has an unnecessary parcel reload at rendezvous node 5; UNMATCHED_RELOAD: Drone 2 has a parcel reload without a physical rendezvous at node 1; UNMATCHED_RELOAD: Drone 2 has a parcel reload without a physical rendezvous at node 5; SYNC_ROUTE: Rendezvous at node 1 is not visited by both truck 3 and drone 2; SYNC_ROUTE: Rendezvous at node 5 is not visited by both truck 3 and drone 2; SYNC_ROUTE: Parcel reload at node 1 is not visited by both truck 3 and drone 2; SYNC_ROUTE: Parcel reload at node 5 is not visited by both truck 3 and drone 2; u_sync at node 1 for drone 2 and truck 3 lacks a joint route visit; u_sync at node 5 for drone 2 and truck 3 lacks a joint route visit; Drone 1 serves customer 2 but the customer is absent from its route; Drone 2 serves customer 3 but the customer is absent from its route; Drone 2 has a parcel reload without a rendezvous at node 1; Drone 2 has a parcel reload without a rendezvous at node 5; Drone 2 has an unnecessary parcel reload at rendezvous node 1; Drone 2 has an unnecessary parcel reload at rendezvous node 5; M5 drone 2 rendezvous node 1 is absent from its route; M5 truck 3 rendezvous node 1 is absent from its route; M5 drone 2 rendezvous node 5 is absent from its route; M5 truck 3 rendezvous node 5 is absent from its route; Drone 2 z1 event at node 1 is not linked to a physical rendezvous; Drone 2 z1 event at node 5 is not linked to a physical rendezvous; MILP objective/evaluator mismatch (4.2137608565454645 vs 0.0).

## Real-data lambda results

| lambda | best validated method (M1–M7) | M5 rank | M5 improvement vs M1 (%) | Z_M5 - Z_M3 | valid methods |
|---:|---|---:|---:|---:|---:|
| 0.075 | M3 Paired baseline | 4 | 9.794219752669045 | 35119.947474093875 | 7 |
| 0.100 | M3 Paired baseline | 3 | 6.923735318791183 | 110381.82137448445 | 7 |
| 0.115 | M4 No-unpairing | 4 | 12.137576110833702 | 9818.371172758401 | 7 |
| 0.120 | M4 No-unpairing | 3 | 23.909298977296313 | 37202.44078242703 | 7 |
| 0.125 | M4 No-unpairing | 3 | 12.479712692280623 | -0.11826823314186186 | 7 |
| 0.130 | M6 NILS-no-local-search | 3 | 10.151362017868015 | -21960.908259312622 | 7 |
| 0.150 | none | -- | -- | -- | 0 |
| 0.200 | none | -- | -- | -- | 0 |

- Feasibility coverage: 42/56 method–lambda rows pass both validators. At lambda=0.150 and 0.200, all seven methods fail feasibility because all three truck routes exceed the shift; no objective or rank is reported for those rows. The historical table also marks these two settings infeasible.
- Historical real-data ranking classification: INTERPRETATION UPDATE.
- Historical infeasible-domain classification at lambda=0.150 and 0.200: UNCHANGED.
- At lambda=0.115, 0.120, and 0.125, the best method is M4 in all three cases; do not claim a best-method switch across this local range. The M5–M3 objective difference still crosses from positive to negative by lambda=0.125, which is descriptive rather than evidence of a specific feasibility mechanism.

## Claim classifications

- Table 5 exact-calibration headline: REMOVE.
- Table 5 fleet description: INTERPRETATION UPDATE; saved inputs associated with the table contain 3 trucks and 3 drones, not one of each.
- Table 16 best-method/rank statements: INTERPRETATION UPDATE.
- Table 16 lambda=0.150/0.200 domain statement: UNCHANGED.
- Section 5.6 method universe: NUMERIC UPDATE ONLY; state explicitly that the real-data rank is over M1–M7. M8 ALNS was not reported in the existing lambda table and is not covered by this refresh.
- Nonmonotone local pattern around lambda=0.115–0.125: INTERPRETATION UPDATE; distinguish the M5–M3 crossover from the best-method ranking and present any discrete-feasibility mechanism as an interpretation, not a demonstrated cause.

## MANUSCRIPT ACTIONS

- Location: Table 5, n=3; action: HOLD; values: exact=None, NILS=27.078289251979008, gap=None%, solver runtime=14.513440132141113 s, NILS 50-start wall time=57.84682790021179 s.
- Location: Table 5, n=4; action: HOLD; values: exact=None, NILS=44.14062550836155, gap=None%, solver runtime=119.42831110954285 s, NILS 50-start wall time=69.31038890004857 s.
- Location: Table 5, n=5; action: HOLD; values: exact=None, NILS=39.859940727561394, gap=None%, solver runtime=119.35321497917175 s, NILS 50-start wall time=97.02893020003103 s.
- Location: Table 5, n=6; action: HOLD; values: exact=None, NILS=45.43199730017539, gap=None%, solver runtime=118.07696199417114 s, NILS 50-start wall time=122.6825698998291 s.
- Location: Table 5, n=7; action: HOLD; values: exact=None, NILS=48.69321549429423, gap=None%, solver runtime=116.68495178222656 s, NILS 50-start wall time=172.74242479988607 s.
- Location: Table 5, n=8; action: HOLD; values: exact=None, NILS=49.908097746788805, gap=None%, solver runtime=113.65709924697876 s, NILS 50-start wall time=247.50886769982753 s.
- Location: Table 5, n=9; action: HOLD; values: exact=None, NILS=45.139251068426375, gap=None%, solver runtime=111.426096200943 s, NILS 50-start wall time=357.09056370001053 s.
- Table 5 exact optimum/gap results: HOLD. This refresh produced no certified exact comparisons; do not replace the manuscript's exact values with solver incumbents until the CBC proof, n=7/n=8 validation/objective issues, and n=9 time-limit case are resolved.
- Location: Table 5 title and Section 5.3 calibration description; action: UPDATE; interpretation: state the archived 3-truck/3-drone fleet used by these frozen instances, not single-truck/single-drone.
- Location: Table 16, lambda=0.075; action: UPDATE numeric row; values: best=M3 Paired baseline, best methods=['paired_baseline'], M5 rank=4, M5 vs M1=9.794219752669045%, Z_M5-Z_M3=35119.947474093875.
- Location: Table 16, lambda=0.100; action: UPDATE numeric row; values: best=M3 Paired baseline, best methods=['paired_baseline'], M5 rank=3, M5 vs M1=6.923735318791183%, Z_M5-Z_M3=110381.82137448445.
- Location: Table 16, lambda=0.115; action: UPDATE numeric row; values: best=M4 No-unpairing, best methods=['no_unpairing'], M5 rank=4, M5 vs M1=12.137576110833702%, Z_M5-Z_M3=9818.371172758401.
- Location: Table 16, lambda=0.120; action: UPDATE numeric row; values: best=M4 No-unpairing, best methods=['no_unpairing'], M5 rank=3, M5 vs M1=23.909298977296313%, Z_M5-Z_M3=37202.44078242703.
- Location: Table 16, lambda=0.125; action: UPDATE numeric row; values: best=M4 No-unpairing, best methods=['no_unpairing'], M5 rank=3, M5 vs M1=12.479712692280623%, Z_M5-Z_M3=-0.11826823314186186.
- Location: Table 16, lambda=0.130; action: UPDATE numeric row; values: best=M6 NILS-no-local-search, best methods=['nils_no_local_search'], M5 rank=3, M5 vs M1=10.151362017868015%, Z_M5-Z_M3=-21960.908259312622.
- Location: Table 16, lambda=0.150; action: UPDATE infeasible-domain status; omit objective and rank; values: best=None, best methods=[], M5 rank=None, M5 vs M1=None%, Z_M5-Z_M3=None.
- Location: Table 16, lambda=0.200; action: UPDATE infeasible-domain status; omit objective and rank; values: best=None, best methods=[], M5 rank=None, M5 vs M1=None%, Z_M5-Z_M3=None.
- Location: Section 5.6 real-data ranking statement; action: UPDATE; interpretation: define the ranking set as M1–M7 and do not extend it to M8 without a matched real-data ALNS run.
