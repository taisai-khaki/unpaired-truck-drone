# Final Audit Verdict

## Q1. Why did 561 M5 objectives change?

**Proven:** the files come from different run lineages: June `run-heuristic-study` versus the August priority-service driver. The June source SHA is unavailable, the August SHA is not recorded, the drivers use different instance/execution pathways, and the August wrapper adds feasibility postprocessing. The raw seeds and 720 instance IDs align. **Likely:** an unrecorded historical code or execution-path difference, including generated in-memory versus JSON-reloaded state and wrapper behavior. **Unresolved:** the single historical source line that caused every numeric difference, because the original June code and in-memory objects are unavailable.

Confirmed reconciliation: `561` changed, `462` August-better, `99` August-worse, and `159` unchanged.

## Q2. Did August improve standard NILS itself?

Not proven. The audit cannot attribute the change solely to an NILS algorithm improvement. Current direct NILS matched the August objective for all sampled n=15 and n=50 cases, but not for the sampled n=75 and n=100 cases. The tracked August additions are primarily NILS-R and reporting infrastructure; standard M5 keeps risk rollback disabled.

## Q3. Are August M5 and M9 reproducible and consistent with final definitions?

The August M5 and M9 outputs use separate method names, but full August M5 reproducibility is not established. In the nine-case replay, current code reproduced all sampled n=15/n=50 August M5 values but not the sampled n=75/n=100 values. Full historical SHA-level reproducibility is not established because the August run did not save its SHA. M9 remains algorithmically distinct from M5, but its August full-run provenance also lacks a recorded SHA.

## Q4. Can existing corrected ALNS output be used for both comparisons?

Yes. The corrected ALNS run has 720/720 feasible rows, 40/40 iterations, 40 candidate evaluations per row, and no ten-second cutoff. Reuse it for both M5--ALNS and M9--ALNS, provided the compared M5/M9 files are explicitly identified and the wall-clock budget is reported.

## Q5. Which results can be retained unchanged?

The corrected ALNS execution validation, August M9 decision-quality outputs, fleet sweep, and any architecture/model results independent of M5 numerical values can be retained. The June M5 tables can also be retained as a coherent submitted-run block, but their provenance must be labeled.

## Q6. Which results require correction or recomputation?

Do not mix the August M5 file with June-derived ALNS or ablation values. Table 15 must use a single matched M5/ALNS provenance. Any June table whose numeric claim directly uses standard M5 must either remain on the June block or be regenerated with the final canonical protocol.

## Q7. Is a new full experiment necessary?

Not for diagnosing the 561 discrepancy or validating the corrected ALNS loop: the nine-instance replay and existing ALNS protocol are sufficient. However, a new full M5/M9 experiment is scientifically necessary if the paper is to claim that all final results come from one pinned, reproducible methodology, because current code did not reproduce the sampled n=75/n=100 M5 values. It is not necessary to rerun ALNS solely to validate its 40-iteration search.

## Q8. Minimum manuscript revision

Add a reproducibility paragraph specifying the Git SHA, raw-instance directory hash, configuration, seeds, NILS iteration/time limits, ALNS 40-iteration protocol, actual runtimes, and the fact that M5 and M9 are separate. Replace or relabel Table 15 so it does not combine incompatible M5 generations. State that the June--August discrepancy is an execution/provenance issue whose historical single-line cause cannot be reconstructed.

## Final recommendation

Use a single canonical result block for the paper. The conservative immediately defensible comparison is the submitted M5 block with the corrected ALNS benchmark, while retaining August M9 only with explicit unpinned-run provenance. If the authors want the revised August methodology to be canonical, rerun M5 and M9 under a pinned commit and one verified raw-instance dataset; do not silently treat the existing August M5 values as fully reproducible.
