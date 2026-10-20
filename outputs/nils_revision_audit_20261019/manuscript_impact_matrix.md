# Manuscript Impact Matrix

The submitted PDF named `Drone_paper (7).pdf` was not present in the repository, so this matrix is based on the requested section/table mapping and the available generated artifacts. The manuscript should be checked against the PDF before final submission.

| Manuscript item | Impact | Recommendation |
|---|---|---|
| Section 4 and Algorithm 1 | Method-definition claim | Retain if it describes standard NILS separately from NILS-R; add the pinned protocol and clarify that M5 is not NILS-R. |
| Tables 5--14 | Depends on June standard-NILS outputs | Do not replace automatically. Retain only after confirming each table uses the June M5 file and does not claim August-driver provenance. |
| Table 15 M5--ALNS | Directly affected by benchmark provenance | Use the corrected ALNS output and the matching submitted M5 source, or rerun both methods under one pinned protocol. Do not mix August M5 with June ALNS. |
| Tables 16--18 NILS-R/decision quality | Uses August M9 artifacts | Retain, with the August run provenance and the distinction between M5 and M9 stated explicitly; do not claim full SHA-level reproducibility. |
| Table 20 fleet marginal value | Uses August M9 fleet sweep | Retain if the fleet sweep is identified as a separate August experiment and its seed/configuration are documented. |
| Appendix B propositions | Mathematical/algorithmic claim | Verify that propositions refer to the stored best feasible incumbent. Do not state monotonicity for the temporary current solution during perturbation. |

## Objective and feasibility consistency

The available reports use the same objective-component naming and feasibility validator for the final M5/M9/ALNS comparison. The August wrapper's ignored return value from `evaluate_solution` should be corrected in a future reproducibility patch before a fresh canonical run; it is not evidence that the existing corrected ALNS benchmark is invalid.

The nine-case replay reproduced the August M5 objective for all sampled n=15 and n=50 instances but not for the sampled n=75 and n=100 instances. Therefore August M5-dependent numerical claims should be retained only as an explicitly historical run block or recomputed under a pinned final protocol.
