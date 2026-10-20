# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 28 | 28 | 100 | 100 | 7 | 21 | 0 | 97192.6 | 51977.6 | 60246.2 | 33282 | -72.6296 | -40.8615 | 32.4874 | 24.8466 | 104.324 | 100.203 | wilcoxon | 61 | 0.000716209 | 0.699507 | rank_biserial |
| 75 | 8 | 8 | 100 | 100 | 4 | 4 | 0 | 4669.78 | 3656.66 | 5720.8 | 2554.21 | -75.2958 | 12.0323 | 16.5597 | 16.2578 | 116.638 | 115.364 | wilcoxon | 14 | 0.640625 | -0.222222 | rank_biserial |
| 100 | 20 | 20 | 100 | 100 | 3 | 17 | 0 | 134202 | 95593.7 | 82056.4 | 54053.1 | -71.5631 | -70.2341 | 38.8585 | 26.2384 | 99.3992 | 97.3588 | wilcoxon | 17 | 0.000394821 | 0.838095 | rank_biserial |
