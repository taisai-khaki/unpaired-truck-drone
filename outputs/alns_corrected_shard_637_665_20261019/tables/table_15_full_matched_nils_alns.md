# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 28 | 28 | 100 | 100 | 18 | 10 | 0 | 169425 | 176810 | 202128 | 195951 | 9.91036 | 7.74309 | 35.8704 | 34.2008 | 122.992 | 125.244 | wilcoxon | 145 | 0.193562 | -0.285714 | rank_biserial |
| 100 | 28 | 28 | 100 | 100 | 18 | 10 | 0 | 169425 | 176810 | 202128 | 195951 | 9.91036 | 7.74309 | 35.8704 | 34.2008 | 122.992 | 125.244 | wilcoxon | 145 | 0.193562 | -0.285714 | rank_biserial |
