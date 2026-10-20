# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 27 | 27 | 100 | 100 | 24 | 3 | 0 | 56463.7 | 31297.2 | 76066.4 | 39817.7 | 23.3473 | 23.3239 | 43.523 | 46.841 | 124.27 | 123.966 | wilcoxon | 27 | 1.87904e-05 | -0.857143 | rank_biserial |
| 100 | 27 | 27 | 100 | 100 | 24 | 3 | 0 | 56463.7 | 31297.2 | 76066.4 | 39817.7 | 23.3473 | 23.3239 | 43.523 | 46.841 | 124.27 | 123.966 | wilcoxon | 27 | 1.87904e-05 | -0.857143 | rank_biserial |
