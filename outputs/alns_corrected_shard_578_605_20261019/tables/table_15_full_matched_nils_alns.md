# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 27 | 27 | 100 | 100 | 3 | 24 | 0 | 78674.7 | 5005.64 | 66635.3 | 2580.54 | -89.5436 | -82.9365 | 25.8763 | 25.3973 | 104.206 | 102.694 | wilcoxon | 69 | 0.00296679 | 0.634921 | rank_biserial |
| 100 | 27 | 27 | 100 | 100 | 3 | 24 | 0 | 78674.7 | 5005.64 | 66635.3 | 2580.54 | -89.5436 | -82.9365 | 25.8763 | 25.3973 | 104.206 | 102.694 | wilcoxon | 69 | 0.00296679 | 0.634921 | rank_biserial |
