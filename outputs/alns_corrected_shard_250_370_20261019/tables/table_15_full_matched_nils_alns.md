# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 120 | 120 | 100 | 100 | 95 | 25 | 0 | 21313.3 | 11678.8 | 31989.3 | 19901.2 | 23.6662 | 28.2295 | 3.61655 | 3.43895 | 29.87 | 28.4557 | wilcoxon | 980 | 3.92091e-12 | -0.730028 | rank_biserial |
| 50 | 120 | 120 | 100 | 100 | 95 | 25 | 0 | 21313.3 | 11678.8 | 31989.3 | 19901.2 | 23.6662 | 28.2295 | 3.61655 | 3.43895 | 29.87 | 28.4557 | wilcoxon | 980 | 3.92091e-12 | -0.730028 | rank_biserial |
