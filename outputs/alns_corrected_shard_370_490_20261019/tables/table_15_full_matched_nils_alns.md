# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 120 | 120 | 100 | 100 | 61 | 59 | 0 | 28504.3 | 4656.88 | 25725.4 | 8451.82 | -80.6775 | 0.768428 | 5.98292 | 5.07354 | 39.3466 | 34.9773 | wilcoxon | 3572 | 0.87927 | -0.015978 | rank_biserial |
| 50 | 70 | 70 | 100 | 100 | 44 | 26 | 0 | 5484.58 | 1348.78 | 8115.97 | 2307.21 | -120.669 | 18.63 | 3.9047 | 4.593 | 37.193 | 39.4665 | wilcoxon | 702 | 0.00156096 | -0.43501 | rank_biserial |
| 75 | 50 | 50 | 100 | 100 | 17 | 33 | 0 | 60731.8 | 25650.5 | 50378.6 | 20637.8 | -24.6888 | -14.7324 | 8.89243 | 8.87294 | 42.3618 | 33.223 | wilcoxon | 372 | 0.0096745 | 0.416471 | rank_biserial |
