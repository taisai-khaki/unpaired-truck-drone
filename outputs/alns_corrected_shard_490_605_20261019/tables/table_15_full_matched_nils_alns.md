# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 4 | 4 | 100 | 100 | 3 | 1 | 0 | 45978.2 | 40676.4 | 38194.4 | 39747.7 | -11.6225 | 11.666 | 18.7854 | 21.5703 | 58.016 | 57.5378 | wilcoxon | 4 | 0.875 | -0.2 | rank_biserial |
| 75 | 4 | 4 | 100 | 100 | 3 | 1 | 0 | 45978.2 | 40676.4 | 38194.4 | 39747.7 | -11.6225 | 11.666 | 18.7854 | 21.5703 | 58.016 | 57.5378 | wilcoxon | 4 | 0.875 | -0.2 | rank_biserial |
