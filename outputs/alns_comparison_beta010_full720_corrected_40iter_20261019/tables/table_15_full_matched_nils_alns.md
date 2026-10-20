# Full 720-Instance NILS--ALNS Comparison

Lower objective values are better. Positive improvement means NILS improves over ALNS.
Wilcoxon tests are paired on instances where both methods are feasible/reportable.
Negative rank-biserial effect sizes favor NILS because the test is applied to NILS minus ALNS objectives.

| size | matched_instances | paired_feasible_instances | nils_feasible_pct | alns_feasible_pct | nils_wins | alns_wins | ties | mean_nils_objective | median_nils_objective | mean_alns_objective | median_alns_objective | mean_nils_improvement_vs_alns_pct | median_nils_improvement_vs_alns_pct | mean_nils_cpu_s | median_nils_cpu_s | mean_alns_cpu_s | median_alns_cpu_s | wilcoxon_test | wilcoxon_statistic | wilcoxon_p_value | wilcoxon_effect_size | wilcoxon_effect_name |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| All | 720 | 720 | 100 | 100 | 401 | 314 | 5 | 48038.5 | 2836.17 | 45022.8 | 2791 | -14.5336 | 1.88414 | 11.777 | 4.17796 | 47.9706 | 31.6224 | wilcoxon | 111273 | 0.00248752 | -0.130578 | rank_biserial |
| 15 | 230 | 230 | 100 | 100 | 138 | 87 | 5 | 212.973 | 116.223 | 870.848 | 125.528 | 17.3038 | 1.32363 | 0.296308 | 0.267208 | 3.6728 | 3.65891 | wilcoxon | 7066 | 7.63523e-09 | -0.444169 | rank_biserial |
| 50 | 210 | 210 | 100 | 100 | 156 | 54 | 0 | 14036.3 | 3992.99 | 21024.7 | 6245.22 | -24.4499 | 27.1029 | 3.79167 | 3.68687 | 30.576 | 29.8308 | wilcoxon | 3985 | 8.64153e-16 | -0.640262 | rank_biserial |
| 75 | 90 | 90 | 100 | 100 | 37 | 53 | 0 | 39808.1 | 11982.5 | 33916.3 | 11436.8 | -24.4971 | -7.76924 | 10.7087 | 8.87294 | 68.2728 | 57.8107 | wilcoxon | 1587 | 0.0638954 | 0.224908 | rank_biserial |
| 100 | 190 | 190 | 100 | 100 | 70 | 120 | 0 | 147412 | 109235 | 130255 | 99029 | -37.3938 | -27.9464 | 35.0065 | 31.0545 | 111.203 | 110.591 | wilcoxon | 6773 | 0.0024489 | 0.253458 | rank_biserial |
