# Priority-customer paired delivery analysis

## Scope and validation

- Dataset: FINAL_720_SEMANTICS_CORRECTED_FROZEN; result commit fef001ea75c82ccc04b6731334e77b5e75cc626e.
- Frozen instances: 720; NILS solutions: 720; ALNS solutions: 720.
- No algorithm reruns, solution modifications, instance regeneration, historical method results, or superseded one-record results were used.
- Frozen JSON input hashes all match the final input manifest.
- Customer-method observations checked across M3, M4, M5/NILS, and ALNS: 158,800; tardiness mismatches: 0; maximum absolute reconstruction difference: 0 minutes.
- All observations have exactly one active truck-or-drone service assignment and a realized delivery time. No competing truck/drone assignments.
- 10 instances contain no high-priority customers. Their high-priority earlier-share difference is set to zero for the 720-unit paired test because no customer comparison is defined; they are ties in total high-priority tardiness.
- Delivery-time tie tolerance: 1e-06 minutes. On time means realized delivery time <= customer ub.

## Primary high-priority comparison

Across 7,977 paired high-priority customer deliveries:

- NILS delivered earlier for 3,468 (43.47% of all; 46.92% excluding ties).
- ALNS delivered earlier for 3,923 (49.18% of all; 53.08% excluding ties).
- Ties: 586 (7.35%).
- NILS earlier-delivery lead: mean 58.68, median 43.19, Q25 15.93, Q75 88.87, maximum 281.73 minutes.
- ALNS earlier-delivery lead: mean 96.89, median 62.00, Q25 15.34, Q75 150.31, maximum 444.43 minutes.
- Across all high-priority pairs, ALNS delivery time minus NILS delivery time: mean -22.14, median 0.00, Q25 -59.93, Q75 33.60, min -444.43, max 281.73 minutes.

### Material improvements

- At least 1 minute(s) earlier: NILS 3388 (42.47%); ALNS 3828 (47.99%).
- At least 5 minute(s) earlier: NILS 3118 (39.09%); ALNS 3442 (43.15%).
- At least 10 minute(s) earlier: NILS 2851 (35.74%); ALNS 3149 (39.48%).
- At least 30 minute(s) earlier: NILS 2101 (26.34%); ALNS 2512 (31.49%).

### On-time and tardy transitions

- On time under both: 1,518 (19.03%).
- Tardy under both: 4,166 (52.23%); within this subset, lower tardiness: NILS 1,600, ALNS 2,400, ties 166.
- On time under NILS but tardy under ALNS: 1,347 (16.89%).
- Tardy under NILS but on time under ALNS: 946 (11.86%).
- Across high-priority customers with lower tardiness under NILS by more than 1e-06 minutes, mean / median reduction: 57.40 / 45.82 minutes.

### By size

| n | HP pairs | NILS earlier | ALNS earlier | Ties | Mean advantage (min) | Median advantage (min) | NILS on-time / ALNS tardy | Reverse transition | Mean NILS runtime (s) | Mean ALNS runtime (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15 | 649 | 370 (57.0%) | 141 (21.7%) | 138 (21.3%) | 11.64 | 2.88 | 165 | 19 | 0.69 | 8.88 |
| 50 | 2103 | 1027 (48.8%) | 777 (36.9%) | 299 (14.2%) | 9.61 | 0.00 | 434 | 203 | 7.19 | 39.33 |
| 75 | 1353 | 538 (39.8%) | 705 (52.1%) | 110 (8.1%) | -12.87 | -1.08 | 200 | 177 | 32.28 | 116.05 |
| 100 | 3872 | 1533 (39.6%) | 2300 (59.4%) | 39 (1.0%) | -48.27 | -25.16 | 548 | 547 | 102.08 | 121.19 |

### Fleet intensity

The design couples customer count and available drones. These ratios are descriptive and do not isolate causal effects.

| (n,D) | Customers/drone | Eligible customers/drone (mean) | NILS earlier | ALNS earlier | Ties | Mean NILS advantage (min) |
|---:|---:|---:|---:|---:|---:|---:|
| (50,2) | 25.0 | 12.50 | 291 (56.2%) | 195 (37.6%) | 32 | 11.74 |
| (75,3) | 25.0 | 12.50 | 538 (39.8%) | 705 (52.1%) | 110 | -12.87 |

Cells (50,2) and (75,3) share nominal 25 customers per drone. They differ in other instance factors; eligible workload is reported separately in eligible_workload_by_design_cell.csv.

### Instance-level versus customer-level outcomes

- Lower total high-priority tardiness by instance: NILS 344/720; ALNS 288/720; ties 88/720.
- Aggregate objective wins by instance: NILS 324/720; ALNS 396/720; ties 0/720.
- These instance-level outcomes are distinct from the customer-level earlier-delivery counts above.

The paired Wilcoxon test uses the 720 instances as the independent units and compares each instance's NILS-minus-ALNS difference in high-priority earlier-delivery share. Two-sided p=6.76228e-05; matched-pairs rank-biserial effect=0.1846 (positive favors NILS). Customer rows are not treated as independent.

### Objective, runtime, and priority-service trade-off

See priority_service_computational_tradeoff.csv for objective and runtime means/wins, customer-level earlier-delivery rates, and transition counts by size and exact fleet cell. The results are descriptive; no causal claim is made.

### M3/M4/M5 architecture comparison

- M4 vs M3: first earlier for 3529/7977 (44.24%); second earlier for 3648 (45.73%); on-time transitions first-only=943, second-only=720.
- M5 vs M3: first earlier for 2996/7977 (37.56%); second earlier for 4259 (53.39%); on-time transitions first-only=977, second-only=790.
- M5 vs M4: first earlier for 2644/7977 (33.15%); second earlier for 3995 (50.08%); on-time transitions first-only=748, second-only=784.

The pairwise counts and size/cell detail are in m3_m4_m5_priority_customer_comparison.csv.

## Conservative answers

1. NILS earlier-delivery count/share: 3,468 / 7,977 (43.47%); mean lead among those customers 58.68 minutes.
2. ALNS earlier-delivery count/share: 3,923 / 7,977 (49.18%); mean lead among those customers 96.89 minutes.
3. NILS-only on-time transitions: 1,347; reverse transitions: 946.
4. Direction by n is reported above and should be read size-by-size.
5. The equal n/D cells have the same nominal customers-per-drone ratio but do not control all other factors.
6. A lower ALNS aggregate objective does not by itself show that ALNS delivers more individual high-priority customers earlier; see the paired counts alongside objective wins.
7. Any runtime/priority trade-off is descriptive, not causal.
8. M5 relative to M3 and M4 is summarized above; avoid claims broader than these paired results support.

## Provenance

Frozen instance JSON path: outputs/full_m3_m4_m5_alns_720_frozen_20261106/raw. The directory contains input instances only; its instance hashes were checked against the final input manifest. No historical method outputs were read.

PRIORITY_CUSTOMER_ANALYSIS_VALID
