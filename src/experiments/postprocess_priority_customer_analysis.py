from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy.stats import wilcoxon

REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET = REPO / "outputs" / "final_m3_m4_m5_alns_720_semantics_corrected_20261113"
DEFAULT_INSTANCE_DIR = REPO / "outputs" / "full_m3_m4_m5_alns_720_frozen_20261106" / "raw"
DATASET_LABEL = "FINAL_720_SEMANTICS_CORRECTED_FROZEN"
RESULT_COMMIT = "fef001ea75c82ccc04b6731334e77b5e75cc626e"
EXECUTION_COMMIT = "8abdc6b7d9f82a7dc630468e4f108e48f2246e84"
METHODS = {
    "M3": "paired_baseline",
    "M4": "no_unpairing",
    "M5": "nils",
    "ALNS": "feasibility_aware_alns",
}
TIE_TOLERANCE = 1e-6


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_gzip_json(path: Path) -> dict[str, Any]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def _keyed_map(entries: Any, label: str) -> dict[tuple[int, ...], float]:
    if not isinstance(entries, list):
        raise ValueError(f"{label} must be a list of keyed values")
    result: dict[tuple[int, ...], float] = {}
    for entry in entries:
        key = tuple(int(part) for part in entry["key"])
        value = float(entry["value"])
        if key in result:
            raise ValueError(f"Duplicate key in {label}: {key}")
        if not math.isfinite(value):
            raise ValueError(f"Non-finite value in {label} at {key}")
        result[key] = value
    return result


def _active_assignment(assignment: dict[tuple[int, ...], float], customer_id: int) -> list[int]:
    return [
        int(key[0])
        for key, value in assignment.items()
        if len(key) == 2 and key[1] == customer_id and value > 0.5
    ]


def reconstruct_customer_delivery(
    solution_payload: dict[str, Any], customer_ids: list[int]
) -> dict[int, dict[str, Any]]:
    """Resolve each customer to its unique active vehicle and arrival variable."""
    u_truck = _keyed_map(solution_payload.get("u_truck"), "u_truck")
    u_drone = _keyed_map(solution_payload.get("u_drone"), "u_drone")
    a_truck = _keyed_map(solution_payload.get("a_truck"), "a_truck")
    a_drone = _keyed_map(solution_payload.get("a_drone"), "a_drone")
    tardiness = {
        int(customer_id): float(value)
        for customer_id, value in solution_payload.get("tardiness", {}).items()
    }
    result: dict[int, dict[str, Any]] = {}
    for customer_id in customer_ids:
        active_trucks = _active_assignment(u_truck, customer_id)
        active_drones = _active_assignment(u_drone, customer_id)
        active_total = len(active_trucks) + len(active_drones)
        if active_total != 1:
            raise ValueError(
                f"Customer {customer_id} has {active_total} active assignments "
                f"(truck={active_trucks}, drone={active_drones})"
            )
        if active_trucks:
            vehicle_id = active_trucks[0]
            key = (vehicle_id, customer_id)
            if key not in a_truck:
                raise ValueError(f"Missing active a_truck{key}")
            delivery_time = a_truck[key]
            service_mode = "truck"
        else:
            vehicle_id = active_drones[0]
            key = (vehicle_id, customer_id)
            if key not in a_drone:
                raise ValueError(f"Missing active a_drone{key}")
            delivery_time = a_drone[key]
            service_mode = "drone"
        if customer_id not in tardiness:
            raise ValueError(f"Missing stored tardiness for customer {customer_id}")
        result[customer_id] = {
            "service_mode": service_mode,
            "serving_vehicle_id": vehicle_id,
            "delivery_time": float(delivery_time),
            "stored_tardiness": tardiness[customer_id],
        }
    return result


def _pct(count: int, total: int) -> float:
    return 100.0 * count / total if total else float("nan")


def _distribution(values: Any, prefix: str) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return {f"{prefix}_{key}": float("nan") for key in ("mean", "median", "q25", "q75", "min", "max")}
    return {
        f"{prefix}_mean": float(np.mean(array)),
        f"{prefix}_median": float(np.median(array)),
        f"{prefix}_q25": float(np.quantile(array, 0.25)),
        f"{prefix}_q75": float(np.quantile(array, 0.75)),
        f"{prefix}_min": float(np.min(array)),
        f"{prefix}_max": float(np.max(array)),
    }


def _comparison_counts(frame: pd.DataFrame, advantage_col: str) -> dict[str, Any]:
    advantage = frame[advantage_col].to_numpy(dtype=float)
    nils_earlier = int(np.sum(advantage > TIE_TOLERANCE))
    alns_earlier = int(np.sum(advantage < -TIE_TOLERANCE))
    tied = int(len(advantage) - nils_earlier - alns_earlier)
    non_tied = nils_earlier + alns_earlier
    return {
        "paired_high_priority_customers": int(len(advantage)),
        "nils_earlier_count": nils_earlier,
        "nils_earlier_pct_all": _pct(nils_earlier, len(advantage)),
        "nils_earlier_pct_excluding_ties": _pct(nils_earlier, non_tied),
        "alns_earlier_count": alns_earlier,
        "alns_earlier_pct_all": _pct(alns_earlier, len(advantage)),
        "alns_earlier_pct_excluding_ties": _pct(alns_earlier, non_tied),
        "tie_count": tied,
        "tie_pct_all": _pct(tied, len(advantage)),
        "mean_delivery_advantage_nils_minutes": float(np.mean(advantage)) if len(advantage) else float("nan"),
        "median_delivery_advantage_nils_minutes": float(np.median(advantage)) if len(advantage) else float("nan"),
    }


def _grouped_summary(frame: pd.DataFrame, group_columns: list[str]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    groups = [((), frame)] if not group_columns else frame.groupby(
        group_columns[0] if len(group_columns) == 1 else group_columns, dropna=False, sort=True
    )
    for key, group in groups:
        key_tuple = key if isinstance(key, tuple) else (key,)
        row = {column: value for column, value in zip(group_columns, key_tuple)}
        row.update(_comparison_counts(group, "delivery_advantage_nils"))
        earlier_nils = group.loc[group.delivery_advantage_nils > TIE_TOLERANCE, "delivery_advantage_nils"]
        earlier_alns = -group.loc[group.delivery_advantage_nils < -TIE_TOLERANCE, "delivery_advantage_nils"]
        row.update(_distribution(earlier_nils, "nils_earlier_magnitude_minutes"))
        row.update(_distribution(earlier_alns, "alns_earlier_magnitude_minutes"))
        row.update(_distribution(group.delivery_advantage_nils, "all_delivery_advantage_minutes"))
        ontime_nils = group.nils_on_time.to_numpy(dtype=bool)
        ontime_alns = group.alns_on_time.to_numpy(dtype=bool)
        row["nils_on_time_alns_tardy_count"] = int(np.sum(ontime_nils & ~ontime_alns))
        row["alns_on_time_nils_tardy_count"] = int(np.sum(ontime_alns & ~ontime_nils))
        row["both_on_time_count"] = int(np.sum(ontime_nils & ontime_alns))
        row["both_tardy_count"] = int(np.sum(~ontime_nils & ~ontime_alns))
        rows.append(row)
    return pd.DataFrame(rows)


def _transition_table(frame: pd.DataFrame, group_columns: list[str]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    groups = [((), frame)] if not group_columns else frame.groupby(
        group_columns[0] if len(group_columns) == 1 else group_columns, dropna=False, sort=True
    )
    categories = (
        ("on_time_both", True, True),
        ("tardy_both", False, False),
        ("on_time_nils_tardy_alns", True, False),
        ("tardy_nils_on_time_alns", False, True),
    )
    for key, group in groups:
        key_tuple = key if isinstance(key, tuple) else (key,)
        row: dict[str, Any] = {column: value for column, value in zip(group_columns, key_tuple)}
        row["high_priority_customer_pairs"] = int(len(group))
        for name, nils_on_time, alns_on_time in categories:
            count = int(np.sum((group.nils_on_time == nils_on_time) & (group.alns_on_time == alns_on_time)))
            row[f"{name}_count"] = count
            row[f"{name}_pct"] = _pct(count, len(group))
        both_tardy = group.loc[~group.nils_on_time & ~group.alns_on_time]
        nils_better = both_tardy.nils_tardiness < both_tardy.alns_tardiness - TIE_TOLERANCE
        alns_better = both_tardy.alns_tardiness < both_tardy.nils_tardiness - TIE_TOLERANCE
        row["both_tardy_nils_lower_tardiness_count"] = int(nils_better.sum())
        row["both_tardy_alns_lower_tardiness_count"] = int(alns_better.sum())
        row["both_tardy_tied_tardiness_count"] = int(len(both_tardy) - nils_better.sum() - alns_better.sum())
        nils_reductions = group.loc[group.alns_tardiness - group.nils_tardiness > TIE_TOLERANCE].eval(
            "alns_tardiness - nils_tardiness"
        )
        both_tardy_reductions = both_tardy.loc[
            both_tardy.alns_tardiness - both_tardy.nils_tardiness > TIE_TOLERANCE
        ].eval("alns_tardiness - nils_tardiness")
        row.update(_distribution(nils_reductions, "nils_tardiness_reduction_when_better_minutes"))
        row.update(_distribution(both_tardy_reductions, "both_tardy_nils_tardiness_reduction_minutes"))
        rows.append(row)
    return pd.DataFrame(rows)


def _load_inputs(manifest_path: Path, instance_dir: Path) -> tuple[list[dict[str, Any]], dict[str, dict[str, Any]]]:
    manifest = pd.read_csv(manifest_path, dtype={"instance_id": str})
    if len(manifest) != 720 or manifest.instance_id.nunique() != 720:
        raise ValueError(f"Expected 720 unique frozen instances, got rows={len(manifest)}")
    instances: dict[str, dict[str, Any]] = {}
    rows = manifest.to_dict("records")
    for row in rows:
        instance_id = str(row["instance_id"])
        path = instance_dir / f"{instance_id}.json"
        if not path.is_file():
            raise FileNotFoundError(f"Frozen instance input is missing: {path}")
        actual_sha = _sha256(path)
        if actual_sha != row["instance_sha256"]:
            raise ValueError(f"Frozen input SHA256 mismatch for {instance_id}")
        data = json.loads(path.read_text(encoding="utf-8"))
        customers = {int(customer["node_id"]): customer for customer in data["customers"]}
        n = int(row["n"])
        if len(customers) != n or set(customers) != set(range(1, n + 1)):
            raise ValueError(f"Frozen customer IDs/count do not match manifest for {instance_id}")
        trucks, drones = data["trucks"], data["drones"]
        if len(drones) != int(row["number_of_drones"]):
            raise ValueError(f"Frozen drone count mismatch for {instance_id}")
        eligible = {int(cid) for cid in data.get("metadata", {}).get("drone_eligible_customers", [])}
        if not eligible.issubset(customers):
            raise ValueError(f"Invalid drone eligibility IDs for {instance_id}")
        if abs(len(eligible) / n - float(row["eligible_share_realized"])) > 1e-12:
            raise ValueError(f"Realized eligible share mismatch for {instance_id}")
        instances[instance_id] = {
            "data": data,
            "customers": customers,
            "eligible": eligible,
            "n_trucks": len(trucks),
            "n_drones": len(drones),
            "input_sha256": actual_sha,
        }
    return rows, instances


def _load_one_method(
    dataset_dir: Path, instance_id: str, method: str, expected_sha: str
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    raw_path = dataset_dir / "raw_returns" / f"{instance_id}__{method}.json.gz"
    record_path = dataset_dir / "run_records" / f"{instance_id}__{method}.json.gz"
    if not raw_path.is_file() or not record_path.is_file():
        raise FileNotFoundError(f"Missing final raw-return/run-record pair for {instance_id}/{method}")
    raw_record, run_record = _read_gzip_json(raw_path), _read_gzip_json(record_path)
    row = run_record.get("row", {})
    for record, label in ((raw_record, "raw return"), (run_record, "run record")):
        if record.get("instance_id") != instance_id or record.get("instance_sha256") != expected_sha:
            raise ValueError(f"{label} instance or input-hash mismatch for {instance_id}/{method}")
        if record.get("method_name") != method:
            raise ValueError(f"{label} method mismatch for {instance_id}/{method}")
    if raw_record.get("raw_returned_solution") != run_record.get("raw_returned_solution"):
        raise ValueError(f"Raw returned solution differs between artifacts for {instance_id}/{method}")
    valid = (
        row.get("instance_id") == instance_id
        and row.get("method_name") == method
        and row.get("full_feasibility") is True
        and row.get("architecture_valid") is True
        and row.get("post_method_repair_applied") is False
        and row.get("post_method_truck_only_fallback_applied") is False
        and str(row.get("method_status", "")).lower() == "feasible"
        and run_record.get("post_method_repair_applied") is False
        and run_record.get("post_method_fallback_applied") is False
    )
    if not valid:
        raise ValueError(f"Feasibility, architecture, or no-repair/fallback check failed for {instance_id}/{method}")
    return raw_record, run_record, row


def _load_observations(dataset_dir: Path, instance_dir: Path) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, dict[str, Any]]:
    manifest_rows, instances = _load_inputs(dataset_dir / "input_manifest.csv", instance_dir)
    frozen_results = pd.read_csv(dataset_dir / "tables" / "raw_720_method_results.csv", dtype={"instance_id": str})
    if frozen_results.duplicated(["instance_id", "method_name"]).any():
        raise ValueError("Duplicate method-instance rows in frozen result table")
    paired_rows: list[dict[str, Any]] = []
    architecture_rows: list[dict[str, Any]] = []
    run_rows: list[dict[str, Any]] = []
    validation = {
        "customer_method_observations_checked": 0,
        "tardiness_mismatches": 0,
        "maximum_absolute_tardiness_reconstruction_difference": 0.0,
        "customers_without_delivery_time": 0,
        "customers_with_missing_or_competing_assignments": 0,
        "truck_drone_competing_assignments": 0,
    }
    for manifest_row in manifest_rows:
        instance_id = str(manifest_row["instance_id"])
        frozen = instances[instance_id]
        customer_map = frozen["customers"]
        customer_ids = sorted(customer_map)
        services: dict[str, dict[int, dict[str, Any]]] = {}
        method_info: dict[str, dict[str, Any]] = {}
        for label, method in METHODS.items():
            raw_record, run_record, result_row = _load_one_method(
                dataset_dir, instance_id, method, str(manifest_row["instance_sha256"])
            )
            payload = raw_record["raw_returned_solution"]
            reconstructed = reconstruct_customer_delivery(payload, customer_ids)
            services[label] = reconstructed
            method_info[label] = {
                "method": method,
                "runtime": float(run_record["method_runtime_seconds"]),
                "objective": float(payload["objective"]),
                "row": result_row,
                "execution_commit": run_record.get("execution_commit_sha"),
            }
            table_row = frozen_results.loc[
                (frozen_results.instance_id == instance_id) & (frozen_results.method_name == method)
            ]
            if len(table_row) != 1:
                raise ValueError(f"Expected one frozen result row for {instance_id}/{method}")
            aggregate = table_row.iloc[0]
            if abs(float(aggregate.objective_total) - method_info[label]["objective"]) > TIE_TOLERANCE:
                raise ValueError(f"Raw objective differs from frozen result table for {instance_id}/{method}")
            if str(aggregate.instance_sha256) != str(manifest_row["instance_sha256"]):
                raise ValueError(f"Result table instance SHA mismatch for {instance_id}/{method}")
            for customer_id in customer_ids:
                customer = customer_map[customer_id]
                service = reconstructed[customer_id]
                delivery = service["delivery_time"]
                if not math.isfinite(delivery):
                    validation["customers_without_delivery_time"] += 1
                    raise ValueError(f"No finite realized delivery time at {instance_id}/{method}/{customer_id}")
                derived = max(0.0, delivery - float(customer["ub"]))
                difference = abs(derived - service["stored_tardiness"])
                validation["customer_method_observations_checked"] += 1
                validation["maximum_absolute_tardiness_reconstruction_difference"] = max(
                    validation["maximum_absolute_tardiness_reconstruction_difference"], difference
                )
                if difference > TIE_TOLERANCE:
                    validation["tardiness_mismatches"] += 1
                    raise ValueError(
                        f"Tardiness mismatch {difference:.12g} at {instance_id}/{method}/{customer_id}"
                    )

        n, d, k = int(manifest_row["n"]), frozen["n_drones"], frozen["n_trucks"]
        eligible_count = len(frozen["eligible"])
        common = {
            "instance_id": instance_id,
            "scenario_id": str(manifest_row["scenario_id"]),
            "scenario_class_id": str(method_info["M5"]["row"].get("scenario_class_id", manifest_row["scenario_id"])),
            "n": n,
            "number_of_trucks": k,
            "number_of_drones": d,
            "customers_per_drone": n / d,
            "drones_per_truck": d / k,
            "eligible_share_target": float(manifest_row["eligible_share_target"]),
            "realized_eligible_share": eligible_count / n,
            "number_of_drone_eligible_customers": eligible_count,
            "eligible_customers_per_drone": eligible_count / d,
            "priority_weight_profile": str(manifest_row["priority_weight_profile"]),
            "spatial_pattern": str(manifest_row["spatial_pattern"]),
        }
        for label in METHODS:
            run_rows.append({
                **common, "method_label": label, "method_name": method_info[label]["method"],
                "objective": method_info[label]["objective"], "runtime_seconds": method_info[label]["runtime"],
                "execution_commit_sha": method_info[label]["execution_commit"],
            })
        for customer_id in customer_ids:
            customer = customer_map[customer_id]
            priority = str(customer["priority"]).lower()
            nils, alns = services["M5"][customer_id], services["ALNS"][customer_id]
            paired_rows.append({
                **common,
                "customer_id": customer_id,
                "priority_class": priority,
                "drone_eligible": customer_id in frozen["eligible"],
                "customer_lb": float(customer["lb"]),
                "customer_ub": float(customer["ub"]),
                "service_time_min": float(customer["service_time_min"]),
                "nils_service_mode": nils["service_mode"],
                "nils_serving_vehicle_id": nils["serving_vehicle_id"],
                "alns_service_mode": alns["service_mode"],
                "alns_serving_vehicle_id": alns["serving_vehicle_id"],
                "nils_delivery_time": nils["delivery_time"],
                "alns_delivery_time": alns["delivery_time"],
                "nils_tardiness": nils["stored_tardiness"],
                "alns_tardiness": alns["stored_tardiness"],
                "nils_minus_alns_delivery_time": nils["delivery_time"] - alns["delivery_time"],
                "delivery_advantage_nils": alns["delivery_time"] - nils["delivery_time"],
                "nils_minus_alns_tardiness": nils["stored_tardiness"] - alns["stored_tardiness"],
                "nils_on_time": nils["delivery_time"] <= float(customer["ub"]),
                "alns_on_time": alns["delivery_time"] <= float(customer["ub"]),
                "nils_objective": method_info["M5"]["objective"],
                "alns_objective": method_info["ALNS"]["objective"],
                "nils_runtime_seconds": method_info["M5"]["runtime"],
                "alns_runtime_seconds": method_info["ALNS"]["runtime"],
                "nils_instance_high_priority_tardiness": float(method_info["M5"]["row"]["high_priority_tardiness_minutes"]),
                "alns_instance_high_priority_tardiness": float(method_info["ALNS"]["row"]["high_priority_tardiness_minutes"]),
            })
            if priority == "high":
                for first, second in (("M4", "M3"), ("M5", "M3"), ("M5", "M4")):
                    first_obs, second_obs = services[first][customer_id], services[second][customer_id]
                    architecture_rows.append({
                        **common,
                        "customer_id": customer_id,
                        "comparison": f"{first}_earlier_than_{second}",
                        "first_method": first,
                        "second_method": second,
                        "first_delivery_time": first_obs["delivery_time"],
                        "second_delivery_time": second_obs["delivery_time"],
                        "delivery_advantage_first_minutes": second_obs["delivery_time"] - first_obs["delivery_time"],
                        "first_on_time": first_obs["delivery_time"] <= float(customer["ub"]),
                        "second_on_time": second_obs["delivery_time"] <= float(customer["ub"]),
                        "first_tardiness": first_obs["stored_tardiness"],
                        "second_tardiness": second_obs["stored_tardiness"],
                    })
    expected_observations = sum(int(row["n"]) for row in manifest_rows) * len(METHODS)
    if validation["customer_method_observations_checked"] != expected_observations:
        raise ValueError(f"Expected {expected_observations} customer-method checks, got {validation['customer_method_observations_checked']}")
    paired = pd.DataFrame(paired_rows)
    if len(paired) != sum(int(row["n"]) for row in manifest_rows) or paired.duplicated(["instance_id", "customer_id"]).any():
        raise ValueError("Missing or duplicate paired customer observations")
    runs = pd.DataFrame(run_rows)
    if len(runs) != 2880 or runs.duplicated(["instance_id", "method_label"]).any():
        raise ValueError("Expected 720 distinct run rows per method")
    return paired, runs, pd.DataFrame(architecture_rows), validation


def _architecture_summary(frame: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for comparison, group in frame.groupby("comparison", sort=True):
        strata = [("overall", "all", group)]
        strata.extend(("size", str(n), group[group.n == n]) for n in (15, 50, 75, 100))
        for n, d in sorted(group[["n", "number_of_drones"]].drop_duplicates().itertuples(index=False, name=None)):
            strata.append(("n_drones_cell", f"{n},{d}", group[(group.n == n) & (group.number_of_drones == d)]))
        for scope, value, subset in strata:
            if subset.empty:
                continue
            delta = subset.delivery_advantage_first_minutes
            first_wins = int((delta > TIE_TOLERANCE).sum())
            second_wins = int((delta < -TIE_TOLERANCE).sum())
            row = {
                "comparison": comparison, "stratum_type": scope, "stratum": value,
                "n": int(subset.n.iloc[0]) if subset.n.nunique() == 1 else "all",
                "number_of_drones": int(subset.number_of_drones.iloc[0]) if subset.number_of_drones.nunique() == 1 else "all",
                "number_of_trucks": int(subset.number_of_trucks.iloc[0]) if subset.number_of_trucks.nunique() == 1 else "all",
                "customers_per_drone": float(subset.customers_per_drone.iloc[0]) if subset.customers_per_drone.nunique() == 1 else float(subset.customers_per_drone.mean()),
                "drones_per_truck": float(subset.drones_per_truck.iloc[0]) if subset.drones_per_truck.nunique() == 1 else float(subset.drones_per_truck.mean()),
                "high_priority_customer_pairs": int(len(subset)),
                "first_method": subset.first_method.iloc[0], "second_method": subset.second_method.iloc[0],
                "first_earlier_count": first_wins, "first_earlier_pct": _pct(first_wins, len(subset)),
                "second_earlier_count": second_wins, "second_earlier_pct": _pct(second_wins, len(subset)),
                "tie_count": int(len(subset) - first_wins - second_wins),
                "mean_delivery_advantage_first_minutes": float(delta.mean()),
                "median_delivery_advantage_first_minutes": float(delta.median()),
                "first_on_time_second_tardy_count": int((subset.first_on_time & ~subset.second_on_time).sum()),
                "second_on_time_first_tardy_count": int((subset.second_on_time & ~subset.first_on_time).sum()),
            }
            row.update(_distribution(delta, "delivery_advantage_first_minutes"))
            rows.append(row)
    return pd.DataFrame(rows)


def _summaries(paired: pd.DataFrame, runs: pd.DataFrame, architecture: pd.DataFrame) -> dict[str, pd.DataFrame]:
    high = paired.loc[paired.priority_class == "high"].copy()
    if high.empty:
        raise ValueError("Frozen instances contain no high-priority customers")
    overall = _grouped_summary(high, [])
    by_size = _grouped_summary(high, ["n"])
    by_cell = _grouped_summary(high, ["n", "number_of_drones"])
    run_sizes = runs.pivot_table(index="n", columns="method_label", values="runtime_seconds", aggfunc="mean")
    run_sizes = run_sizes.rename(columns={
        "M5": "mean_nils_runtime_seconds",
        "ALNS": "mean_alns_runtime_seconds",
    }).reset_index()
    size_meta = runs.drop_duplicates("instance_id").groupby("n", as_index=False).agg(
        instances=("instance_id", "nunique"),
        mean_available_drones=("number_of_drones", "mean"),
        number_of_distinct_fleet_sizes=("number_of_drones", "nunique"),
    )
    by_size = by_size.merge(run_sizes[["n", "mean_nils_runtime_seconds", "mean_alns_runtime_seconds"]], on="n", validate="one_to_one")
    by_size = by_size.merge(size_meta, on="n", validate="one_to_one")
    run_cells = runs.pivot_table(
        index=["n", "number_of_drones"], columns="method_label", values="runtime_seconds", aggfunc="mean"
    ).rename(columns={
        "M5": "mean_nils_runtime_seconds",
        "ALNS": "mean_alns_runtime_seconds",
    }).reset_index()
    cell_meta = runs.drop_duplicates("instance_id").groupby(["n", "number_of_drones"], as_index=False).agg(
        instances=("instance_id", "nunique"),
        number_of_trucks=("number_of_trucks", "first"),
        customers_per_drone=("customers_per_drone", "first"),
        drones_per_truck=("drones_per_truck", "first"),
        eligible_share_target_mean=("eligible_share_target", "mean"),
        realized_eligible_share_mean=("realized_eligible_share", "mean"),
        eligible_customers_per_drone_mean=("eligible_customers_per_drone", "mean"),
        eligible_customers_per_drone_min=("eligible_customers_per_drone", "min"),
        eligible_customers_per_drone_max=("eligible_customers_per_drone", "max"),
    )
    by_cell = by_cell.merge(run_cells, on=["n", "number_of_drones"], validate="one_to_one")
    by_cell = by_cell.merge(cell_meta, on=["n", "number_of_drones"], validate="one_to_one")
    transitions = _transition_table(high, [])
    transitions_size = _transition_table(high, ["n"])
    transitions_cell = _transition_table(high, ["n", "number_of_drones"])
    run_pivot = runs.pivot(index="instance_id", columns="method_label", values=["objective", "runtime_seconds"])
    run_meta = runs.drop_duplicates("instance_id").set_index("instance_id")[[
        "n", "number_of_drones", "number_of_trucks", "customers_per_drone",
        "drones_per_truck", "eligible_share_target", "realized_eligible_share",
        "number_of_drone_eligible_customers", "eligible_customers_per_drone",
    ]]
    objective_runtime = pd.concat([run_meta, run_pivot], axis=1).reset_index()
    objective_runtime.columns = [
        "_".join(str(part) for part in column if part != "").strip("_") if isinstance(column, tuple) else str(column)
        for column in objective_runtime.columns
    ]
    objective_runtime = objective_runtime.rename(columns={
        "objective_M5": "nils_objective", "objective_ALNS": "alns_objective",
        "runtime_seconds_M5": "nils_runtime_seconds", "runtime_seconds_ALNS": "alns_runtime_seconds",
    })
    hp_instance = high.groupby("instance_id", as_index=False).agg(
        high_priority_customers=("customer_id", "count"),
        nils_earlier_count=("delivery_advantage_nils", lambda x: int((x > TIE_TOLERANCE).sum())),
        alns_earlier_count=("delivery_advantage_nils", lambda x: int((x < -TIE_TOLERANCE).sum())),
        tied_count=("delivery_advantage_nils", lambda x: int((x.abs() <= TIE_TOLERANCE).sum())),
        nils_high_priority_tardiness=("nils_tardiness", "sum"),
        alns_high_priority_tardiness=("alns_tardiness", "sum"),
    )
    instance = objective_runtime.merge(hp_instance, on="instance_id", how="left", validate="one_to_one")
    count_columns = [
        "high_priority_customers", "nils_earlier_count", "alns_earlier_count", "tied_count",
        "nils_high_priority_tardiness", "alns_high_priority_tardiness",
    ]
    instance[count_columns] = instance[count_columns].fillna(0)
    instance["no_high_priority_customers"] = instance.high_priority_customers == 0
    instance["nils_earlier_fraction"] = np.where(
        instance.high_priority_customers > 0,
        instance.nils_earlier_count / instance.high_priority_customers.replace(0, np.nan),
        np.nan,
    )
    instance["alns_earlier_fraction"] = np.where(
        instance.high_priority_customers > 0,
        instance.alns_earlier_count / instance.high_priority_customers.replace(0, np.nan),
        np.nan,
    )
    instance["customer_earlier_fraction_difference_nils_minus_alns"] = np.where(
        instance.high_priority_customers > 0,
        instance.nils_earlier_fraction - instance.alns_earlier_fraction,
        0.0,
    )
    hp_instance["nils_lower_instance_high_priority_tardiness"] = hp_instance.nils_high_priority_tardiness < hp_instance.alns_high_priority_tardiness - TIE_TOLERANCE
    hp_instance["alns_lower_instance_high_priority_tardiness"] = hp_instance.alns_high_priority_tardiness < hp_instance.nils_high_priority_tardiness - TIE_TOLERANCE
    hp_instance["instance_high_priority_tardiness_tie"] = ~hp_instance.nils_lower_instance_high_priority_tardiness & ~hp_instance.alns_lower_instance_high_priority_tardiness
    instance = instance.drop(columns=[
        "nils_lower_instance_high_priority_tardiness",
        "alns_lower_instance_high_priority_tardiness",
        "instance_high_priority_tardiness_tie",
    ], errors="ignore")
    instance["nils_lower_instance_high_priority_tardiness"] = (
        instance.nils_high_priority_tardiness < instance.alns_high_priority_tardiness - TIE_TOLERANCE
    )
    instance["alns_lower_instance_high_priority_tardiness"] = (
        instance.alns_high_priority_tardiness < instance.nils_high_priority_tardiness - TIE_TOLERANCE
    )
    instance["instance_high_priority_tardiness_tie"] = (
        ~instance.nils_lower_instance_high_priority_tardiness
        & ~instance.alns_lower_instance_high_priority_tardiness
    )
    instance["nils_objective_win"] = instance.nils_objective < instance.alns_objective - TIE_TOLERANCE
    instance["alns_objective_win"] = instance.alns_objective < instance.nils_objective - TIE_TOLERANCE
    instance["objective_tie"] = ~instance.nils_objective_win & ~instance.alns_objective_win

    def tradeoff(group_columns: list[str]) -> pd.DataFrame:
        groups = objective_runtime.groupby(
            group_columns[0] if len(group_columns) == 1 else group_columns, dropna=False, sort=True
        )
        result = []
        for key, runs_group in groups:
            key_tuple = key if isinstance(key, tuple) else (key,)
            row = {column: value for column, value in zip(group_columns, key_tuple)}
            mask = pd.Series(True, index=high.index)
            for column, value in zip(group_columns, key_tuple):
                mask &= high[column] == value
            customer_group = high.loc[mask]
            obj_delta = runs_group.nils_objective - runs_group.alns_objective
            nils_obj_wins = int((obj_delta < -TIE_TOLERANCE).sum())
            row.update({
                "instances": int(runs_group.instance_id.nunique()),
                "high_priority_customer_pairs": int(len(customer_group)),
                "mean_nils_objective": float(runs_group.nils_objective.mean()),
                "mean_alns_objective": float(runs_group.alns_objective.mean()),
                "mean_nils_runtime_seconds": float(runs_group.nils_runtime_seconds.mean()),
                "mean_alns_runtime_seconds": float(runs_group.alns_runtime_seconds.mean()),
                "nils_objective_wins": nils_obj_wins,
                "nils_objective_win_rate_pct": _pct(nils_obj_wins, len(obj_delta)),
                "alns_objective_wins": int((obj_delta > TIE_TOLERANCE).sum()),
                "objective_ties": int((obj_delta.abs() <= TIE_TOLERANCE).sum()),
                "nils_high_priority_earlier_count": int((customer_group.delivery_advantage_nils > TIE_TOLERANCE).sum()),
                "nils_high_priority_earlier_pct_all": _pct(int((customer_group.delivery_advantage_nils > TIE_TOLERANCE).sum()), len(customer_group)),
                "alns_high_priority_earlier_count": int((customer_group.delivery_advantage_nils < -TIE_TOLERANCE).sum()),
                "alns_high_priority_earlier_pct_all": _pct(int((customer_group.delivery_advantage_nils < -TIE_TOLERANCE).sum()), len(customer_group)),
                "high_priority_delivery_ties": int((customer_group.delivery_advantage_nils.abs() <= TIE_TOLERANCE).sum()),
                "nils_on_time_alns_tardy_count": int((customer_group.nils_on_time & ~customer_group.alns_on_time).sum()),
                "alns_on_time_nils_tardy_count": int((customer_group.alns_on_time & ~customer_group.nils_on_time).sum()),
                "mean_nils_minus_alns_high_priority_tardiness_by_customer": float(customer_group.nils_minus_alns_tardiness.mean()),
            })
            result.append(row)
        return pd.DataFrame(result)

    workload = high.groupby(["n", "number_of_drones", "eligible_customers_per_drone"], as_index=False).agg(
        high_priority_customer_pairs=("customer_id", "count"),
        nils_earlier_count=("delivery_advantage_nils", lambda x: int((x > TIE_TOLERANCE).sum())),
        alns_earlier_count=("delivery_advantage_nils", lambda x: int((x < -TIE_TOLERANCE).sum())),
        tie_count=("delivery_advantage_nils", lambda x: int((x.abs() <= TIE_TOLERANCE).sum())),
        mean_delivery_advantage_nils_minutes=("delivery_advantage_nils", "mean"),
    )
    workload["nils_earlier_pct"] = 100 * workload.nils_earlier_count / workload.high_priority_customer_pairs
    workload["alns_earlier_pct"] = 100 * workload.alns_earlier_count / workload.high_priority_customer_pairs
    workload_cell = workload.groupby(["n", "number_of_drones"], as_index=False).agg(
        observed_eligible_customers_per_drone_min=("eligible_customers_per_drone", "min"),
        observed_eligible_customers_per_drone_mean=("eligible_customers_per_drone", "mean"),
        observed_eligible_customers_per_drone_max=("eligible_customers_per_drone", "max"),
        distinct_observed_workloads=("eligible_customers_per_drone", "nunique"),
        high_priority_customer_pairs=("high_priority_customer_pairs", "sum"),
    )
    class_rows = []
    for priority, group in paired.groupby("priority_class", sort=True):
        class_rows.append({"priority_class": priority, **_comparison_counts(group, "delivery_advantage_nils")})
    return {
        "high": high, "overall": overall, "by_size": by_size, "by_cell": by_cell,
        "transitions": transitions, "transitions_size": transitions_size,
        "transitions_cell": transitions_cell, "instance": instance,
        "tradeoff_size": tradeoff(["n"]), "tradeoff_cell": tradeoff(["n", "number_of_drones"]),
        "workload": workload, "workload_cell": workload_cell,
        "architecture": _architecture_summary(architecture),
        "class_summary": pd.DataFrame(class_rows),
    }


def _statistical_test(instance_frame: pd.DataFrame) -> pd.DataFrame:
    values = instance_frame.customer_earlier_fraction_difference_nils_minus_alns.to_numpy(dtype=float)
    nonzero = values[np.abs(values) > TIE_TOLERANCE]
    if nonzero.size == 0:
        statistic, pvalue, positive_sum, negative_sum, rbc = 0.0, 1.0, 0.0, 0.0, 0.0
    else:
        test = wilcoxon(values, zero_method="wilcox", correction=False, alternative="two-sided", method="auto")
        statistic, pvalue = float(test.statistic), float(test.pvalue)
        ranks = pd.Series(np.abs(nonzero)).rank(method="average").to_numpy(dtype=float)
        positive_sum = float(ranks[nonzero > 0].sum())
        negative_sum = float(ranks[nonzero < 0].sum())
        rbc = (positive_sum - negative_sum) / (positive_sum + negative_sum) if positive_sum + negative_sum else 0.0
    return pd.DataFrame([{
        "independent_paired_units": int(len(values)),
        "instances_without_high_priority_customers": int(instance_frame.no_high_priority_customers.sum()),
        "nonzero_instance_pairs": int(nonzero.size),
        "estimand": "within-instance NILS-earlier share minus ALNS-earlier share among high-priority customers",
        "mean_instance_difference": float(np.mean(values)),
        "median_instance_difference": float(np.median(values)),
        "wilcoxon_signed_rank_statistic_min_rank_sum": statistic,
        "wilcoxon_two_sided_p_value": pvalue,
        "positive_rank_sum": positive_sum,
        "negative_rank_sum": negative_sum,
        "matched_pairs_rank_biserial_correlation_positive_favors_nils": rbc,
        "customer_observations_treated_as_independent": False,
    }])


def _report(
    summary: dict[str, pd.DataFrame], stats: pd.DataFrame, validation: dict[str, Any],
    manifest: dict[str, Any],
) -> str:
    high = summary["high"]
    all_counts = _comparison_counts(high, "delivery_advantage_nils")
    nils_earlier = high.loc[high.delivery_advantage_nils > TIE_TOLERANCE, "delivery_advantage_nils"]
    alns_earlier = -high.loc[high.delivery_advantage_nils < -TIE_TOLERANCE, "delivery_advantage_nils"]
    transitions = summary["transitions"].iloc[0]
    test = stats.iloc[0]
    size_lines = []
    for _, row in summary["by_size"].sort_values("n").iterrows():
        runtimes = summary["tradeoff_size"].loc[summary["tradeoff_size"].n == row.n].iloc[0]
        size_lines.append(
            f"| {int(row.n)} | {int(row.paired_high_priority_customers)} | "
            f"{int(row.nils_earlier_count)} ({row.nils_earlier_pct_all:.1f}%) | "
            f"{int(row.alns_earlier_count)} ({row.alns_earlier_pct_all:.1f}%) | "
            f"{int(row.tie_count)} ({row.tie_pct_all:.1f}%) | "
            f"{row.mean_delivery_advantage_nils_minutes:.2f} | {row.median_delivery_advantage_nils_minutes:.2f} | "
            f"{int(row.nils_on_time_alns_tardy_count)} | {int(row.alns_on_time_nils_tardy_count)} | "
            f"{runtimes.mean_nils_runtime_seconds:.2f} | {runtimes.mean_alns_runtime_seconds:.2f} |"
        )
    equal_cells = summary["by_cell"].loc[
        ((summary["by_cell"].n == 50) & (summary["by_cell"].number_of_drones == 2))
        | ((summary["by_cell"].n == 75) & (summary["by_cell"].number_of_drones == 3))
    ]
    equal_lines = []
    for _, row in equal_cells.iterrows():
        workloads = summary["workload_cell"].loc[
            (summary["workload_cell"].n == row.n)
            & (summary["workload_cell"].number_of_drones == row.number_of_drones)
        ].iloc[0]
        equal_lines.append(
            f"| ({int(row.n)},{int(row.number_of_drones)}) | {row.customers_per_drone:.1f} | "
            f"{workloads.observed_eligible_customers_per_drone_mean:.2f} | "
            f"{int(row.nils_earlier_count)} ({row.nils_earlier_pct_all:.1f}%) | "
            f"{int(row.alns_earlier_count)} ({row.alns_earlier_pct_all:.1f}%) | "
            f"{int(row.tie_count)} | {row.mean_delivery_advantage_nils_minutes:.2f} |"
        )
    material_lines = []
    for threshold in (1, 5, 10, 30):
        n_count = int((high.delivery_advantage_nils >= threshold).sum())
        a_count = int((high.delivery_advantage_nils <= -threshold).sum())
        material_lines.append(
            f"- At least {threshold} minute(s) earlier: NILS {n_count} ({_pct(n_count, len(high)):.2f}%); "
            f"ALNS {a_count} ({_pct(a_count, len(high)):.2f}%)."
        )
    tardy_both = high.loc[~high.nils_on_time & ~high.alns_on_time]
    n_tardy_better = int((tardy_both.nils_tardiness < tardy_both.alns_tardiness - TIE_TOLERANCE).sum())
    a_tardy_better = int((tardy_both.alns_tardiness < tardy_both.nils_tardiness - TIE_TOLERANCE).sum())
    n_obj_wins = int((summary["instance"].nils_objective < summary["instance"].alns_objective - TIE_TOLERANCE).sum())
    a_obj_wins = int((summary["instance"].alns_objective < summary["instance"].nils_objective - TIE_TOLERANCE).sum())
    instance_nils_hpt = int(summary["instance"].nils_lower_instance_high_priority_tardiness.sum())
    instance_alns_hpt = int(summary["instance"].alns_lower_instance_high_priority_tardiness.sum())
    n_obj_wins = int(summary["instance"].nils_objective_win.sum())
    a_obj_wins = int(summary["instance"].alns_objective_win.sum())
    architecture_lines = []
    for label in ("M4_earlier_than_M3", "M5_earlier_than_M3", "M5_earlier_than_M4"):
        row = summary["architecture"].query("comparison == @label and stratum_type == 'overall'").iloc[0]
        architecture_lines.append(
            f"- {row.first_method} vs {row.second_method}: first earlier for {int(row.first_earlier_count)}/"
            f"{int(row.high_priority_customer_pairs)} ({row.first_earlier_pct:.2f}%); second earlier for "
            f"{int(row.second_earlier_count)} ({row.second_earlier_pct:.2f}%); on-time transitions first-only="
            f"{int(row.first_on_time_second_tardy_count)}, second-only={int(row.second_on_time_first_tardy_count)}."
        )
    return f"""# Priority-customer paired delivery analysis

## Scope and validation

- Dataset: {DATASET_LABEL}; result commit {RESULT_COMMIT}.
- Frozen instances: {manifest["frozen_instances_used"]}; NILS solutions: {manifest["nils_solutions_used"]}; ALNS solutions: {manifest["alns_solutions_used"]}.
- No algorithm reruns, solution modifications, instance regeneration, historical method results, or superseded one-record results were used.
- Frozen JSON input hashes all match the final input manifest.
- Customer-method observations checked across M3, M4, M5/NILS, and ALNS: {validation["customer_method_observations_checked"]:,}; tardiness mismatches: {validation["tardiness_mismatches"]}; maximum absolute reconstruction difference: {validation["maximum_absolute_tardiness_reconstruction_difference"]:.12g} minutes.
- All observations have exactly one active truck-or-drone service assignment and a realized delivery time. No competing truck/drone assignments.
- {int(summary["instance"].no_high_priority_customers.sum())} instances contain no high-priority customers. Their high-priority earlier-share difference is set to zero for the 720-unit paired test because no customer comparison is defined; they are ties in total high-priority tardiness.
- Delivery-time tie tolerance: {TIE_TOLERANCE:g} minutes. On time means realized delivery time <= customer ub.

## Primary high-priority comparison

Across {all_counts["paired_high_priority_customers"]:,} paired high-priority customer deliveries:

- NILS delivered earlier for {all_counts["nils_earlier_count"]:,} ({all_counts["nils_earlier_pct_all"]:.2f}% of all; {all_counts["nils_earlier_pct_excluding_ties"]:.2f}% excluding ties).
- ALNS delivered earlier for {all_counts["alns_earlier_count"]:,} ({all_counts["alns_earlier_pct_all"]:.2f}% of all; {all_counts["alns_earlier_pct_excluding_ties"]:.2f}% excluding ties).
- Ties: {all_counts["tie_count"]:,} ({all_counts["tie_pct_all"]:.2f}%).
- NILS earlier-delivery lead: mean {nils_earlier.mean():.2f}, median {nils_earlier.median():.2f}, Q25 {nils_earlier.quantile(.25):.2f}, Q75 {nils_earlier.quantile(.75):.2f}, maximum {nils_earlier.max():.2f} minutes.
- ALNS earlier-delivery lead: mean {alns_earlier.mean():.2f}, median {alns_earlier.median():.2f}, Q25 {alns_earlier.quantile(.25):.2f}, Q75 {alns_earlier.quantile(.75):.2f}, maximum {alns_earlier.max():.2f} minutes.
- Across all high-priority pairs, ALNS delivery time minus NILS delivery time: mean {high.delivery_advantage_nils.mean():.2f}, median {high.delivery_advantage_nils.median():.2f}, Q25 {high.delivery_advantage_nils.quantile(.25):.2f}, Q75 {high.delivery_advantage_nils.quantile(.75):.2f}, min {high.delivery_advantage_nils.min():.2f}, max {high.delivery_advantage_nils.max():.2f} minutes.

### Material improvements

{chr(10).join(material_lines)}

### On-time and tardy transitions

- On time under both: {int(transitions.on_time_both_count):,} ({transitions.on_time_both_pct:.2f}%).
- Tardy under both: {int(transitions.tardy_both_count):,} ({transitions.tardy_both_pct:.2f}%); within this subset, lower tardiness: NILS {n_tardy_better:,}, ALNS {a_tardy_better:,}, ties {int(transitions.both_tardy_tied_tardiness_count):,}.
- On time under NILS but tardy under ALNS: {int(transitions.on_time_nils_tardy_alns_count):,} ({transitions.on_time_nils_tardy_alns_pct:.2f}%).
- Tardy under NILS but on time under ALNS: {int(transitions.tardy_nils_on_time_alns_count):,} ({transitions.tardy_nils_on_time_alns_pct:.2f}%).
- Across high-priority customers with lower tardiness under NILS by more than {TIE_TOLERANCE:g} minutes, mean / median reduction: {high.loc[high.alns_tardiness - high.nils_tardiness > TIE_TOLERANCE].eval("alns_tardiness - nils_tardiness").mean():.2f} / {high.loc[high.alns_tardiness - high.nils_tardiness > TIE_TOLERANCE].eval("alns_tardiness - nils_tardiness").median():.2f} minutes.

### By size

| n | HP pairs | NILS earlier | ALNS earlier | Ties | Mean advantage (min) | Median advantage (min) | NILS on-time / ALNS tardy | Reverse transition | Mean NILS runtime (s) | Mean ALNS runtime (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(size_lines)}

### Fleet intensity

The design couples customer count and available drones. These ratios are descriptive and do not isolate causal effects.

| (n,D) | Customers/drone | Eligible customers/drone (mean) | NILS earlier | ALNS earlier | Ties | Mean NILS advantage (min) |
|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(equal_lines)}

Cells (50,2) and (75,3) share nominal 25 customers per drone. They differ in other instance factors; eligible workload is reported separately in eligible_workload_by_design_cell.csv.

### Instance-level versus customer-level outcomes

- Lower total high-priority tardiness by instance: NILS {instance_nils_hpt}/720; ALNS {instance_alns_hpt}/720; ties {int(summary["instance"].instance_high_priority_tardiness_tie.sum())}/720.
- Aggregate objective wins by instance: NILS {n_obj_wins}/720; ALNS {a_obj_wins}/720; ties {int(summary["instance"].objective_tie.sum())}/720.
- These instance-level outcomes are distinct from the customer-level earlier-delivery counts above.

The paired Wilcoxon test uses the 720 instances as the independent units and compares each instance's NILS-minus-ALNS difference in high-priority earlier-delivery share. Two-sided p={test.wilcoxon_two_sided_p_value:.6g}; matched-pairs rank-biserial effect={test.matched_pairs_rank_biserial_correlation_positive_favors_nils:.4f} (positive favors NILS). Customer rows are not treated as independent.

### Objective, runtime, and priority-service trade-off

See priority_service_computational_tradeoff.csv for objective and runtime means/wins, customer-level earlier-delivery rates, and transition counts by size and exact fleet cell. The results are descriptive; no causal claim is made.

### M3/M4/M5 architecture comparison

{chr(10).join(architecture_lines)}

The pairwise counts and size/cell detail are in m3_m4_m5_priority_customer_comparison.csv.

## Conservative answers

1. NILS earlier-delivery count/share: {all_counts["nils_earlier_count"]:,} / {all_counts["paired_high_priority_customers"]:,} ({all_counts["nils_earlier_pct_all"]:.2f}%); mean lead among those customers {nils_earlier.mean():.2f} minutes.
2. ALNS earlier-delivery count/share: {all_counts["alns_earlier_count"]:,} / {all_counts["paired_high_priority_customers"]:,} ({all_counts["alns_earlier_pct_all"]:.2f}%); mean lead among those customers {alns_earlier.mean():.2f} minutes.
3. NILS-only on-time transitions: {int(transitions.on_time_nils_tardy_alns_count):,}; reverse transitions: {int(transitions.tardy_nils_on_time_alns_count):,}.
4. Direction by n is reported above and should be read size-by-size.
5. The equal n/D cells have the same nominal customers-per-drone ratio but do not control all other factors.
6. A lower ALNS aggregate objective does not by itself show that ALNS delivers more individual high-priority customers earlier; see the paired counts alongside objective wins.
7. Any runtime/priority trade-off is descriptive, not causal.
8. M5 relative to M3 and M4 is summarized above; avoid claims broader than these paired results support.

## Provenance

Frozen instance JSON path: {manifest["frozen_instance_input_dir"]}. The directory contains input instances only; its instance hashes were checked against the final input manifest. No historical method outputs were read.

{manifest["validity_status"]}
"""


def run_analysis(dataset_dir: Path = DEFAULT_DATASET, instance_dir: Path = DEFAULT_INSTANCE_DIR) -> Path:
    dataset_dir, instance_dir = dataset_dir.resolve(), instance_dir.resolve()
    source = json.loads((dataset_dir / "run_manifest.json").read_text(encoding="utf-8"))
    if source.get("dataset_status") != DATASET_LABEL or source.get("completed_method_instance_runs") != 2880:
        raise ValueError("Source is not the completed frozen 720-instance dataset")
    if source.get("execution_commit") != EXECUTION_COMMIT:
        raise ValueError("Frozen source execution commit differs from authorized provenance")
    if _sha256(dataset_dir / "run_config.yaml") != source.get("configuration_sha256"):
        raise ValueError("Frozen run configuration hash mismatch")
    output_dir = dataset_dir / "priority_customer_analysis"
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Refusing to overwrite existing post-processing directory: {output_dir}")
    paired, runs, arch_pairs, validation = _load_observations(dataset_dir, instance_dir)
    summary = _summaries(paired, runs, arch_pairs)
    tests = _statistical_test(summary["instance"])
    summary["statistical_tests"] = tests
    tradeoff = pd.concat([
        summary["tradeoff_size"].assign(
            stratum_type="size", stratum=summary["tradeoff_size"].n.astype(str)
        ),
        summary["tradeoff_cell"].assign(
            stratum_type="n_drones_cell",
            stratum=summary["tradeoff_cell"].apply(lambda row: f"{int(row.n)},{int(row.number_of_drones)}", axis=1),
        ),
    ], ignore_index=True, sort=False)
    high = summary["high"]
    material_rows = []
    for threshold in (1, 5, 10, 30):
        for method, count in (
            ("NILS", int((high.delivery_advantage_nils >= threshold).sum())),
            ("ALNS", int((high.delivery_advantage_nils <= -threshold).sum())),
        ):
            material_rows.append({
                "earlier_method": method, "threshold_minutes": threshold,
                "high_priority_customer_count": count,
                "pct_of_all_high_priority_customers": _pct(count, len(high)),
                "numerical_tie_tolerance_minutes": TIE_TOLERANCE,
            })
    outputs = {
        "priority_customer_paired_raw.csv": (paired, "Paired NILS/ALNS customer records for all priority classes"),
        "high_priority_customer_overall.csv": (summary["overall"], "Pooled high-priority paired customer comparison"),
        "high_priority_customer_by_size.csv": (summary["by_size"], "High-priority paired comparison by customer count"),
        "high_priority_customer_by_n_drones.csv": (summary["by_cell"], "High-priority paired comparison by exact n/drone cell"),
        "high_priority_ontime_transitions.csv": (summary["transitions"], "Overall high-priority on-time transitions and tardiness"),
        "high_priority_material_improvements.csv": (pd.DataFrame(material_rows), "Earlier-delivery thresholds of 1, 5, 10, and 30 minutes"),
        "priority_service_computational_tradeoff.csv": (tradeoff, "Objective, runtime, and customer-level priority comparison by size/cell"),
        "m3_m4_m5_priority_customer_comparison.csv": (summary["architecture"], "M3/M4/M5 paired high-priority customer comparison"),
        "priority_customer_statistical_tests.csv": (tests, "Instance-level paired Wilcoxon test and rank-biserial effect"),
        "high_priority_customer_by_eligible_workload.csv": (summary["workload"], "High-priority comparison by exact eligible workload per drone"),
        "eligible_workload_by_design_cell.csv": (summary["workload_cell"], "Observed eligible workload range/mean by fleet cell"),
        "high_priority_by_size_ontime_transitions.csv": (summary["transitions_size"], "On-time transitions by customer count"),
        "high_priority_by_cell_ontime_transitions.csv": (summary["transitions_cell"], "On-time transitions by exact fleet cell"),
        "high_priority_instance_vs_customer_level.csv": (summary["instance"], "Side-by-side instance and customer paired results"),
        "priority_customer_by_class_overall.csv": (
            pd.DataFrame([
                {"priority_class": cls, **_comparison_counts(group, "delivery_advantage_nils")}
                for cls, group in paired.groupby("priority_class", sort=True)
            ]),
            "Secondary earlier/tie/later counts by priority class",
        ),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    for filename, (frame, _) in outputs.items():
        frame.to_csv(output_dir / filename, index=False, float_format="%.10f", na_rep="NA")
    manifest = {
        "dataset_label": DATASET_LABEL,
        "source_result_commit": RESULT_COMMIT,
        "source_execution_commit": EXECUTION_COMMIT,
        "source_dataset_path": dataset_dir.relative_to(REPO).as_posix(),
        "frozen_instance_input_dir": instance_dir.relative_to(REPO).as_posix(),
        "frozen_instances_used": 720,
        "nils_solutions_used": 720,
        "alns_solutions_used": 720,
        "m3_solutions_used": 720,
        "m4_solutions_used": 720,
        "all_method_solutions_used": 2880,
        "paired_customer_records": int(len(paired)),
        "high_priority_paired_customer_observations": int(len(high)),
        "instances_without_high_priority_customers": int(summary["instance"].no_high_priority_customers.sum()),
        "customer_method_observations_checked": validation["customer_method_observations_checked"],
        "tardiness_mismatches": validation["tardiness_mismatches"],
        "maximum_absolute_tardiness_reconstruction_difference": validation["maximum_absolute_tardiness_reconstruction_difference"],
        "customers_without_delivery_time": validation["customers_without_delivery_time"],
        "customers_with_missing_or_competing_assignments": validation["customers_with_missing_or_competing_assignments"],
        "truck_drone_competing_assignments": validation["truck_drone_competing_assignments"],
        "numerical_tie_tolerance_minutes": TIE_TOLERANCE,
        "on_time_definition": "realized delivery time <= frozen customer ub",
        "tardiness_reconstruction": "max(0, realized delivery time - frozen customer ub)",
        "algorithms_rerun": False,
        "solutions_modified": False,
        "instances_regenerated": False,
        "historical_method_results_used": False,
        "superseded_one_record_attempt_used": False,
        "tardiness_reconstruction_validated": validation["tardiness_mismatches"] == 0,
        "all_720_instances_have_exact_nils_and_alns_pair": True,
        "all_2880_runs_feasible_architecture_valid_and_unmodified": True,
        "statistical_unit": "instance; 720 paired units",
        "primary_inference": "two-sided paired Wilcoxon signed-rank test on within-instance high-priority earlier-delivery-share difference",
        "validity_status": "PRIORITY_CUSTOMER_ANALYSIS_VALID",
        "validation": validation,
        "outputs": {},
    }
    for filename, (_, description) in outputs.items():
        manifest["outputs"][filename] = {"description": description, "sha256": _sha256(output_dir / filename)}
    manifest_path = output_dir / "priority_customer_analysis_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    (output_dir / "priority_customer_analysis_report.md").write_text(
        _report(summary, tests, validation, manifest), encoding="utf-8"
    )
    sidecar = {
        "report_sha256": _sha256(output_dir / "priority_customer_analysis_report.md"),
        "manifest_sha256": _sha256(manifest_path),
    }
    (output_dir / "report_manifest_hashes.json").write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return output_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Post-process frozen 720-instance priority-customer results.")
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--instance-dir", type=Path, default=DEFAULT_INSTANCE_DIR)
    args = parser.parse_args()
    print(f"Priority-customer analysis written to {run_analysis(args.dataset_dir, args.instance_dir)}")


if __name__ == "__main__":
    main()
