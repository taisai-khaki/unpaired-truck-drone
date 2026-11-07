import collections
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


root = Path(__file__).resolve().parents[1]
rows_path = root / "checkpoints" / "heuristic_study_rows.jsonl"
rows = [json.loads(line) for line in rows_path.read_text(encoding="utf-8").splitlines() if line]
expected_methods = {
    "paired_baseline",
    "no_unpairing",
    "nils",
    "feasibility_aware_alns",
}
instance_ids = {row["instance_id"] for row in rows}
counts = collections.Counter(row["method_name"] for row in rows)
pair_counts = collections.Counter((row["instance_id"], row["method_name"]) for row in rows)
bad_pairs = [key for key, count in pair_counts.items() if count != 1]
missing_pairs = [
    (instance_id, method)
    for instance_id in instance_ids
    for method in expected_methods
    if pair_counts[(instance_id, method)] != 1
]
not_feasible = [
    row
    for row in rows
    if row.get("status") != "feasible" or row.get("final_solution_feasibility_flag") is not True
]
shared_methods = {"paired_baseline", "no_unpairing", "nils"}
shared_rows = collections.defaultdict(list)
for row in rows:
    if row["method_name"] in shared_methods:
        shared_rows[row["instance_id"]].append(row)
bad_backbones = []
for instance_id, group in shared_rows.items():
    hashes = {row.get("initial_truck_backbone_sha256") for row in group}
    methods = {row["method_name"] for row in group}
    if (
        len(group) != 3
        or methods != shared_methods
        or len(hashes) != 1
        or any(row.get("initial_truck_backbone_reused") is not True for row in group)
    ):
        bad_backbones.append(instance_id)
bad_caps = [
    row
    for row in rows
    if row["method_name"] == "nils"
    and (row.get("max_displacement_candidates") != 100 or row.get("max_repair_candidates") != 100)
]

if len(rows) != 2880 or len(instance_ids) != 720:
    raise SystemExit(f"Unexpected run coverage: rows={len(rows)}, instances={len(instance_ids)}")
if set(counts) != expected_methods or any(counts[method] != 720 for method in expected_methods):
    raise SystemExit(f"Unexpected method coverage: {dict(counts)}")
if bad_pairs or missing_pairs or not_feasible or bad_backbones or bad_caps:
    raise SystemExit(
        "Run audit failed: "
        f"duplicate_pairs={len(bad_pairs)}, missing_pairs={len(missing_pairs)}, "
        f"nonfeasible={len(not_feasible)}, bad_backbones={len(bad_backbones)}, "
        f"bad_caps={len(bad_caps)}"
    )

provenance_path = root / "provenance" / "run_provenance.json"
provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
provenance.update(
    {
        "status": "complete",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "completed_instances": len(instance_ids),
        "completed_method_runs": len(rows),
        "final_feasible_method_runs": len(rows) - len(not_feasible),
        "final_backbone_mismatch_instances": len(bad_backbones),
        "final_result_rows": str(rows_path.relative_to(root.parent.parent)).replace("\\", "/"),
        "final_artifact_sha256_manifest": "provenance/final_artifact_sha256_manifest.csv",
    }
)
provenance_path.write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")

manifest_path = root / "provenance" / "final_artifact_sha256_manifest.csv"
files = sorted(path for path in root.rglob("*") if path.is_file() and path != manifest_path)
with manifest_path.open("w", newline="", encoding="utf-8") as handle:
    writer = csv.writer(handle)
    writer.writerow(["relative_path", "sha256", "bytes"])
    for path in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        writer.writerow([path.relative_to(root).as_posix(), digest, path.stat().st_size])

manifest_sha256 = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
print(
    json.dumps(
        {
            "status": provenance["status"],
            "execution_commit": provenance["execution_commit"],
            "instances": len(instance_ids),
            "method_runs": len(rows),
            "method_counts": dict(sorted(counts.items())),
            "feasible_runs": len(rows) - len(not_feasible),
            "shared_backbone_mismatches": len(bad_backbones),
            "nils_cap_violations": len(bad_caps),
            "hashed_artifact_files": len(files),
            "artifact_manifest": str(manifest_path.relative_to(root)),
            "artifact_manifest_sha256": manifest_sha256,
        },
        indent=2,
    )
)
