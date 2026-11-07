from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[2]
OUTPUT = Path(__file__).resolve().parent
SOURCE_OUTPUT = REPO / "outputs/full_revised_usync_20261101_paper_ready"
SOURCE_RAW = SOURCE_OUTPUT / "raw"
SOURCE_ROWS = SOURCE_OUTPUT / "paper_method_rows_partial.csv"
BASE_CONFIG = REPO / "config/experiment_full_revised_usync_20261030_budgeted.yaml"
RUN_CONFIG = OUTPUT / "run_config.yaml"
MANIFEST = OUTPUT / "provenance/file_sha256_manifest.csv"
PROVENANCE = OUTPUT / "provenance/run_provenance.json"
EXPECTED_COMMIT = "a414cf2b2e2b16faaa514504dc38266a9245c0b2"
METHODS = ["paired_baseline", "no_unpairing", "nils", "feasibility_aware_alns"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    if commit != EXPECTED_COMMIT:
        raise RuntimeError(f"Unexpected execution commit: {commit}")
    tracked_status = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO, text=True
    )
    if tracked_status:
        raise RuntimeError(f"Tracked source tree is not clean: {tracked_status}")

    source_files = sorted(SOURCE_RAW.glob("*.json"))
    input_files = sorted((OUTPUT / "raw").glob("*.json"))
    if len(source_files) != 720 or len(input_files) != 720:
        raise RuntimeError(f"Expected 720 source and copied inputs, got {len(source_files)} and {len(input_files)}")
    if [path.name for path in source_files] != [path.name for path in input_files]:
        raise RuntimeError("Copied input filenames differ from the source instance set")
    for source, target in zip(source_files, input_files):
        if sha256(source) != sha256(target):
            raise RuntimeError(f"Input copy hash mismatch for {source.name}")

    with SOURCE_ROWS.open(newline="", encoding="utf-8-sig") as stream:
        historical_rows = list(csv.DictReader(stream))
    instance_ids = {row["instance_id"] for row in historical_rows}
    if len(instance_ids) != 720 or instance_ids != {path.stem for path in source_files}:
        raise RuntimeError("Input instances do not match the saved 720-instance result design")

    config = yaml.safe_load(BASE_CONFIG.read_text(encoding="utf-8"))
    config["experiment"]["output_dir"] = OUTPUT.relative_to(REPO).as_posix()
    config["experiment"]["study_methods"] = METHODS
    config["experiment"]["study_resume"] = True
    config["experiment"]["study_max_new_instances_per_run"] = 0
    RUN_CONFIG.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8")

    manifest_rows = []
    for path in sorted((REPO / "src").rglob("*.py")):
        manifest_rows.append(
            {"role": "source_code", "path": path.relative_to(REPO).as_posix(), "sha256": sha256(path)}
        )
    manifest_rows.extend(
        [
            {"role": "base_config", "path": BASE_CONFIG.relative_to(REPO).as_posix(), "sha256": sha256(BASE_CONFIG)},
            {"role": "effective_config", "path": RUN_CONFIG.relative_to(REPO).as_posix(), "sha256": sha256(RUN_CONFIG)},
            {"role": "source_run_rows", "path": SOURCE_ROWS.relative_to(REPO).as_posix(), "sha256": sha256(SOURCE_ROWS)},
            {"role": "run_preparation_script", "path": Path(__file__).resolve().relative_to(REPO).as_posix(), "sha256": sha256(Path(__file__).resolve())},
        ]
    )
    for path in input_files:
        manifest_rows.append(
            {"role": "input_instance", "path": path.relative_to(REPO).as_posix(), "sha256": sha256(path)}
        )
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    with MANIFEST.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["role", "path", "sha256"])
        writer.writeheader()
        writer.writerows(manifest_rows)

    command = (
        "python -u -m src.cli run-heuristic-study "
        f"--config {RUN_CONFIG.relative_to(REPO).as_posix()} "
        f"--output-dir {OUTPUT.relative_to(REPO).as_posix()}"
    )
    command_file = OUTPUT / "provenance/resume_command.txt"
    command_file.write_text(command + "\n", encoding="utf-8")
    sizes = {}
    for row in historical_rows:
        sizes[row["n"]] = sizes.get(row["n"], 0) + 1
    sizes = {size: count // len(METHODS) for size, count in sizes.items()}
    metadata = {
        "status": "prepared",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "execution_commit": commit,
        "tracked_source_tree_clean": True,
        "base_config": BASE_CONFIG.relative_to(REPO).as_posix(),
        "base_config_sha256": sha256(BASE_CONFIG),
        "effective_run_config": RUN_CONFIG.relative_to(REPO).as_posix(),
        "effective_run_config_sha256": sha256(RUN_CONFIG),
        "source_instances_dir": SOURCE_RAW.relative_to(REPO).as_posix(),
        "source_run_rows": SOURCE_ROWS.relative_to(REPO).as_posix(),
        "input_instances": len(input_files),
        "input_instances_by_size": sizes,
        "input_instance_set_sha256": hashlib.sha256(
            ("\n".join(sorted(instance_ids)) + "\n").encode("utf-8")
        ).hexdigest(),
        "input_set_verified_against_prior_720_instance_results": True,
        "methods": METHODS,
        "method_labels": {
            "paired_baseline": "M3",
            "no_unpairing": "M4",
            "nils": "M5",
            "feasibility_aware_alns": "ALNS",
        },
        "expected_method_runs": 720 * len(METHODS),
        "nils_seed": 2026,
        "alns_seed": 2237,
        "nils_max_outer_iterations": 25,
        "nils_max_no_improve": 8,
        "nils_time_limit_seconds": 120,
        "alns_max_iterations": 40,
        "alns_time_limit_seconds": 120,
        "max_displacement_candidates": 100,
        "max_repair_candidates": 100,
        "shared_initial_backbone_methods": ["paired_baseline", "no_unpairing", "nils"],
        "architecture_validation_enabled_for_nils_methods": True,
        "post_local_search_feasibility_guard_enabled": True,
        "nils_operator_changes": False,
        "alns_changes": False,
        "checkpoint_file": (OUTPUT / "checkpoints/heuristic_study_rows.jsonl").relative_to(REPO).as_posix(),
        "sha256_manifest": MANIFEST.relative_to(REPO).as_posix(),
        "sha256_manifest_sha256": sha256(MANIFEST),
        "resume_command_file": command_file.relative_to(REPO).as_posix(),
        "resume_command_sha256": sha256(command_file),
    }
    PROVENANCE.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "output_dir": OUTPUT.relative_to(REPO).as_posix(),
                "execution_commit": commit,
                "input_instances": len(input_files),
                "method_runs": metadata["expected_method_runs"],
                "size_counts": sizes,
                "source_code_files_hashed": sum(row["role"] == "source_code" for row in manifest_rows),
                "manifest_rows": len(manifest_rows),
                "effective_config_sha256": metadata["effective_run_config_sha256"],
                "resume_command": command,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
