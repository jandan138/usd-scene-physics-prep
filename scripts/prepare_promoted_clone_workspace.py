#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import subprocess
from pathlib import Path


POLICY_TAG = "bbox_primary_rmse_observe"
OUT_VERSION = "v1"
REQUIRED_CERT_BUNDLE_FILES = (
    "filtered_mapping.json",
    "filtered_mapping.stats.json",
    "pair_certificates.jsonl",
    "certified_graph.json",
)
COPY_STARTED_MARKER = ".dataset_copy_started"
COPY_COMPLETE_MARKER = ".dataset_copy_complete"


def _category_name_from_root_name(
    root_name: str, *, policy_tag: str, out_version: str
) -> str:
    suffix = f"_{policy_tag}_{out_version}"
    if not root_name.endswith(suffix):
        raise ValueError(f"unexpected category root: {root_name}")
    return root_name[: -len(suffix)]


def _iter_seedable_cert_dirs(
    source_c1_bulk_root: Path, *, policy_tag: str, out_version: str
):
    seedable = []
    for category_root in sorted(
        source_c1_bulk_root.glob(f"*_{policy_tag}_{out_version}")
    ):
        cert_dir = category_root / "01_cert"
        if not cert_dir.exists():
            continue
        missing_files = [
            name
            for name in REQUIRED_CERT_BUNDLE_FILES
            if not (cert_dir / name).exists()
        ]
        if missing_files:
            missing_display = ", ".join(missing_files)
            raise ValueError(
                f"incomplete cert bundle for {category_root.name}: missing {missing_display}"
            )
        category = _category_name_from_root_name(
            category_root.name,
            policy_tag=policy_tag,
            out_version=out_version,
        )
        seedable.append((category, cert_dir))
    return seedable


def _copytree(src: Path, dst: Path) -> None:
    shutil.copytree(src, dst, symlinks=True)


def _is_recognized_resumable_workspace(workspace_root: Path) -> bool:
    allowed_root_entries = {"dataset", COPY_STARTED_MARKER}
    actual_root_entries = {path.name for path in workspace_root.iterdir()}
    return (
        workspace_root.is_dir()
        and (workspace_root / COPY_STARTED_MARKER).is_file()
        and (workspace_root / "dataset").is_dir()
        and not (workspace_root / COPY_COMPLETE_MARKER).exists()
        and actual_root_entries == allowed_root_entries
    )


def _prepare_workspace_root_for_copy(workspace_root: Path) -> None:
    if workspace_root.exists():
        if not _is_recognized_resumable_workspace(workspace_root):
            raise FileExistsError(f"workspace already exists: {workspace_root}")
    else:
        workspace_root.mkdir(parents=True, exist_ok=False)


def _copy_dataset_resumable(source_dataset_root: Path, dataset_dst: Path) -> None:
    workspace_root = dataset_dst.parent
    (workspace_root / COPY_STARTED_MARKER).write_text("started\n", encoding="utf-8")
    dataset_dst.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "rsync",
            "-a",
            "--partial",
            str(source_dataset_root) + "/",
            str(dataset_dst) + "/",
        ],
        check=True,
    )
    (workspace_root / COPY_COMPLETE_MARKER).write_text("complete\n", encoding="utf-8")


def _build_run_command(
    *,
    workspace_root: Path,
    group_label: str,
    source_report: Path,
    mode_reports_dir: Path,
    policy_tag: str,
    out_version: str,
) -> str:
    dataset_root = workspace_root / "dataset"
    bak_root = workspace_root / "bak"
    c1_bulk_root = workspace_root / "c1_bulk"
    repo_root = Path(__file__).resolve().parents[1]
    cmd = [
        str(repo_root / "scripts" / "isaac_python.sh"),
        str(repo_root / "scripts" / "c1_autorun_categories.py"),
        "--dataset-root",
        str(dataset_root),
        "--bak-root",
        str(bak_root),
        "--report",
        str(source_report),
        "--c1-bulk-dir",
        str(c1_bulk_root),
        "--group-label",
        group_label,
        "--bbox-gated",
        "--bbox-policy",
        policy_tag,
        "--step6-mode",
        "apply",
        "--dedup-mode",
        "geom_only",
        "--v-matrix-mode",
        "auto",
        "--mode-reports-dir",
        str(mode_reports_dir),
        "--scene-files",
        "layout.usd",
    ]
    if out_version != OUT_VERSION:
        cmd.extend(["--out-version", out_version])
    return " ".join(shlex.quote(part) for part in cmd)


def _validate_run_inputs(source_report: Path, mode_reports_dir: Path) -> None:
    if not source_report.is_file():
        raise FileNotFoundError(f"missing source report: {source_report}")
    if not mode_reports_dir.is_dir():
        raise FileNotFoundError(f"missing mode reports dir: {mode_reports_dir}")


def prepare_promoted_clone_workspace(
    *,
    source_dataset_root: Path,
    source_c1_bulk_root: Path,
    workspace_root: Path,
    group_label: str,
    source_report: Path | None = None,
    mode_reports_dir: Path | None = None,
    policy_tag: str = POLICY_TAG,
    out_version: str = OUT_VERSION,
):
    source_dataset_root = Path(source_dataset_root).resolve()
    source_c1_bulk_root = Path(source_c1_bulk_root).resolve()
    workspace_root = Path(workspace_root).resolve()
    source_report = (
        Path(source_report).resolve()
        if source_report
        else source_c1_bulk_root.parent
        / "v8_prededup"
        / "union_3way"
        / "all_categories_union_merged.json"
    )
    mode_reports_dir = (
        Path(mode_reports_dir).resolve()
        if mode_reports_dir
        else source_c1_bulk_root.parent / "v8_prededup"
    )

    if not source_dataset_root.is_dir():
        raise FileNotFoundError(f"missing source dataset root: {source_dataset_root}")
    if not source_c1_bulk_root.is_dir():
        raise FileNotFoundError(f"missing source c1 bulk root: {source_c1_bulk_root}")
    _validate_run_inputs(source_report, mode_reports_dir)

    seedable_cert_dirs = _iter_seedable_cert_dirs(
        source_c1_bulk_root,
        policy_tag=policy_tag,
        out_version=out_version,
    )

    dataset_dst = workspace_root / "dataset"
    bak_dst = workspace_root / "bak"
    c1_bulk_dst = workspace_root / "c1_bulk"
    notes_dst = workspace_root / "notes"

    _prepare_workspace_root_for_copy(workspace_root)
    _copy_dataset_resumable(source_dataset_root, dataset_dst)
    bak_dst.mkdir(parents=True, exist_ok=False)
    c1_bulk_dst.mkdir(parents=True, exist_ok=False)
    notes_dst.mkdir(parents=True, exist_ok=False)

    seeded_categories = []
    for category, cert_dir in seedable_cert_dirs:
        dst_cert_dir = (
            c1_bulk_dst / f"{category}_{policy_tag}_{out_version}" / "01_cert"
        )
        dst_cert_dir.parent.mkdir(parents=True, exist_ok=True)
        _copytree(cert_dir, dst_cert_dir)
        seeded_categories.append(category)

    run_command = _build_run_command(
        workspace_root=workspace_root,
        group_label=group_label,
        source_report=source_report,
        mode_reports_dir=mode_reports_dir,
        policy_tag=policy_tag,
        out_version=out_version,
    )
    run_script_path = workspace_root / "run_promoted_clone.sh"
    run_script_path.write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n" + run_command + "\n",
        encoding="utf-8",
    )
    run_script_path.chmod(0o755)

    manifest = {
        "source_dataset_root": str(source_dataset_root),
        "source_c1_bulk_root": str(source_c1_bulk_root),
        "workspace_root": str(workspace_root),
        "group_label": group_label,
        "policy_tag": policy_tag,
        "out_version": out_version,
        "source_report": str(source_report),
        "mode_reports_dir": str(mode_reports_dir),
        "seeded_categories": seeded_categories,
        "seeded_category_count": len(seeded_categories),
        "run_script_path": str(run_script_path),
    }
    (workspace_root / "workspace_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return {**manifest, "run_command": run_command}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dataset-root", required=True)
    parser.add_argument("--source-c1-bulk-root", required=True)
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--group-label", required=True)
    parser.add_argument("--source-report", default=None)
    parser.add_argument("--mode-reports-dir", default=None)
    parser.add_argument("--policy-tag", default=POLICY_TAG)
    parser.add_argument("--out-version", default=OUT_VERSION)
    args = parser.parse_args()

    result = prepare_promoted_clone_workspace(
        source_dataset_root=Path(args.source_dataset_root),
        source_c1_bulk_root=Path(args.source_c1_bulk_root),
        workspace_root=Path(args.workspace_root),
        group_label=args.group_label,
        source_report=Path(args.source_report) if args.source_report else None,
        mode_reports_dir=Path(args.mode_reports_dir) if args.mode_reports_dir else None,
        policy_tag=args.policy_tag,
        out_version=args.out_version,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
