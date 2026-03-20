#!/usr/bin/env python3
"""Create synthetic pre-C1 layout snapshots for a normalized GRScenes dataset.

The helper scans normalized scenes under:
  <normalized_root>/GRScenes100/<scene_type>/<scene_id>/layout.usd

For each discovered layout, it creates a sibling snapshot named:
  layout.pre_c1_normalize_only.<RUN_ID>.usd

It also writes a JSON report summarizing the run.
"""

from __future__ import annotations

import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


def _default_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _abs_path(path: str) -> Path:
    return Path(path).expanduser().resolve()


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _discover_scene_layouts(scenes_root: Path) -> Tuple[List[Dict[str, str]], List[str]]:
    discovered: List[Dict[str, str]] = []
    missing_layouts: List[str] = []

    for scene_type_dir in sorted(path for path in scenes_root.iterdir() if path.is_dir()):
        if scene_type_dir.name.startswith("."):
            continue
        for scene_dir in sorted(path for path in scene_type_dir.iterdir() if path.is_dir()):
            if scene_dir.name.startswith("."):
                continue
            layout_path = scene_dir / "layout.usd"
            scene_id = f"{scene_type_dir.name}/{scene_dir.name}"
            if not layout_path.is_file():
                missing_layouts.append(scene_id)
                continue
            discovered.append(
                {
                    "scene_id": scene_id,
                    "scene_dir": str(scene_dir),
                    "layout_path": str(layout_path),
                }
            )

    return discovered, missing_layouts


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--normalized-root",
        required=True,
        help="Normalized dataset root containing GRScenes100/",
    )
    parser.add_argument(
        "--run-id",
        default=_default_run_id(),
        help="Snapshot/report run id. Default: current UTC timestamp.",
    )
    parser.add_argument(
        "--report-path",
        default=None,
        help=(
            "Output JSON report path. Default: "
            "<normalized-root>/pre_c1_normalize_only_report.<RUN_ID>.json"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Plan the snapshot creation and print the summary without writing files.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    normalized_root = _abs_path(args.normalized_root)
    scenes_root = normalized_root / "GRScenes100"
    if not normalized_root.is_dir():
        raise SystemExit(f"Normalized root directory not found: {normalized_root}")
    if not scenes_root.is_dir():
        raise SystemExit(f"Scenes root directory not found: {scenes_root}")

    layouts, missing_layouts = _discover_scene_layouts(scenes_root)
    if not layouts:
        raise SystemExit(f"No layout.usd files found under: {scenes_root}")

    run_id = args.run_id
    snapshot_name = f"layout.pre_c1_normalize_only.{run_id}.usd"
    report_path = (
        _abs_path(args.report_path)
        if args.report_path
        else normalized_root / f"pre_c1_normalize_only_report.{run_id}.json"
    )

    planned: List[Dict[str, str]] = []
    conflicts: List[Dict[str, str]] = []
    for layout_info in layouts:
        scene_dir = Path(layout_info["scene_dir"])
        snapshot_path = scene_dir / snapshot_name
        item = {
            "scene_id": layout_info["scene_id"],
            "layout_path": layout_info["layout_path"],
            "snapshot_path": str(snapshot_path),
        }
        planned.append(item)
        if snapshot_path.exists():
            conflicts.append(item)

    if conflicts:
        conflict_preview = "\n".join(
            f"  - {item['scene_id']}: {item['snapshot_path']}" for item in conflicts[:20]
        )
        raise SystemExit(
            "Refusing to overwrite existing snapshot target(s):\n"
            f"{conflict_preview}\n"
            f"Total conflicts: {len(conflicts)}"
        )

    if not args.dry_run:
        for item in planned:
            shutil.copy2(item["layout_path"], item["snapshot_path"])

    report = {
        "status": "dry_run" if args.dry_run else "ok",
        "run_id": run_id,
        "normalized_root": str(normalized_root),
        "scenes_root": str(scenes_root),
        "report_path": str(report_path),
        "snapshot_name": snapshot_name,
        "scene_count": len(layouts),
        "created_count": 0 if args.dry_run else len(planned),
        "planned_count": len(planned),
        "missing_layout_count": len(missing_layouts),
        "missing_layout_scenes": missing_layouts,
        "snapshots": planned,
    }

    if not args.dry_run:
        _write_json(report_path, report)

    print(f"normalized_root: {normalized_root}")
    print(f"scenes_root: {scenes_root}")
    print(f"run_id: {run_id}")
    print(f"snapshot_name: {snapshot_name}")
    print(f"scene_count: {len(layouts)}")
    print(f"missing_layout_count: {len(missing_layouts)}")
    if args.dry_run:
        print("status: dry_run")
        print(f"report_path: {report_path} (not written)")
        print("snapshots_created: 0")
    else:
        print("status: ok")
        print(f"report_path: {report_path}")
        print(f"snapshots_created: {len(planned)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
