#!/usr/bin/env python3
"""Orchestrate category-by-category C1 asset dedup runs.

This script is intentionally *not* a pxr script. It shells out to the existing
Isaac/kit wrapper + repo scripts, and provides:
- auto-discovery of categories under <dataset>/GRScenes_assets
- optional skipping of already-completed categories
- per-category mapping build (if missing) from the big geom-only report
- bulk apply (layout + start_result_*) then Step6 promote/scan/soft-delete
- stop-on-failure semantics (scan hits, non-zero exit codes)

Typical usage:
  python3 scripts/c1_autorun_categories.py \
    --dataset-root GRScenes-test1 \
    --bak-root GRScenes-test1_bak \
    --report check_reports/test1_asset_mesh_dedup_geom_only.json \
    --c1-bulk-dir check_reports/c1_bulk

Defaults are tuned for safety (skip door_* variants, skip already-done, stop on errors).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
ISAAC_PY = REPO_ROOT / "scripts" / "isaac_python.sh"
SCRIPT_BUILD_MAPPING = (
    REPO_ROOT / "scripts" / "c1_build_bulk_mapping_from_dedup_report.py"
)
SCRIPT_BULK_APPLY = REPO_ROOT / "scripts" / "c1_bulk_apply_layout_dedup.py"
SCRIPT_STEP6 = (
    REPO_ROOT / "scripts" / "c1_bulk_step6_category_promote_scan_soft_delete.py"
)
SCRIPT_AUDIT = REPO_ROOT / "scripts" / "placement_pairwise_compare.py"


@dataclass(frozen=True)
class CategoryPlan:
    category: str
    mapping_json: Path
    mapping_stats_json: Path
    out_name: str
    batch_report_dir: Path
    step6_report_dir: Path


@dataclass(frozen=True)
class BBoxCategoryPlan:
    category: str
    category_root: Path
    cert_dir: Path
    apply_dir: Path
    audit_dir: Path
    step6_dir: Path
    mapping_json: Path
    mapping_stats_json: Path
    certificate_jsonl: Path
    certificate_summary_json: Path
    certified_graph_json: Path
    reject_ledger_jsonl: Path
    audit_report_json: Path
    audit_verdict_json: Path
    out_name: str


def _run(cmd: list[str], *, cwd: Path, log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = None
    log_failed = False

    def _disable_log(exc: OSError) -> None:
        nonlocal log_file, log_failed
        if log_failed:
            return
        log_failed = True
        sys.stderr.write(f"[bbox-autorun] log write disabled for {log_path}: {exc}\n")
        if log_file is not None:
            try:
                log_file.close()
            except OSError:
                pass
            log_file = None

    def _write_log(text: str, *, flush: bool = False) -> None:
        if log_file is None or log_failed:
            return
        try:
            log_file.write(text)
            if flush:
                log_file.flush()
        except OSError as exc:
            _disable_log(exc)

    try:
        log_file = log_path.open("w", encoding="utf-8", buffering=1)
    except OSError as exc:
        _disable_log(exc)

    _write_log("# cmd\n")
    _write_log(" ".join(cmd) + "\n\n", flush=True)

    env = os.environ.copy()
    env.setdefault("TF_LOG", "0")
    env.setdefault("TF_WARN_OUTPUT_FILE", str(log_path))
    proc = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        sys.stdout.write(line)
        _write_log(line)
    rc = proc.wait()
    if log_file is not None:
        try:
            log_file.close()
        except OSError as exc:
            _disable_log(exc)
    return rc


def _list_categories(dataset_root: Path) -> list[str]:
    assets_dir = dataset_root / "GRScenes_assets"
    if not assets_dir.is_dir():
        raise FileNotFoundError(f"Missing assets dir: {assets_dir}")
    return sorted([p.name for p in assets_dir.iterdir() if p.is_dir()])


def _is_step6_complete(step6_dir: Path) -> bool:
    """Returns True only if step6 finished fully and cleanly.

    This is intentionally stricter than "dir exists" so the autorun can safely
    resume after machine interruptions without skipping partially-completed work.
    """

    scan_full = step6_dir / "post_promote_full_usd_scan_excluding_backups_pxr.json"
    scan_layout = step6_dir / "post_soft_delete_layout_scan_pxr.json"
    if not scan_full.exists() or not scan_layout.exists():
        return False

    try:
        full = json.loads(scan_full.read_text(encoding="utf-8"))
        layout = json.loads(scan_layout.read_text(encoding="utf-8"))
    except Exception:
        return False

    hit_files = full.get("hit_files")
    hit_layouts = layout.get("hit_layouts")
    if not isinstance(hit_files, int) or not isinstance(hit_layouts, int):
        return False
    return hit_files == 0 and hit_layouts == 0


# Public aliases for external consumers (e.g., orchestrator)
is_step6_complete = _is_step6_complete


def _iter_step6_dirs(category: str, c1_bulk_dir: Path) -> list[Path]:
    prefix = f"{category}_bulk_step6"
    dirs: list[Path] = []
    for p in c1_bulk_dir.iterdir():
        if p.is_dir() and p.name.startswith(prefix):
            dirs.append(p)

    def _version_key(p: Path) -> tuple[int, str]:
        m = re.search(r"_v(\d+)$", p.name)
        if m:
            return (int(m.group(1)), p.name)
        # Unversioned dirs sort before versioned ones.
        return (0, p.name)

    return sorted(dirs, key=_version_key)


def _is_done(category: str, c1_bulk_dir: Path) -> bool:
    for step6_dir in _iter_step6_dirs(category, c1_bulk_dir):
        if _is_step6_complete(step6_dir):
            return True
    return False


def _next_step6_dir(
    category: str, c1_bulk_dir: Path, *, prefer_v1: bool = True
) -> Path:
    # If there is an incomplete step6 dir, resume into it.
    step6_dirs = _iter_step6_dirs(category, c1_bulk_dir)
    incomplete = [p for p in step6_dirs if not _is_step6_complete(p)]
    if incomplete:
        return incomplete[-1]

    # Otherwise, pick a new dir name.
    if not step6_dirs:
        return c1_bulk_dir / (
            f"{category}_bulk_step6_v1" if prefer_v1 else f"{category}_bulk_step6"
        )

    v1 = c1_bulk_dir / f"{category}_bulk_step6_v1"
    if prefer_v1 and not v1.exists():
        return v1

    # Find next available vN after the max existing version.
    max_v = 1
    for p in step6_dirs:
        m = re.search(r"_v(\d+)$", p.name)
        if m:
            max_v = max(max_v, int(m.group(1)))
    for i in range(max_v + 1, 1000):
        cand = c1_bulk_dir / f"{category}_bulk_step6_v{i}"
        if not cand.exists():
            return cand
    raise RuntimeError(f"Too many existing step6 dirs for category={category}")


def _read_mapping_pairs(stats_path: Path) -> Optional[int]:
    if not stats_path.exists():
        return None
    try:
        with stats_path.open("r", encoding="utf-8") as f:
            d = json.load(f)
        # stats file format can evolve; try common keys
        for key in ("mapping_pairs", "pairs", "mapping_pair_count"):
            if key in d and isinstance(d[key], int):
                return d[key]
    except Exception:
        return None
    return None


def _write_ledger(ledger_path: Path, event: dict) -> None:
    ledger_path.parent.mkdir(parents=True, exist_ok=True)
    event = dict(event)
    event.setdefault("ts", time.strftime("%Y-%m-%d %H:%M:%S"))
    with ledger_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(event, ensure_ascii=False) + "\n")


def _changed_scene_ids_from_batch_summary(
    batch_summary_path: Path, dataset_root: Path
) -> list[str]:
    if not batch_summary_path.exists():
        return []
    try:
        payload = json.loads(batch_summary_path.read_text(encoding="utf-8"))
    except Exception:
        return []

    changed: set[str] = set()
    base = dataset_root / "GRScenes100"
    for item in payload.get("per_layout", []):
        if item.get("scene_file") != "layout.usd":
            continue
        counts = item.get("counts") or {}
        if not any(
            int(counts.get(k, 0))
            for k in ("refs_changed", "payloads_changed", "asset_attrs_changed")
        ):
            continue
        layout_in = item.get("layout_in")
        if not isinstance(layout_in, str):
            continue
        try:
            rel = Path(layout_in).resolve().relative_to(base.resolve())
        except Exception:
            continue
        if len(rel.parts) >= 3:
            changed.add(f"{rel.parts[0]}/{rel.parts[1]}")
    return sorted(changed)


def _load_category_allowlist(path: Optional[str]) -> Optional[set[str]]:
    if not path:
        return None
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return {str(item) for item in payload if isinstance(item, str)}


def build_plan(
    category: str,
    *,
    c1_bulk_dir: Path,
    group_label: str,
    out_version: str,
    step6_dir: Path,
) -> CategoryPlan:
    mapping_json = c1_bulk_dir / f"{category}_geom_only_mapping.json"
    mapping_stats_json = c1_bulk_dir / f"{category}_geom_only_mapping.stats.json"
    out_name = f"layout.{group_label}_dedup_{out_version}.usd"
    batch_report_dir = c1_bulk_dir / f"{category}_bulk_batch_{out_version}"
    step6_report_dir = step6_dir
    return CategoryPlan(
        category=category,
        mapping_json=mapping_json,
        mapping_stats_json=mapping_stats_json,
        out_name=out_name,
        batch_report_dir=batch_report_dir,
        step6_report_dir=step6_report_dir,
    )


def _is_bbox_done(category_root: Path, *, step6_mode: str = "off") -> bool:
    verdict = category_root / "03_audit" / "audit_verdict.json"
    if not verdict.exists():
        return False
    try:
        payload = json.loads(verdict.read_text(encoding="utf-8"))
    except Exception:
        return False
    if not bool(payload.get("passed")):
        return False
    if step6_mode == "apply":
        return _is_step6_complete(category_root / "04_step6")
    return True


def build_bbox_plan(
    category: str,
    *,
    c1_bulk_dir: Path,
    policy_tag: str,
    out_version: str,
    group_label: str,
) -> BBoxCategoryPlan:
    category_root = c1_bulk_dir / f"{category}_{policy_tag}_{out_version}"
    cert_dir = category_root / "01_cert"
    apply_dir = category_root / "02_apply"
    audit_dir = category_root / "03_audit"
    step6_dir = category_root / "04_step6"
    out_name = f"layout.{group_label}_{policy_tag}_{out_version}.usd"
    return BBoxCategoryPlan(
        category=category,
        category_root=category_root,
        cert_dir=cert_dir,
        apply_dir=apply_dir,
        audit_dir=audit_dir,
        step6_dir=step6_dir,
        mapping_json=cert_dir / "filtered_mapping.json",
        mapping_stats_json=cert_dir / "filtered_mapping.stats.json",
        certificate_jsonl=cert_dir / "pair_certificates.jsonl",
        certificate_summary_json=cert_dir / "pair_certificate_summary.json",
        certified_graph_json=cert_dir / "certified_graph.json",
        reject_ledger_jsonl=apply_dir / "rewrite_reject_ledger.jsonl",
        audit_report_json=audit_dir / "placement_pairwise_compare.json",
        audit_verdict_json=audit_dir / "audit_verdict.json",
        out_name=out_name,
    )


def _build_bbox_audit_cmd(
    *,
    dataset_root: Path,
    plan: BBoxCategoryPlan,
    changed_scene_list_json: Path,
    args: argparse.Namespace,
) -> list[str]:
    cmd = [
        str(ISAAC_PY),
        str(SCRIPT_AUDIT.relative_to(REPO_ROOT)),
        "--left-root",
        str(dataset_root.relative_to(REPO_ROOT))
        if dataset_root.is_relative_to(REPO_ROOT)
        else str(dataset_root),
        "--right-root",
        str(dataset_root.relative_to(REPO_ROOT))
        if dataset_root.is_relative_to(REPO_ROOT)
        else str(dataset_root),
        "--left-mode",
        "current",
        "--right-mode",
        "current",
        "--right-layout-name",
        plan.out_name,
        "--label",
        f"{plan.category}_{args.bbox_policy}",
        "--out",
        str(plan.audit_report_json.relative_to(REPO_ROOT))
        if plan.audit_report_json.is_relative_to(REPO_ROOT)
        else str(plan.audit_report_json),
        "--verdict-out",
        str(plan.audit_verdict_json.relative_to(REPO_ROOT))
        if plan.audit_verdict_json.is_relative_to(REPO_ROOT)
        else str(plan.audit_verdict_json),
        "--scene-list-json",
        str(changed_scene_list_json.relative_to(REPO_ROOT))
        if changed_scene_list_json.is_relative_to(REPO_ROOT)
        else str(changed_scene_list_json),
        "--certificate-jsonl",
        str(plan.certificate_jsonl.relative_to(REPO_ROOT))
        if plan.certificate_jsonl.is_relative_to(REPO_ROOT)
        else str(plan.certificate_jsonl),
        "--bbox-policy",
        args.bbox_policy,
        "--eps-bbox",
        str(args.eps_bbox),
        "--eps-pos",
        str(args.eps_pos),
        "--eps-angle",
        str(args.eps_angle),
        "--eps-geom",
        str(args.eps_geom),
        "--allow-no-mesh",
    ]
    if args.mode_reports_dir:
        cmd.extend(["--mode-reports-dir", args.mode_reports_dir])
    return cmd


def _run_bbox_gated(args: argparse.Namespace) -> int:
    dataset_root = Path(args.dataset_root).resolve()
    bak_root = Path(args.bak_root).resolve()
    baseline_root = Path(args.baseline_root).resolve() if args.baseline_root else None
    report_path = Path(args.report).resolve()
    c1_bulk_dir = Path(args.c1_bulk_dir).resolve()
    c1_bulk_dir.mkdir(parents=True, exist_ok=True)

    include_re = re.compile(args.include_regex) if args.include_regex else None
    exclude_re = re.compile(args.exclude_regex) if args.exclude_regex else None
    category_allowlist = _load_category_allowlist(args.category_list_json)

    categories = _list_categories(dataset_root)
    if category_allowlist is not None:
        categories = [c for c in categories if c in category_allowlist]
    if args.skip_door_variants:
        categories = [c for c in categories if not c.startswith("door_")]
    if include_re:
        categories = [c for c in categories if include_re.search(c)]
    if exclude_re:
        categories = [c for c in categories if not exclude_re.search(c)]
    if args.skip_done:
        categories = [
            c
            for c in categories
            if not _is_bbox_done(
                c1_bulk_dir / f"{c}_{args.bbox_policy}_{args.out_version}",
                step6_mode=args.step6_mode,
            )
        ]
    if args.max_categories and args.max_categories > 0:
        categories = categories[: args.max_categories]

    run_stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = (
        c1_bulk_dir / "_autorun" / f"{args.group_label}_{args.bbox_policy}_{run_stamp}"
    )
    ledger_path = run_dir / "ledger.jsonl"
    _write_ledger(
        ledger_path,
        {
            "event": "run_start",
            "bbox_gated": True,
            "bbox_policy": args.bbox_policy,
            "baseline_root": str(baseline_root) if baseline_root else None,
            "dataset_root": str(dataset_root),
            "bak_root": str(bak_root),
            "report": str(report_path),
            "c1_bulk_dir": str(c1_bulk_dir),
            "step6_mode": args.step6_mode,
            "categories": categories,
        },
    )

    print(f"[bbox-autorun] categories_to_run={len(categories)} run_dir={run_dir}")

    failed_categories: list[str] = []
    for idx, category in enumerate(categories, start=1):
        group_label = (
            args.group_label
            if args.group_label != "c1_autorun"
            else f"c1_{category}_bbox"
        )
        plan = build_bbox_plan(
            category,
            c1_bulk_dir=c1_bulk_dir,
            policy_tag=args.bbox_policy,
            out_version=args.out_version,
            group_label=group_label,
        )
        _write_ledger(
            ledger_path,
            {
                "event": "category_start",
                "category": category,
                "group_label": group_label,
            },
        )
        print(
            f"\n[bbox-autorun] ({idx}/{len(categories)}) category={category} policy={args.bbox_policy}"
        )

        # 1) Certificates + filtered mapping
        if not plan.mapping_json.exists():
            cmd = [
                str(ISAAC_PY),
                str(SCRIPT_BUILD_MAPPING.relative_to(REPO_ROOT)),
                "--report",
                str(report_path.relative_to(REPO_ROOT))
                if report_path.is_relative_to(REPO_ROOT)
                else str(report_path),
                "--dataset-root",
                str(dataset_root.relative_to(REPO_ROOT))
                if dataset_root.is_relative_to(REPO_ROOT)
                else str(dataset_root),
                "--category",
                category,
                "--out-mapping-json",
                str(plan.mapping_json.relative_to(REPO_ROOT))
                if plan.mapping_json.is_relative_to(REPO_ROOT)
                else str(plan.mapping_json),
                "--out-stats-json",
                str(plan.mapping_stats_json.relative_to(REPO_ROOT))
                if plan.mapping_stats_json.is_relative_to(REPO_ROOT)
                else str(plan.mapping_stats_json),
                "--bbox-gated",
                "--bbox-policy",
                args.bbox_policy,
                "--dedup-mode",
                args.dedup_mode,
                "--out-certificate-jsonl",
                str(plan.certificate_jsonl.relative_to(REPO_ROOT))
                if plan.certificate_jsonl.is_relative_to(REPO_ROOT)
                else str(plan.certificate_jsonl),
                "--out-certificate-summary-json",
                str(plan.certificate_summary_json.relative_to(REPO_ROOT))
                if plan.certificate_summary_json.is_relative_to(REPO_ROOT)
                else str(plan.certificate_summary_json),
                "--out-certified-graph-json",
                str(plan.certified_graph_json.relative_to(REPO_ROOT))
                if plan.certified_graph_json.is_relative_to(REPO_ROOT)
                else str(plan.certified_graph_json),
            ]
            if args.mode_reports_dir:
                cmd.extend(["--mode-reports-dir", args.mode_reports_dir])
            rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "01_cert.log")
            if rc != 0:
                _write_ledger(
                    ledger_path,
                    {
                        "event": "category_fail",
                        "category": category,
                        "step": "cert",
                        "rc": rc,
                    },
                )
                if args.continue_on_failure:
                    failed_categories.append(category)
                    print(
                        f"[bbox-autorun] category={category} cert FAILED (rc={rc}), continuing"
                    )
                    continue
                return rc

        pairs = _read_mapping_pairs(plan.mapping_stats_json)
        if pairs is not None and pairs == 0:
            print(f"[bbox-autorun] category={category} mapping_pairs=0 -> skip")
            _write_ledger(
                ledger_path,
                {
                    "event": "category_skip",
                    "category": category,
                    "reason": "mapping_pairs_0",
                },
            )
            continue

        # 2) Bulk apply
        cmd = [
            str(ISAAC_PY),
            str(SCRIPT_BULK_APPLY.relative_to(REPO_ROOT)),
            "--dataset-root",
            str(dataset_root.relative_to(REPO_ROOT))
            if dataset_root.is_relative_to(REPO_ROOT)
            else str(dataset_root),
            "--mapping-json",
            str(plan.mapping_json.relative_to(REPO_ROOT))
            if plan.mapping_json.is_relative_to(REPO_ROOT)
            else str(plan.mapping_json),
            "--mapping-stats-json",
            str(plan.mapping_stats_json.relative_to(REPO_ROOT))
            if plan.mapping_stats_json.is_relative_to(REPO_ROOT)
            else str(plan.mapping_stats_json),
            "--certificate-jsonl",
            str(plan.certificate_jsonl.relative_to(REPO_ROOT))
            if plan.certificate_jsonl.is_relative_to(REPO_ROOT)
            else str(plan.certificate_jsonl),
            "--group-label",
            group_label,
            "--out-name",
            plan.out_name,
            "--scene-files",
            args.scene_files,
            "--report-dir",
            str(plan.apply_dir.relative_to(REPO_ROOT))
            if plan.apply_dir.is_relative_to(REPO_ROOT)
            else str(plan.apply_dir),
            "--bbox-gated",
            "--bbox-policy",
            args.bbox_policy,
            "--reject-ledger-jsonl",
            str(plan.reject_ledger_jsonl.relative_to(REPO_ROOT))
            if plan.reject_ledger_jsonl.is_relative_to(REPO_ROOT)
            else str(plan.reject_ledger_jsonl),
            "--v-matrix-mode",
            args.v_matrix_mode,
        ]
        if baseline_root:
            cmd.extend(["--baseline-root", str(baseline_root)])
        if args.mode_reports_dir:
            cmd.extend(["--mode-reports-dir", args.mode_reports_dir])
        if args.input_layout_name:
            cmd.extend(["--input-layout-name", args.input_layout_name])
        if args.set_instanceable:
            cmd.append("--set-instanceable")
        rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "02_apply.log")
        if rc != 0:
            _write_ledger(
                ledger_path,
                {
                    "event": "category_fail",
                    "category": category,
                    "step": "apply",
                    "rc": rc,
                },
            )
            if args.continue_on_failure:
                failed_categories.append(category)
                print(
                    f"[bbox-autorun] category={category} apply FAILED (rc={rc}), continuing"
                )
                continue
            return rc

        changed_scene_ids = _changed_scene_ids_from_batch_summary(
            plan.apply_dir / "batch_summary.json", dataset_root
        )
        changed_scene_list_json = plan.apply_dir / "changed_scene_ids.json"
        changed_scene_list_json.parent.mkdir(parents=True, exist_ok=True)
        changed_scene_list_json.write_text(
            json.dumps(changed_scene_ids, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        # 3) Authoritative audit
        if changed_scene_ids:
            cmd = _build_bbox_audit_cmd(
                dataset_root=dataset_root,
                plan=plan,
                changed_scene_list_json=changed_scene_list_json,
                args=args,
            )
            rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "03_audit.log")
            if rc != 0:
                _write_ledger(
                    ledger_path,
                    {
                        "event": "category_fail",
                        "category": category,
                        "step": "audit",
                        "rc": rc,
                    },
                )
                if args.continue_on_failure:
                    failed_categories.append(category)
                    print(
                        f"[bbox-autorun] category={category} audit FAILED (rc={rc}), continuing"
                    )
                    continue
                return rc
        else:
            empty_aggregate = {
                "scenes_ok": 0,
                "scenes_error": 0,
                "total_common_prims": 0,
                "total_compared": 0,
                "total_no_mesh": 0,
                "total_only_in_left": 0,
                "total_only_in_right": 0,
                "total_ref_changed": 0,
                "total_ref_same": 0,
                "ref_changed_hard_fail_count": 0,
                "displaced_breakdown": {
                    "gt_0.01": 0,
                    "gt_0.1": 0,
                    "gt_1.0": 0,
                    "gt_10.0": 0,
                },
                "max_per_mesh_breakdown": {
                    "gt_0.01": 0,
                    "gt_0.1": 0,
                    "gt_1.0": 0,
                    "gt_10.0": 0,
                },
                "vertex_rmse_breakdown": {
                    "gt_0.01": 0,
                    "gt_0.1": 0,
                    "gt_1.0": 0,
                    "gt_10.0": 0,
                },
                "total_vertex_rmse_count": 0,
                "ref_changed_breakdown": {
                    "gt_0.01": 0,
                    "gt_0.1": 0,
                    "gt_1.0": 0,
                    "gt_10.0": 0,
                },
                "ref_same_breakdown": {
                    "gt_0.01": 0,
                    "gt_0.1": 0,
                    "gt_1.0": 0,
                    "gt_10.0": 0,
                },
                "blocking_reason_counts": {},
                "category_maxima": {},
                "global_top_50_worst": [],
                "scene_errors": [],
            }
            plan.audit_dir.mkdir(parents=True, exist_ok=True)
            plan.audit_report_json.write_text(
                json.dumps(
                    {
                        "label": f"{category}_{args.bbox_policy}",
                        "bbox_policy": args.bbox_policy,
                        "changed_scene_ids": changed_scene_ids,
                        "aggregate": empty_aggregate,
                        "per_scene": [],
                        "skipped_reason": "no_changed_layouts",
                    },
                    indent=2,
                    ensure_ascii=False,
                )
                + "\n",
                encoding="utf-8",
            )
            plan.audit_verdict_json.write_text(
                json.dumps(
                    {
                        "label": f"{category}_{args.bbox_policy}",
                        "policy": args.bbox_policy,
                        "thresholds": {
                            "eps_bbox": args.eps_bbox,
                            "eps_pos": args.eps_pos,
                            "eps_angle": args.eps_angle,
                            "eps_geom": args.eps_geom,
                        },
                        "passed": True,
                        "compared_scope_complete": True,
                        "scenes_error": 0,
                        "total_no_mesh": 0,
                        "ref_changed_fail_count": 0,
                        "blocking_reason_counts": {},
                        "skipped_reason": "no_changed_layouts",
                    },
                    indent=2,
                    ensure_ascii=False,
                )
                + "\n",
                encoding="utf-8",
            )

        # 4) Step6 gate / dry-run / apply
        if args.step6_mode != "off":
            cmd = [
                str(ISAAC_PY),
                str(SCRIPT_STEP6.relative_to(REPO_ROOT)),
                "--dataset-root",
                str(dataset_root.relative_to(REPO_ROOT))
                if dataset_root.is_relative_to(REPO_ROOT)
                else str(dataset_root),
                "--bak-root",
                str(bak_root.relative_to(REPO_ROOT))
                if bak_root.is_relative_to(REPO_ROOT)
                else str(bak_root),
                "--mapping-json",
                str(plan.mapping_json.relative_to(REPO_ROOT))
                if plan.mapping_json.is_relative_to(REPO_ROOT)
                else str(plan.mapping_json),
                "--category",
                category,
                "--group-label",
                group_label,
                "--out-name",
                plan.out_name,
                "--promote-scene-files",
                args.scene_files,
                "--report-dir",
                str(plan.step6_dir.relative_to(REPO_ROOT))
                if plan.step6_dir.is_relative_to(REPO_ROOT)
                else str(plan.step6_dir),
                "--audit-verdict-json",
                str(plan.audit_verdict_json.relative_to(REPO_ROOT))
                if plan.audit_verdict_json.is_relative_to(REPO_ROOT)
                else str(plan.audit_verdict_json),
                "--bbox-gated",
            ]
            if args.step6_mode == "dry_run":
                cmd.append("--dry-run")
            rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "04_step6.log")
            if rc != 0:
                _write_ledger(
                    ledger_path,
                    {
                        "event": "category_fail",
                        "category": category,
                        "step": "step6",
                        "rc": rc,
                    },
                )
                if args.continue_on_failure:
                    failed_categories.append(category)
                    print(
                        f"[bbox-autorun] category={category} step6 FAILED (rc={rc}), continuing"
                    )
                    continue
                return rc

        _write_ledger(
            ledger_path,
            {
                "event": "category_done",
                "category": category,
                "group_label": group_label,
                "mapping_json": str(plan.mapping_json),
                "audit_verdict_json": str(plan.audit_verdict_json),
            },
        )

    _write_ledger(
        ledger_path, {"event": "run_done", "failed_categories": failed_categories}
    )
    if failed_categories:
        print(
            f"[bbox-autorun] DONE with {len(failed_categories)} failed categories: {failed_categories}"
        )
        return 1
    print("[bbox-autorun] DONE")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--dataset-root", required=True, help="Dataset root dir (e.g. GRScenes-test1)"
    )
    ap.add_argument(
        "--bak-root", required=True, help="Backup root dir (e.g. GRScenes-test1_bak)"
    )
    ap.add_argument("--report", required=True, help="Big geom-only dedup report JSON")
    ap.add_argument(
        "--c1-bulk-dir",
        default="check_reports/c1_bulk",
        help="Autorun workspace root for mappings, batch reports, step6 dirs, and ledger",
    )

    ap.add_argument(
        "--group-label",
        default="c1_autorun",
        help="Group label used for backups and out-name",
    )
    ap.add_argument(
        "--out-version", default="v1", help="Out version tag used in file/dir naming"
    )

    ap.add_argument(
        "--max-categories", type=int, default=0, help="Max categories to run (0=all)"
    )
    ap.add_argument(
        "--sleep-seconds", type=float, default=0.0, help="Sleep between categories"
    )

    ap.add_argument("--skip-done", action="store_true", default=True)
    ap.add_argument("--no-skip-done", dest="skip_done", action="store_false")

    ap.add_argument(
        "--skip-door-variants",
        action="store_true",
        default=True,
        help="Skip directories like door_XXXX",
    )
    ap.add_argument(
        "--include-door-variants", dest="skip_door_variants", action="store_false"
    )

    ap.add_argument(
        "--include-regex", default="", help="Only run categories matching this regex"
    )
    ap.add_argument(
        "--exclude-regex", default="", help="Skip categories matching this regex"
    )
    ap.add_argument(
        "--category-list-json",
        default=None,
        help="Optional JSON array of exact category names to run.",
    )

    ap.add_argument(
        "--set-instanceable",
        action="store_true",
        default=True,
        help="Pass --set-instanceable to bulk apply (default on)",
    )
    ap.add_argument(
        "--no-set-instanceable", dest="set_instanceable", action="store_false"
    )

    ap.add_argument(
        "--v-matrix-mode",
        choices=["none", "auto"],
        default="none",
        help="V matrix compensation mode: 'none' (legacy) or 'auto' (mode-dispatched)",
    )
    ap.add_argument(
        "--mode-reports-dir",
        default=None,
        help="Directory containing dedup mode reports (geom_only/, topo_filesize/, shape_invariant/)",
    )
    ap.add_argument(
        "--scene-files",
        default="layout.usd",
        help="Comma-separated scene USD filenames for bbox-gated apply/promote. Default focuses A/B on layout.usd only.",
    )
    ap.add_argument(
        "--bbox-gated",
        action="store_true",
        help="Enable bbox-gated cert -> apply -> audit -> step6 flow.",
    )
    ap.add_argument(
        "--bbox-policy",
        choices=["bbox_primary_rmse_observe", "bbox_primary_rmse_harder"],
        default="bbox_primary_rmse_observe",
    )
    ap.add_argument(
        "--baseline-root",
        default=None,
        help="Canonical upstream baseline recorded in bbox-gated runs.",
    )
    ap.add_argument(
        "--step6-mode",
        choices=["off", "dry_run", "apply"],
        default="off",
        help="Step6 behavior for bbox-gated runs.",
    )
    ap.add_argument(
        "--dedup-mode",
        choices=["geom_only", "shape_invariant", "topo_filesize", "transitive"],
        default="geom_only",
        help="Input report mode passed into bbox-gated certificate build.",
    )
    ap.add_argument(
        "--continue-on-failure",
        action="store_true",
        default=False,
        help="Log category failures to ledger and continue to next category instead of aborting the run.",
    )
    ap.add_argument("--eps-bbox", type=float, default=0.01)
    ap.add_argument("--eps-pos", type=float, default=0.01)
    ap.add_argument("--eps-angle", type=float, default=1.0)
    ap.add_argument("--eps-geom", type=float, default=0.01)
    ap.add_argument(
        "--input-layout-name",
        default=None,
        help="If set, pass --input-layout-name to c1_bulk_apply so it reads from a snapshot "
        "instead of layout.usd (e.g. layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd).",
    )

    args = ap.parse_args()

    dataset_root = (REPO_ROOT / args.dataset_root).resolve()
    bak_root = (REPO_ROOT / args.bak_root).resolve()
    report_path = (REPO_ROOT / args.report).resolve()
    c1_bulk_dir = (REPO_ROOT / args.c1_bulk_dir).resolve()

    if not ISAAC_PY.exists():
        raise FileNotFoundError(f"Missing Isaac wrapper: {ISAAC_PY}")
    for p in (SCRIPT_BUILD_MAPPING, SCRIPT_BULK_APPLY, SCRIPT_STEP6, SCRIPT_AUDIT):
        if not p.exists():
            raise FileNotFoundError(f"Missing required script: {p}")

    if args.bbox_gated:
        if not args.mode_reports_dir:
            raise SystemExit(
                "ERROR: --mode-reports-dir is required when --bbox-gated is set."
            )
        return _run_bbox_gated(args)

    c1_bulk_dir.mkdir(parents=True, exist_ok=True)

    include_re = re.compile(args.include_regex) if args.include_regex else None
    exclude_re = re.compile(args.exclude_regex) if args.exclude_regex else None
    category_allowlist = _load_category_allowlist(args.category_list_json)

    categories = _list_categories(dataset_root)
    if category_allowlist is not None:
        categories = [c for c in categories if c in category_allowlist]
    if args.skip_door_variants:
        categories = [c for c in categories if not c.startswith("door_")]

    if include_re:
        categories = [c for c in categories if include_re.search(c)]
    if exclude_re:
        categories = [c for c in categories if not exclude_re.search(c)]

    if args.skip_done:
        categories = [c for c in categories if not _is_done(c, c1_bulk_dir)]

    if args.max_categories and args.max_categories > 0:
        categories = categories[: args.max_categories]

    run_stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = c1_bulk_dir / "_autorun" / f"{args.group_label}_{run_stamp}"
    ledger_path = run_dir / "ledger.jsonl"

    _write_ledger(
        ledger_path,
        {
            "event": "run_start",
            "dataset_root": str(dataset_root.relative_to(REPO_ROOT)),
            "bak_root": str(bak_root.relative_to(REPO_ROOT)),
            "report": str(report_path.relative_to(REPO_ROOT)),
            "c1_bulk_dir": str(c1_bulk_dir.relative_to(REPO_ROOT)),
            "group_label": args.group_label,
            "out_version": args.out_version,
            "v_matrix_mode": args.v_matrix_mode,
            "mode_reports_dir": args.mode_reports_dir,
            "categories": categories,
        },
    )

    print(f"[autorun] categories_to_run={len(categories)} run_dir={run_dir}")

    for idx, category in enumerate(categories, start=1):
        step6_dir = _next_step6_dir(category, c1_bulk_dir, prefer_v1=True)
        plan = build_plan(
            category,
            c1_bulk_dir=c1_bulk_dir,
            group_label=args.group_label
            if args.group_label != "c1_autorun"
            else f"c1_{category}_bulk",
            out_version=args.out_version,
            step6_dir=step6_dir,
        )

        # If using auto per-category group label (recommended), keep it consistent with prior convention.
        group_label = args.group_label
        if args.group_label == "c1_autorun":
            group_label = f"c1_{category}_bulk"
            plan = build_plan(
                category,
                c1_bulk_dir=c1_bulk_dir,
                group_label=group_label,
                out_version=args.out_version,
                step6_dir=step6_dir,
            )

        print(
            f"\n[autorun] ({idx}/{len(categories)}) category={category} group_label={group_label}"
        )
        _write_ledger(
            ledger_path,
            {
                "event": "category_start",
                "category": category,
                "group_label": group_label,
            },
        )

        # 1) Build mapping if missing
        if not plan.mapping_json.exists():
            cmd = [
                str(ISAAC_PY),
                str(SCRIPT_BUILD_MAPPING.relative_to(REPO_ROOT)),
                "--report",
                str(report_path.relative_to(REPO_ROOT)),
                "--dataset-root",
                str(dataset_root.relative_to(REPO_ROOT)),
                "--category",
                category,
                "--out-mapping-json",
                str(plan.mapping_json.relative_to(REPO_ROOT)),
                "--out-stats-json",
                str(plan.mapping_stats_json.relative_to(REPO_ROOT)),
            ]
            rc = _run(
                cmd, cwd=REPO_ROOT, log_path=run_dir / category / "01_build_mapping.log"
            )
            if rc != 0:
                _write_ledger(
                    ledger_path,
                    {
                        "event": "category_fail",
                        "category": category,
                        "step": "build_mapping",
                        "rc": rc,
                    },
                )
                return rc

        pairs = _read_mapping_pairs(plan.mapping_stats_json)
        if pairs is not None and pairs == 0:
            print(f"[autorun] category={category} mapping_pairs=0 -> skip")
            _write_ledger(
                ledger_path,
                {
                    "event": "category_skip",
                    "category": category,
                    "reason": "mapping_pairs_0",
                },
            )
            continue

        # 2) Bulk apply (rewrite layout + start_result_*)
        cmd = [
            str(ISAAC_PY),
            str(SCRIPT_BULK_APPLY.relative_to(REPO_ROOT)),
            "--dataset-root",
            str(dataset_root.relative_to(REPO_ROOT)),
            "--mapping-json",
            str(plan.mapping_json.relative_to(REPO_ROOT)),
            "--group-label",
            group_label,
            "--out-name",
            plan.out_name,
            "--report-dir",
            str(plan.batch_report_dir.relative_to(REPO_ROOT)),
            "--v-matrix-mode",
            args.v_matrix_mode,
        ]
        if args.mode_reports_dir:
            cmd.extend(
                [
                    "--mode-reports-dir",
                    str(
                        (REPO_ROOT / args.mode_reports_dir)
                        .resolve()
                        .relative_to(REPO_ROOT)
                    ),
                ]
            )
        if args.input_layout_name:
            cmd.extend(["--input-layout-name", args.input_layout_name])
        if args.set_instanceable:
            cmd.append("--set-instanceable")
        rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "02_bulk_apply.log")
        if rc != 0:
            _write_ledger(
                ledger_path,
                {
                    "event": "category_fail",
                    "category": category,
                    "step": "bulk_apply",
                    "rc": rc,
                },
            )
            return rc

        # 3) Step6 (promote + scan gate + soft delete)
        cmd = [
            str(ISAAC_PY),
            str(SCRIPT_STEP6.relative_to(REPO_ROOT)),
            "--dataset-root",
            str(dataset_root.relative_to(REPO_ROOT)),
            "--bak-root",
            str(bak_root.relative_to(REPO_ROOT)),
            "--mapping-json",
            str(plan.mapping_json.relative_to(REPO_ROOT)),
            "--category",
            category,
            "--group-label",
            group_label,
            "--out-name",
            plan.out_name,
            "--report-dir",
            str(plan.step6_report_dir.relative_to(REPO_ROOT)),
        ]
        rc = _run(cmd, cwd=REPO_ROOT, log_path=run_dir / category / "03_step6.log")
        if rc != 0:
            _write_ledger(
                ledger_path,
                {
                    "event": "category_fail",
                    "category": category,
                    "step": "step6",
                    "rc": rc,
                },
            )
            return rc

        _write_ledger(
            ledger_path,
            {
                "event": "category_done",
                "category": category,
                "group_label": group_label,
                "step6_dir": str(plan.step6_report_dir.relative_to(REPO_ROOT)),
            },
        )

        if args.sleep_seconds and args.sleep_seconds > 0:
            time.sleep(args.sleep_seconds)

    _write_ledger(ledger_path, {"event": "run_done"})
    print("[autorun] DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
