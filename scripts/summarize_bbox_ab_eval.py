#!/usr/bin/env python3
"""Aggregate bbox-gated A/B evaluation outputs into JSON and Markdown."""

from __future__ import annotations

import argparse
import json
import re
import time
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


POLICY_FLAGS = {
    "policy_a": "bbox_primary_rmse_observe",
    "policy_b": "bbox_primary_rmse_harder",
}

SUMMARY_METRIC_KEYS = (
    "candidate_pairs",
    "eligible_pairs",
    "rejected_pairs",
    "mapping_pairs",
    "changed_layouts",
    "changed_scenes",
    "apply_reject_rows",
    "audit_passed_categories",
    "audit_failed_categories",
    "audit_ref_changed_fail_count",
    "audit_total_no_mesh",
    "audit_scenes_error",
    "step6_gate_passed_categories",
    "step6_gate_failed_categories",
    "step6_post_promote_hit_files",
    "step6_post_soft_delete_hit_layouts",
)


def _read_json(path: Path) -> Any | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        rows.append(json.loads(line))
    return rows


def _scene_id_from_layout_path(layout_in: str | None) -> str | None:
    if not layout_in:
        return None
    parts = Path(layout_in).parts
    try:
        idx = parts.index("GRScenes100")
    except ValueError:
        return None
    if len(parts) <= idx + 2:
        return None
    return f"{parts[idx + 1]}/{parts[idx + 2]}"


def _count_changed_scenes(batch_summary: dict[str, Any] | None) -> tuple[int, list[str]]:
    if not isinstance(batch_summary, dict):
        return 0, []
    changed: set[str] = set()
    for item in batch_summary.get("per_layout", []):
        if item.get("scene_file") != "layout.usd":
            continue
        counts = item.get("counts") or {}
        if not any(int(counts.get(key, 0)) for key in ("refs_changed", "payloads_changed", "asset_attrs_changed")):
            continue
        scene_id = _scene_id_from_layout_path(item.get("layout_in"))
        if scene_id:
            changed.add(scene_id)
    return len(changed), sorted(changed)


def _sum_counters(counters: Iterable[dict[str, Any]]) -> dict[str, int]:
    acc: Counter[str] = Counter()
    for payload in counters:
        if not isinstance(payload, dict):
            continue
        for key, value in payload.items():
            try:
                acc[str(key)] += int(value)
            except (TypeError, ValueError):
                continue
    return dict(sorted(acc.items()))


def _sort_top(counter: Counter[str], limit: int = 10) -> list[dict[str, Any]]:
    return [{"name": name, "count": count} for name, count in counter.most_common(limit)]


def _artifact_paths(category_root: Path) -> dict[str, Path]:
    return {
        "cert_summary": category_root / "01_cert" / "pair_certificate_summary.json",
        "mapping_stats": category_root / "01_cert" / "filtered_mapping.stats.json",
        "apply_summary": category_root / "02_apply" / "batch_summary.json",
        "apply_reject_ledger": category_root / "02_apply" / "rewrite_reject_ledger.jsonl",
        "audit_report": category_root / "03_audit" / "placement_pairwise_compare.json",
        "audit_verdict": category_root / "03_audit" / "audit_verdict.json",
        "step6_gate": category_root / "04_step6" / "step6_gate_decision.json",
        "step6_post_promote_full_scan": category_root / "04_step6" / "post_promote_full_usd_scan_excluding_backups_pxr.json",
        "step6_post_soft_delete_layout_scan": category_root / "04_step6" / "post_soft_delete_layout_scan_pxr.json",
    }


def _load_category_summary(category_root: Path, category: str, policy_name: str, policy_flag: str, version: str) -> dict[str, Any]:
    artifacts = _artifact_paths(category_root)
    missing_artifacts = [name for name, path in artifacts.items() if not path.exists()]

    cert_summary = _read_json(artifacts["cert_summary"]) or {}
    mapping_stats = _read_json(artifacts["mapping_stats"]) or {}
    batch_summary = _read_json(artifacts["apply_summary"]) or {}
    reject_rows = _read_jsonl(artifacts["apply_reject_ledger"])
    audit_report = _read_json(artifacts["audit_report"]) or {}
    audit_verdict = _read_json(artifacts["audit_verdict"]) or {}
    step6_gate = _read_json(artifacts["step6_gate"]) or {}
    step6_full_scan = _read_json(artifacts["step6_post_promote_full_scan"]) or {}
    step6_layout_scan = _read_json(artifacts["step6_post_soft_delete_layout_scan"]) or {}

    changed_scene_count, changed_scene_ids = _count_changed_scenes(batch_summary)
    reject_reason_counts = Counter()
    for row in reject_rows:
        reason = row.get("reject_reason") or row.get("kind") or "unknown"
        reject_reason_counts[str(reason)] += 1

    audit_global_worst = []
    aggregate = audit_report.get("aggregate") or {}
    for row in aggregate.get("global_top_50_worst", []) or []:
        scene_id = row.get("scene_id")
        if scene_id:
            audit_global_worst.append(str(scene_id))

    summary = {
        "category": category,
        "policy_name": policy_name,
        "policy_flag": policy_flag,
        "version": version,
        "root": str(category_root),
        "missing_artifacts": missing_artifacts,
        "cert": {
            "candidate_pairs": int(cert_summary.get("candidate_pairs", mapping_stats.get("candidate_pairs", 0)) or 0),
            "eligible_pairs": int(cert_summary.get("eligible_pairs", 0) or 0),
            "rejected_pairs": int(cert_summary.get("rejected_pairs", 0) or 0),
            "reject_reason_counts": cert_summary.get("reject_reason_counts", {}) or {},
            "mapping_pairs": int(mapping_stats.get("mapping_pairs", 0) or 0),
            "groups_included": int(cert_summary.get("groups_included", mapping_stats.get("groups_included", 0)) or 0),
        },
        "apply": {
            "changed_layouts": int(batch_summary.get("changed_layouts", 0) or 0),
            "layouts_total": int(batch_summary.get("layouts_total", 0) or 0),
            "changed_scene_count": changed_scene_count,
            "changed_scene_ids": changed_scene_ids,
            "reject_rows": len(reject_rows),
            "reject_reason_counts": dict(sorted(reject_reason_counts.items())),
        },
        "audit": {
            "present": artifacts["audit_verdict"].exists(),
            "passed": bool(audit_verdict.get("passed")) if audit_verdict else None,
            "compared_scope_complete": audit_verdict.get("compared_scope_complete"),
            "ref_changed_fail_count": int(audit_verdict.get("ref_changed_fail_count", 0) or 0),
            "total_no_mesh": int(audit_verdict.get("total_no_mesh", 0) or 0),
            "scenes_error": int(audit_verdict.get("scenes_error", 0) or 0),
            "blocking_reason_counts": audit_verdict.get("blocking_reason_counts", {}) or {},
            "matched_scene_count": int(audit_report.get("matched_scene_count", 0) or 0),
            "global_worst_scene_ids": audit_global_worst,
        },
        "step6": {
            "present": artifacts["step6_gate"].exists(),
            "audit_passed": step6_gate.get("audit_passed"),
            "post_promote_hit_files": int(step6_full_scan.get("hit_files", 0) or 0),
            "post_soft_delete_hit_layouts": int(step6_layout_scan.get("hit_layouts", 0) or 0),
        },
    }
    return summary


def _discover_categories(policy_root: Path, policy_name: str, policy_flag: str) -> dict[str, dict[str, Any]]:
    categories: dict[str, dict[str, Any]] = {}
    if not policy_root.is_dir():
        return categories
    pattern = re.compile(rf"^(?P<category>.+)_{re.escape(policy_flag)}_(?P<version>v\d+)$")
    for child in sorted(policy_root.iterdir()):
        if not child.is_dir() or child.name.startswith("_"):
            continue
        match = pattern.match(child.name)
        if not match:
            continue
        category = match.group("category")
        version = match.group("version")
        categories[category] = _load_category_summary(child, category, policy_name, policy_flag, version)
    return categories


def _policy_rollup(policy_name: str, policy_flag: str, categories: dict[str, dict[str, Any]]) -> dict[str, Any]:
    totals = {key: 0 for key in SUMMARY_METRIC_KEYS}
    cert_rejects: list[dict[str, Any]] = []
    apply_rejects: list[dict[str, Any]] = []
    audit_blocking: list[dict[str, Any]] = []
    top_mapping_pairs: list[tuple[int, str]] = []
    top_changed_layouts: list[tuple[int, str]] = []
    top_no_mesh: list[tuple[int, str]] = []
    top_ref_changed_fails: list[tuple[int, str]] = []
    scene_counter: Counter[str] = Counter()
    incomplete_categories = []

    for category, summary in categories.items():
        cert = summary["cert"]
        apply = summary["apply"]
        audit = summary["audit"]
        step6 = summary["step6"]

        totals["candidate_pairs"] += cert["candidate_pairs"]
        totals["eligible_pairs"] += cert["eligible_pairs"]
        totals["rejected_pairs"] += cert["rejected_pairs"]
        totals["mapping_pairs"] += cert["mapping_pairs"]
        totals["changed_layouts"] += apply["changed_layouts"]
        totals["changed_scenes"] += apply["changed_scene_count"]
        totals["apply_reject_rows"] += apply["reject_rows"]
        totals["audit_ref_changed_fail_count"] += audit["ref_changed_fail_count"]
        totals["audit_total_no_mesh"] += audit["total_no_mesh"]
        totals["audit_scenes_error"] += audit["scenes_error"]
        totals["step6_post_promote_hit_files"] += step6["post_promote_hit_files"]
        totals["step6_post_soft_delete_hit_layouts"] += step6["post_soft_delete_hit_layouts"]

        if audit["present"]:
            if audit["passed"]:
                totals["audit_passed_categories"] += 1
            else:
                totals["audit_failed_categories"] += 1
        if step6["present"]:
            if bool(step6["audit_passed"]):
                totals["step6_gate_passed_categories"] += 1
            else:
                totals["step6_gate_failed_categories"] += 1

        cert_rejects.append(cert["reject_reason_counts"])
        apply_rejects.append(apply["reject_reason_counts"])
        audit_blocking.append(audit["blocking_reason_counts"])

        top_mapping_pairs.append((cert["mapping_pairs"], category))
        top_changed_layouts.append((apply["changed_layouts"], category))
        top_no_mesh.append((audit["total_no_mesh"], category))
        top_ref_changed_fails.append((audit["ref_changed_fail_count"], category))
        scene_counter.update(audit["global_worst_scene_ids"])

        if summary["missing_artifacts"]:
            incomplete_categories.append(
                {
                    "category": category,
                    "missing_artifacts": summary["missing_artifacts"],
                }
            )

    def _top_category(rows: list[tuple[int, str]], limit: int = 10) -> list[dict[str, Any]]:
        rows = [row for row in rows if row[0] > 0]
        rows.sort(key=lambda item: (-item[0], item[1]))
        return [{"category": category, "value": value} for value, category in rows[:limit]]

    return {
        "policy_name": policy_name,
        "policy_flag": policy_flag,
        "discovered_category_count": len(categories),
        "totals": totals,
        "cert_reject_reason_counts": _sum_counters(cert_rejects),
        "apply_reject_reason_counts": _sum_counters(apply_rejects),
        "audit_blocking_reason_counts": _sum_counters(audit_blocking),
        "top_categories_by_mapping_pairs": _top_category(top_mapping_pairs),
        "top_categories_by_changed_layouts": _top_category(top_changed_layouts),
        "top_categories_by_total_no_mesh": _top_category(top_no_mesh),
        "top_categories_by_ref_changed_fail_count": _top_category(top_ref_changed_fails),
        "top_scenes_from_audit_global_worst": _sort_top(scene_counter),
        "incomplete_categories": incomplete_categories,
        "categories": categories,
    }


def _comparison_section(policy_a: dict[str, Any], policy_b: dict[str, Any]) -> dict[str, Any]:
    delta = {}
    for key in SUMMARY_METRIC_KEYS:
        delta[key] = int(policy_b["totals"].get(key, 0)) - int(policy_a["totals"].get(key, 0))

    all_categories = sorted(set(policy_a["categories"]) | set(policy_b["categories"]))
    category_deltas = []
    for category in all_categories:
        a = policy_a["categories"].get(category)
        b = policy_b["categories"].get(category)
        a_cert = (a or {}).get("cert", {})
        b_cert = (b or {}).get("cert", {})
        a_apply = (a or {}).get("apply", {})
        b_apply = (b or {}).get("apply", {})
        a_audit = (a or {}).get("audit", {})
        b_audit = (b or {}).get("audit", {})
        row = {
            "category": category,
            "mapping_pairs_delta_b_minus_a": int(b_cert.get("mapping_pairs", 0)) - int(a_cert.get("mapping_pairs", 0)),
            "eligible_pairs_delta_b_minus_a": int(b_cert.get("eligible_pairs", 0)) - int(a_cert.get("eligible_pairs", 0)),
            "rejected_pairs_delta_b_minus_a": int(b_cert.get("rejected_pairs", 0)) - int(a_cert.get("rejected_pairs", 0)),
            "changed_layouts_delta_b_minus_a": int(b_apply.get("changed_layouts", 0)) - int(a_apply.get("changed_layouts", 0)),
            "audit_ref_changed_fail_delta_b_minus_a": int(b_audit.get("ref_changed_fail_count", 0)) - int(a_audit.get("ref_changed_fail_count", 0)),
            "audit_total_no_mesh_delta_b_minus_a": int(b_audit.get("total_no_mesh", 0)) - int(a_audit.get("total_no_mesh", 0)),
        }
        if any(row[key] for key in row if key != "category"):
            category_deltas.append(row)

    category_deltas.sort(
        key=lambda row: (
            -abs(row["mapping_pairs_delta_b_minus_a"]),
            -abs(row["eligible_pairs_delta_b_minus_a"]),
            row["category"],
        )
    )
    return {
        "totals_delta_b_minus_a": delta,
        "category_deltas": category_deltas,
        "largest_mapping_pair_deltas": category_deltas[:10],
    }


def build_summary(run_root: Path) -> dict[str, Any]:
    policies = {}
    for policy_name, policy_flag in POLICY_FLAGS.items():
        policy_root = run_root / policy_name
        categories = _discover_categories(policy_root, policy_name, policy_flag)
        policies[policy_name] = _policy_rollup(policy_name, policy_flag, categories)
    return {
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "run_root": str(run_root.resolve()),
        "policies": policies,
        "comparison": _comparison_section(policies["policy_a"], policies["policy_b"]),
    }


def _totals_md_lines(label: str, policy: dict[str, Any]) -> list[str]:
    totals = policy["totals"]
    return [
        f"### {label}",
        "",
        f"- discovered categories: {policy['discovered_category_count']}",
        f"- mapping_pairs: {totals['mapping_pairs']}",
        f"- eligible_pairs: {totals['eligible_pairs']}",
        f"- rejected_pairs: {totals['rejected_pairs']}",
        f"- changed_layouts: {totals['changed_layouts']}",
        f"- changed_scenes: {totals['changed_scenes']}",
        f"- audit_passed_categories: {totals['audit_passed_categories']}",
        f"- audit_failed_categories: {totals['audit_failed_categories']}",
        f"- audit_ref_changed_fail_count: {totals['audit_ref_changed_fail_count']}",
        f"- audit_total_no_mesh: {totals['audit_total_no_mesh']}",
        f"- step6_gate_passed_categories: {totals['step6_gate_passed_categories']}",
        f"- step6_gate_failed_categories: {totals['step6_gate_failed_categories']}",
        "",
    ]


def _top_list_md(title: str, rows: list[dict[str, Any]], key: str) -> list[str]:
    lines = [f"### {title}", ""]
    if not rows:
        lines.append("- none")
        lines.append("")
        return lines
    for row in rows:
        name = row.get("category") or row.get("name")
        lines.append(f"- {name}: {row.get(key)}")
    lines.append("")
    return lines


def render_markdown(summary: dict[str, Any]) -> str:
    policy_a = summary["policies"]["policy_a"]
    policy_b = summary["policies"]["policy_b"]
    comparison = summary["comparison"]
    lines = [
        "# BBox-Gated A/B Comparison Summary",
        "",
        f"- generated_at_utc: {summary['generated_at_utc']}",
        f"- run_root: {summary['run_root']}",
        "",
    ]
    lines.extend(_totals_md_lines("Policy A", policy_a))
    lines.extend(_totals_md_lines("Policy B", policy_b))
    lines.append("## Deltas")
    lines.append("")
    for key, value in comparison["totals_delta_b_minus_a"].items():
        lines.append(f"- {key} (B - A): {value}")
    lines.append("")
    lines.extend(_top_list_md("Largest Mapping Pair Deltas", comparison["largest_mapping_pair_deltas"], "mapping_pairs_delta_b_minus_a"))
    lines.extend(_top_list_md("Policy A Top Categories By Mapping Pairs", policy_a["top_categories_by_mapping_pairs"], "value"))
    lines.extend(_top_list_md("Policy B Top Categories By Mapping Pairs", policy_b["top_categories_by_mapping_pairs"], "value"))
    lines.extend(_top_list_md("Policy A Top Audit Worst Scenes", policy_a["top_scenes_from_audit_global_worst"], "count"))
    lines.extend(_top_list_md("Policy B Top Audit Worst Scenes", policy_b["top_scenes_from_audit_global_worst"], "count"))

    incomplete_a = policy_a["incomplete_categories"]
    incomplete_b = policy_b["incomplete_categories"]
    lines.append("## Incomplete Categories")
    lines.append("")
    lines.append(f"- policy_a incomplete categories: {len(incomplete_a)}")
    lines.append(f"- policy_b incomplete categories: {len(incomplete_b)}")
    lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize bbox-gated A/B evaluation outputs.")
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--out-json", required=True)
    parser.add_argument("--out-md", required=True)
    args = parser.parse_args()

    run_root = Path(args.run_root).resolve()
    out_json = Path(args.out_json).resolve()
    out_md = Path(args.out_md).resolve()

    summary = build_summary(run_root)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    out_md.write_text(render_markdown(summary), encoding="utf-8")
    print(f"WROTE {out_json}")
    print(f"WROTE {out_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
