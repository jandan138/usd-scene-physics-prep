#!/usr/bin/env python3
"""Build mapping artifacts from a dedup report.

Legacy mode:
- emit a raw old->canonical mapping JSON and a stats JSON

BBox-gated mode:
- emit pair certificates
- emit a certified graph rebuilt from eligible edges only
- emit a filtered mapping JSON derived from certified components
- optionally revoke attributable old->canonical edges and rebuild
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import ijson  # type: ignore

try:
    from pxr import Usd  # type: ignore

    _PXR_ERR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Usd = None  # type: ignore
    _PXR_ERR = e

import compute_vertex_transform as _cvt


def _norm_asset_path(p: str) -> str:
    return (p or "").replace("\\", "/").strip()


def _strip_asset_sigils(p: str) -> str:
    p = _norm_asset_path(p)
    if p.startswith("@") and p.endswith("@"):
        return p[1:-1]
    return p


def _to_subset_rel_asset_path(p: str) -> Optional[str]:
    s = _strip_asset_sigils(p)
    if not s:
        return None

    marker = "GRScenes_assets/"
    idx = s.find(marker)
    if idx < 0:
        return None
    return s[idx:].lstrip("/")


def _to_dataset_rel_asset_path(subset_rel: str, dataset_name: str) -> str:
    subset_rel = _norm_asset_path(subset_rel).lstrip("/")
    return f"{dataset_name}/{subset_rel}"


def _abs_from_layout_ref(base_dir: str, asset_path: str) -> str:
    s = _strip_asset_sigils(asset_path)
    if not s:
        return ""
    if os.path.isabs(s):
        return os.path.abspath(s)
    return os.path.abspath(os.path.join(base_dir, s))


def _abs_to_subset_rel(abs_path: str) -> Optional[str]:
    p = _norm_asset_path(abs_path)
    marker = "/GRScenes_assets/"
    idx = p.find(marker)
    if idx < 0:
        return None
    return p[idx + 1 :].lstrip("/")


def _iter_duplicate_groups(report_path: Path) -> Iterable[Dict]:
    with open(report_path, "rb") as f:
        for item in ijson.items(f, "duplicates.item"):
            yield item


def _count_layout_asset_usage(dataset_root: Path, *, dataset_rel: bool) -> Counter:
    if _PXR_ERR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_ERR}")

    dataset_name = dataset_root.name
    layout_files = sorted((dataset_root / "GRScenes100").glob("**/layout.usd"))
    counts: Counter = Counter()

    for layout in layout_files:
        base_dir = str(layout.parent)
        stage = Usd.Stage.Open(str(layout), load=Usd.Stage.LoadNone)
        if stage is None:
            continue

        for prim in stage.Traverse():
            if not prim or not prim.IsValid():
                continue

            if prim.HasAuthoredReferences():
                refs = prim.GetMetadata("references")
                if refs:
                    try:
                        items = list(refs.GetAddedOrExplicitItems())
                    except Exception:
                        items = []
                    for r in items:
                        ap = getattr(r, "assetPath", "") or ""
                        subset_rel = _abs_to_subset_rel(_abs_from_layout_ref(base_dir, ap))
                        if not subset_rel:
                            continue
                        key = _to_dataset_rel_asset_path(subset_rel, dataset_name) if dataset_rel else subset_rel
                        counts[key] += 1

            if prim.HasAuthoredPayloads():
                pls = prim.GetMetadata("payloads")
                if pls:
                    try:
                        items = list(pls.GetAddedOrExplicitItems())
                    except Exception:
                        items = []
                    for r in items:
                        ap = getattr(r, "assetPath", "") or ""
                        subset_rel = _abs_to_subset_rel(_abs_from_layout_ref(base_dir, ap))
                        if not subset_rel:
                            continue
                        key = _to_dataset_rel_asset_path(subset_rel, dataset_name) if dataset_rel else subset_rel
                        counts[key] += 1

    return counts


def _read_revoked_edges(path: Optional[str]) -> Set[Tuple[str, str]]:
    if not path:
        return set()
    p = Path(path)
    if not p.exists():
        return set()

    revoked: Set[Tuple[str, str]] = set()
    with p.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            item = json.loads(line)
            old_asset = item.get("old_asset")
            canonical_asset = item.get("canonical_asset")
            if isinstance(old_asset, str) and isinstance(canonical_asset, str):
                revoked.add((_norm_asset_path(old_asset), _norm_asset_path(canonical_asset)))
    return revoked


def _connected_components(nodes: Sequence[str], edges: Sequence[Tuple[str, str]]) -> List[List[str]]:
    adjacency: Dict[str, Set[str]] = defaultdict(set)
    for left, right in edges:
        adjacency[left].add(right)
        adjacency[right].add(left)

    seen: Set[str] = set()
    components: List[List[str]] = []
    for node in sorted(set(nodes)):
        if node in seen:
            continue
        stack = [node]
        comp: List[str] = []
        seen.add(node)
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for nxt in sorted(adjacency.get(cur, ())):
                if nxt in seen:
                    continue
                seen.add(nxt)
                stack.append(nxt)
        components.append(sorted(comp))
    return components


def _component_canonical(component: Sequence[str], usage: Counter) -> str:
    scored = [(usage.get(path, 0), path) for path in component]
    scored.sort(key=lambda item: (-item[0], item[1]))
    return scored[0][1]


def _infer_dedup_mode(report_path: Path, explicit_mode: Optional[str]) -> str:
    if explicit_mode:
        return explicit_mode
    lower = str(report_path).lower()
    if "shape_invariant" in lower:
        return "shape_invariant"
    if "topo_filesize" in lower:
        return "topo_filesize"
    return "geom_only"


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _run_legacy(args: argparse.Namespace) -> int:
    report = Path(args.report)
    dataset_root = Path(args.dataset_root)
    dataset_name = dataset_root.name

    usage = _count_layout_asset_usage(dataset_root, dataset_rel=True)

    mapping: Dict[str, str] = {}
    group_count = 0
    group_kept = 0
    assets_total = 0
    assets_kept = 0
    canonical_selections: List[Dict] = []

    for g in _iter_duplicate_groups(report):
        group_count += 1
        sig = str(g.get("sig") or "")
        raw_paths = [p for p in (g.get("usd_paths") or []) if isinstance(p, str)]
        paths: List[str] = []
        for raw in raw_paths:
            subset_rel = _to_subset_rel_asset_path(raw)
            if not subset_rel:
                continue
            path = _to_dataset_rel_asset_path(subset_rel, dataset_name)
            paths.append(path)

        if not paths:
            continue

        if args.category:
            want = str(args.category)
            paths = [p for p in paths if f"{dataset_name}/GRScenes_assets/{want}/" in p]
            if not paths:
                continue

        scored = [(usage.get(p, 0), p) for p in sorted(set(paths))]
        scored.sort(key=lambda x: (-x[0], x[1]))
        canonical = scored[0][1]

        group_kept += 1
        assets_total += len(paths)
        assets_kept += 1

        for p in paths:
            if p == canonical:
                continue
            mapping[p] = canonical

        canonical_selections.append(
            {
                "sig": sig,
                "member_count": len(paths),
                "canonical": canonical,
                "canonical_usage_in_layouts": int(usage.get(canonical, 0)),
            }
        )

        if args.progress_every and group_count % args.progress_every == 0:
            print(
                f"groups_seen={group_count} kept={group_kept} mapping_pairs={len(mapping)}",
                flush=True,
            )

    canonical_set = set(mapping.values())
    conflict_keys = [k for k in mapping if k in canonical_set]
    if conflict_keys:
        print(
            f"WARNING: removing {len(conflict_keys)} transitive canonical conflicts "
            f"(old assets that are also canonical elsewhere)",
            flush=True,
        )
        for k in conflict_keys:
            del mapping[k]

    out_mapping = Path(args.out_mapping_json)
    out_stats = Path(args.out_stats_json)
    _write_json(out_mapping, mapping)
    _write_json(
        out_stats,
        {
            "report": str(report),
            "dataset_root": str(dataset_root),
            "category": args.category,
            "groups_seen": group_count,
            "groups_included": group_kept,
            "assets_included_total": assets_total,
            "canonicals_kept": assets_kept,
            "mapping_pairs": len(mapping),
            "canonical_selections": canonical_selections,
        },
    )
    print(f"DONE groups_included={group_kept} mapping_pairs={len(mapping)} out={out_mapping}", flush=True)
    return 0


def _run_bbox_gated(args: argparse.Namespace) -> int:
    report = Path(args.report)
    dataset_root = Path(args.dataset_root)
    usage = _count_layout_asset_usage(dataset_root, dataset_rel=False)
    report_mode = _infer_dedup_mode(report, args.dedup_mode)
    revoked_edges = _read_revoked_edges(args.revoked_edges_jsonl)

    certificate_rows: List[Dict[str, object]] = []
    eligible_edges: List[Tuple[str, str]] = []
    all_nodes: Set[str] = set()
    candidate_pair_count = 0
    groups_seen = 0
    groups_included = 0
    rejected_by_reason: Counter = Counter()

    for group in _iter_duplicate_groups(report):
        groups_seen += 1
        sig = str(group.get("sig") or "")
        raw_paths = [p for p in (group.get("usd_paths") or []) if isinstance(p, str)]
        paths = []
        for raw in raw_paths:
            subset_rel = _to_subset_rel_asset_path(raw)
            if subset_rel:
                paths.append(subset_rel)

        if not paths:
            continue

        if args.category:
            want = str(args.category)
            paths = [p for p in paths if f"GRScenes_assets/{want}/" in p]
            if not paths:
                continue

        paths = sorted(set(paths))
        if len(paths) < 2:
            continue

        groups_included += 1
        initial_canonical = _component_canonical(paths, usage)
        all_nodes.update(paths)

        for old_asset in paths:
            if old_asset == initial_canonical:
                continue

            candidate_pair_count += 1
            cert = _cvt.build_pair_certificate(
                old_usd=str(dataset_root / old_asset),
                canonical_usd=str(dataset_root / initial_canonical),
                mode=report_mode,
                policy=args.bbox_policy,
            )
            cert["old_asset"] = old_asset
            cert["canonical_asset"] = initial_canonical
            cert["group_sig"] = sig
            cert["group_member_count"] = len(paths)
            cert["initial_canonical"] = initial_canonical
            cert["old_asset_usage_in_layouts"] = int(usage.get(old_asset, 0))
            cert["canonical_usage_in_layouts"] = int(usage.get(initial_canonical, 0))

            if (old_asset, initial_canonical) in revoked_edges:
                cert["eligible"] = False
                cert["reject_reason"] = "revoked_edge"
                cert["proof_source"] = "revoked_edge"
                cert["alternate_proof_kind"] = cert.get("alternate_proof_kind")
                cert["alternate_proof_passed"] = False

            if cert.get("eligible"):
                eligible_edges.append((initial_canonical, old_asset))
            else:
                rejected_by_reason[str(cert.get("reject_reason") or "unknown")] += 1

            certificate_rows.append(cert)

    components = _connected_components(sorted(all_nodes), eligible_edges)
    filtered_mapping: Dict[str, str] = {}
    component_rows: List[Dict[str, object]] = []
    mapping_pairs = 0

    for idx, component in enumerate(components, start=1):
        if len(component) < 2:
            component_rows.append(
                {
                    "component_id": idx,
                    "canonical": component[0],
                    "nodes": component,
                    "mapping_pairs": [],
                    "eligible": False,
                }
            )
            continue

        canonical = _component_canonical(component, usage)
        pairs = []
        for node in component:
            if node == canonical:
                continue
            filtered_mapping[node] = canonical
            pairs.append({"old_asset": node, "canonical_asset": canonical})
            mapping_pairs += 1

        component_rows.append(
            {
                "component_id": idx,
                "canonical": canonical,
                "nodes": component,
                "mapping_pairs": pairs,
                "eligible": True,
            }
        )

    out_mapping = Path(args.out_mapping_json)
    out_stats = Path(args.out_stats_json)
    out_cert_jsonl = Path(args.out_certificate_jsonl)
    out_cert_summary = Path(args.out_certificate_summary_json)
    out_graph = Path(args.out_certified_graph_json)

    out_cert_jsonl.parent.mkdir(parents=True, exist_ok=True)
    with out_cert_jsonl.open("w", encoding="utf-8") as f:
        for row in certificate_rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    _write_json(out_mapping, filtered_mapping)
    _write_json(
        out_stats,
        {
            "report": str(report),
            "dataset_root": str(dataset_root),
            "category": args.category,
            "bbox_gated": True,
            "bbox_policy": args.bbox_policy,
            "dedup_mode": report_mode,
            "groups_seen": groups_seen,
            "groups_included": groups_included,
            "candidate_pairs": candidate_pair_count,
            "mapping_pairs": mapping_pairs,
            "certificate_jsonl": str(out_cert_jsonl),
            "certificate_summary_json": str(out_cert_summary),
            "certified_graph_json": str(out_graph),
            "revoked_edges_jsonl": args.revoked_edges_jsonl,
            "canonical_selections": [
                {
                    "component_id": row["component_id"],
                    "canonical": row["canonical"],
                    "member_count": len(row["nodes"]),
                    "eligible": row["eligible"],
                }
                for row in component_rows
            ],
        },
    )
    _write_json(
        out_cert_summary,
        {
            "report": str(report),
            "dataset_root": str(dataset_root),
            "bbox_policy": args.bbox_policy,
            "dedup_mode": report_mode,
            "category": args.category,
            "groups_seen": groups_seen,
            "groups_included": groups_included,
            "candidate_pairs": candidate_pair_count,
            "eligible_pairs": len(eligible_edges),
            "rejected_pairs": candidate_pair_count - len(eligible_edges),
            "reject_reason_counts": dict(sorted(rejected_by_reason.items())),
            "revoked_edge_count": sum(1 for row in certificate_rows if row.get("reject_reason") == "revoked_edge"),
        },
    )
    _write_json(
        out_graph,
        {
            "bbox_policy": args.bbox_policy,
            "dedup_mode": report_mode,
            "category": args.category,
            "eligible_edge_count": len(eligible_edges),
            "eligible_edges": [
                {"canonical_asset": left, "old_asset": right}
                for left, right in eligible_edges
            ],
            "revoked_edges": [
                {"old_asset": old, "canonical_asset": canonical}
                for old, canonical in sorted(revoked_edges)
            ],
            "components": component_rows,
        },
    )

    print(
        f"DONE bbox_gated category={args.category or 'ALL'} candidate_pairs={candidate_pair_count} "
        f"eligible_pairs={len(eligible_edges)} mapping_pairs={mapping_pairs} out={out_mapping}",
        flush=True,
    )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--report",
        default="check_reports/test1_asset_mesh_dedup_geom_only.json",
        help="Dedup report JSON",
    )
    ap.add_argument(
        "--dataset-root",
        default="GRScenes-test1",
        help="Dataset root containing GRScenes_assets and GRScenes100",
    )
    ap.add_argument(
        "--category",
        default=None,
        help="If set, only build mapping for this category.",
    )
    ap.add_argument(
        "--out-mapping-json",
        required=True,
        help="Output mapping JSON path",
    )
    ap.add_argument(
        "--out-stats-json",
        required=True,
        help="Output stats JSON path",
    )
    ap.add_argument(
        "--progress-every",
        type=int,
        default=200,
        help="Print progress every N duplicate groups",
    )
    ap.add_argument(
        "--bbox-gated",
        action="store_true",
        help="Emit bbox-gated certificate, graph, and filtered mapping artifacts.",
    )
    ap.add_argument(
        "--bbox-policy",
        choices=["bbox_primary_rmse_observe", "bbox_primary_rmse_harder"],
        default="bbox_primary_rmse_observe",
        help="Policy variant to encode in the certificate artifacts.",
    )
    ap.add_argument(
        "--dedup-mode",
        choices=["geom_only", "shape_invariant", "topo_filesize", "transitive"],
        default=None,
        help="Explicit mode for the input report. Defaults to path-based inference.",
    )
    ap.add_argument(
        "--out-certificate-jsonl",
        default=None,
        help="Required in --bbox-gated mode: output JSONL path for pair certificates.",
    )
    ap.add_argument(
        "--out-certificate-summary-json",
        default=None,
        help="Required in --bbox-gated mode: output JSON path for certificate summary.",
    )
    ap.add_argument(
        "--out-certified-graph-json",
        default=None,
        help="Required in --bbox-gated mode: output JSON path for certified graph/components.",
    )
    ap.add_argument(
        "--revoked-edges-jsonl",
        default=None,
        help="Optional JSONL of attributable revoked old->canonical edges for rebuilds.",
    )
    args = ap.parse_args()

    if args.bbox_gated:
        required = {
            "--out-certificate-jsonl": args.out_certificate_jsonl,
            "--out-certificate-summary-json": args.out_certificate_summary_json,
            "--out-certified-graph-json": args.out_certified_graph_json,
        }
        missing = [flag for flag, value in required.items() if not value]
        if missing:
            raise SystemExit(f"Missing required bbox-gated outputs: {', '.join(missing)}")
        return _run_bbox_gated(args)

    return _run_legacy(args)


if __name__ == "__main__":
    raise SystemExit(main())
