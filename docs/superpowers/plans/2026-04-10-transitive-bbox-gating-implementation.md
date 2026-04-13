---
title: Transitive BBox Gating Implementation Plan
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/c1_autorun_categories.py
  - tests/test_compute_vertex_transform.py
  - tests/test_bbox_gated_mapping.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
  - tests/test_c1_autorun_categories.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: approved
doc_class: archive
---

# Transitive BBox Gating Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add transitive-edge support to the bbox-gated dedup pipeline by letting transitive paths generate candidate endpoint transforms, then certifying those pairs only when direct endpoint verification passes.

**Architecture:** Keep the current 3-layer contract intact. `compute_vertex_transform.py` remains the geometry source of truth and learns deterministic transitive witness selection plus shared V-based certificate evaluation. `c1_build_bulk_mapping_from_dedup_report.py` stops hard-rejecting transitive pairs and instead calls the new transitive builder. `placement_pairwise_compare.py` becomes certificate-aware so audit uses the same effective-mode semantics that cert used, and `c1_autorun_categories.py` simply passes the existing per-category `pair_certificates.jsonl` into audit.

**Tech Stack:** Python 3, numpy, pxr/USD, pytest, Isaac Sim wrapper for pxr-backed tests

---

## File Structure

- Modify: `scripts/compute_vertex_transform.py`
  Purpose: deterministic witness-path selection, transitive `V` metadata, shared endpoint verification, and a dedicated transitive certificate builder.
- Modify: `scripts/c1_build_bulk_mapping_from_dedup_report.py`
  Purpose: call the transitive builder instead of emitting `transitive_not_supported`, while preserving summary and certified-graph semantics.
- Modify: `scripts/placement_pairwise_compare.py`
  Purpose: load certificate metadata, resolve effective mode from certificates first, and apply audit thresholds to certified transitive pairs.
- Modify: `scripts/c1_autorun_categories.py`
  Purpose: pass `pair_certificates.jsonl` to audit and extract the audit command assembly into a unit-testable helper.
- Modify: `tests/test_compute_vertex_transform.py`
  Purpose: cover deterministic witness selection, effective-mode summarization, and transitive endpoint verification.
- Modify: `tests/test_bbox_gated_mapping.py`
  Purpose: prove bbox-gated mapping now admits safe transitive pairs and reports transitive rejects cleanly.
- Modify: `tests/test_placement_pairwise_compare_bbox_gate.py`
  Purpose: prove audit prefers certificate semantics over legacy fallback and softens certified transitive tier2 edges under observe policy.
- Create: `tests/test_c1_autorun_categories.py`
  Purpose: verify audit command wiring includes `--certificate-jsonl`.
- Create: `docs/records/changes/2026-04-10_transitive_bbox_gating.md`
  Purpose: record the landed feature, test evidence, and remaining rollout follow-up.
- Modify: `docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md`
  Purpose: note that transitive edges are now supported in cert/audit code, while rollout metrics remain unchanged until a rerun is executed.

### Task 1: Add Deterministic Transitive Witnesses And Shared Certificate Evaluation

**Files:**
- Modify: `scripts/compute_vertex_transform.py:99-122`
- Modify: `scripts/compute_vertex_transform.py:805-979`
- Modify: `scripts/compute_vertex_transform.py:1006-1145`
- Test: `tests/test_compute_vertex_transform.py:12-329`

- [ ] **Step 1: Write the failing tests**

Add the new imports and tests below to `tests/test_compute_vertex_transform.py`.

```python
from compute_vertex_transform import (
    build_pair_certificate,
    build_transitive_pair_certificate,
    find_transitive_witness,
    procrustes_full,
    compute_V_shape_invariant,
)


def _asset_path(uid: str) -> str:
    return f"/tmp/GRScenes_assets/chair/{uid}/usd/{uid}.usd"


def test_find_transitive_witness_prefers_lower_risk_shortest_path(monkeypatch):
    members = [_asset_path(uid) for uid in ("canon", "mid_good", "mid_risky", "old")]
    mode_index = {
        ("canon", "mid_good"): "geom_only",
        ("mid_good", "canon"): "geom_only",
        ("mid_good", "old"): "topo_filesize",
        ("old", "mid_good"): "topo_filesize",
        ("canon", "mid_risky"): "shape_invariant",
        ("mid_risky", "canon"): "shape_invariant",
        ("mid_risky", "old"): "shape_invariant",
        ("old", "mid_risky"): "shape_invariant",
    }

    monkeypatch.setattr(
        "compute_vertex_transform._accumulate_V_along_path",
        lambda path: np.eye(4, dtype=np.float64),
    )

    witness = find_transitive_witness(
        old_usd=_asset_path("old"),
        canonical_usd=_asset_path("canon"),
        group_members=members,
        mode_index=mode_index,
    )

    assert witness["uids"] == ["canon", "mid_good", "old"]
    assert witness["modes"] == ["geom_only", "topo_filesize"]
    assert witness["effective_mode"] == "topo_filesize"
    assert witness["hops"] == 2
    assert witness["path_hash"]


def test_build_transitive_pair_certificate_records_witness(monkeypatch):
    monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)
    monkeypatch.setattr(
        "compute_vertex_transform.find_transitive_witness",
        lambda **kwargs: {
            "V": np.eye(4, dtype=np.float64),
            "uids": ["canon", "mid", "old"],
            "modes": ["geom_only", "topo_filesize"],
            "effective_mode": "topo_filesize",
            "path_hash": "abc123deadbeef00",
            "hops": 2,
        },
    )
    monkeypatch.setattr(
        "compute_vertex_transform.extract_instance_space_vertices",
        lambda path: BASE_PTS.copy(),
    )

    cert = build_transitive_pair_certificate(
        old_usd="/tmp/old.usd",
        canonical_usd="/tmp/canon.usd",
        group_members=["/tmp/canon.usd", "/tmp/mid.usd", "/tmp/old.usd"],
        mode_index={("canon", "mid"): "geom_only"},
    )

    assert cert["eligible"] is True
    assert cert["proof_source"] == "transitive_bbox_gated_proof"
    assert cert["transitive_endpoint_verified"] is True
    assert cert["transitive_effective_mode"] == "topo_filesize"
    assert cert["transitive_witness_uids"] == ["canon", "mid", "old"]
    assert cert["transitive_witness_modes"] == ["geom_only", "topo_filesize"]


def test_build_transitive_pair_certificate_fails_closed_on_endpoint_bbox(monkeypatch):
    monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)
    monkeypatch.setattr(
        "compute_vertex_transform.find_transitive_witness",
        lambda **kwargs: {
            "V": np.eye(4, dtype=np.float64),
            "uids": ["canon", "mid", "old"],
            "modes": ["shape_invariant", "shape_invariant"],
            "effective_mode": "shape_invariant",
            "path_hash": "badbadbadbadbad0",
            "hops": 2,
        },
    )
    monkeypatch.setattr(
        "compute_vertex_transform.extract_instance_space_vertices",
        lambda path: BASE_PTS.copy() + np.array([10.0, 0.0, 0.0]) if "old" in path else BASE_PTS.copy(),
    )

    cert = build_transitive_pair_certificate(
        old_usd="/tmp/old.usd",
        canonical_usd="/tmp/canon.usd",
        group_members=["/tmp/canon.usd", "/tmp/mid.usd", "/tmp/old.usd"],
        mode_index={("canon", "mid"): "shape_invariant"},
    )

    assert cert["eligible"] is False
    assert cert["reject_reason"] == "bbox_precheck_failed_transitive"
    assert cert["transitive_endpoint_verified"] is False
```

- [ ] **Step 2: Run the compute tests to verify they fail**

Run:

```bash
python -m pytest tests/test_compute_vertex_transform.py -q
```

Expected:

- FAIL because `build_transitive_pair_certificate` and `find_transitive_witness` do not exist yet.
- Existing `mode_not_enabled_transitive` behavior still makes the new transitive certificate expectations fail.

- [ ] **Step 3: Implement deterministic witness selection and transitive certificate building**

Modify `scripts/compute_vertex_transform.py` with the focused additions below.

```python
def _mode_risk(mode: str) -> int:
    return {"geom_only": 0, "topo_filesize": 1, "shape_invariant": 2}.get(mode, 99)


def _effective_mode_from_modes(modes: Sequence[str]) -> str:
    if "shape_invariant" in modes:
        return "shape_invariant"
    if "topo_filesize" in modes:
        return "topo_filesize"
    return "geom_only"


def find_transitive_witness(
    old_usd: str,
    canonical_usd: str,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
) -> Dict[str, Any]:
    canonical_uid = _uid_from_path(canonical_usd)
    old_uid = _uid_from_path(old_usd)
    graph = _build_group_graph(group_members, mode_index)
    uid_to_path = {_uid_from_path(p): p for p in group_members}

    import hashlib
    import heapq

    heap = [(0, tuple(), (canonical_uid,), canonical_uid, [])]
    seen: Dict[str, tuple] = {}

    while heap:
        hops, risk_seq, uid_path, cur_uid, path = heapq.heappop(heap)
        state_key = (cur_uid, uid_path)
        if state_key in seen:
            continue
        seen[state_key] = (hops, risk_seq, uid_path)
        if cur_uid == old_uid:
            modes = [step[2] for step in path]
            payload = {"uids": list(uid_path), "modes": modes}
            return {
                "V": _accumulate_V_along_path(path),
                "uids": list(uid_path),
                "modes": modes,
                "effective_mode": _effective_mode_from_modes(modes),
                "path_hash": hashlib.sha1(
                    json.dumps(payload, sort_keys=True).encode("utf-8")
                ).hexdigest()[:16],
                "hops": len(modes),
            }

        for neighbor_uid, mode, neighbor_usd in sorted(graph.get(cur_uid, []), key=lambda item: item[0]):
            if neighbor_uid in uid_path:
                continue
            next_path = path + [
                (cur_uid, neighbor_uid, mode, uid_to_path[cur_uid], neighbor_usd)
            ]
            next_modes = [step[2] for step in next_path]
            heapq.heappush(
                heap,
                (
                    len(next_path),
                    tuple(_mode_risk(m) for m in next_modes),
                    tuple(list(uid_path) + [neighbor_uid]),
                    neighbor_uid,
                    next_path,
                ),
            )

    raise RuntimeError(
        f"No transitive path from canonical={canonical_uid} to old={old_uid}"
    )


def build_pair_certificate_from_V(
    old_usd: str,
    canonical_usd: str,
    mode: str,
    V_np: np.ndarray,
    *,
    policy: str = "bbox_primary_rmse_observe",
) -> Dict[str, Any]:
    cert = _base_pair_certificate(old_usd=old_usd, canonical_usd=canonical_usd, mode=mode, policy=policy)
    old_pts = extract_instance_space_vertices(old_usd)
    canonical_pts = extract_instance_space_vertices(canonical_usd)
    if old_pts is None or canonical_pts is None:
        cert["reject_reason"] = "mesh_probe_failed"
        cert["rmse_unavailable_reason"] = "mesh_probe_failed"
        return cert

    canon_h = np.hstack([canonical_pts, np.ones((len(canonical_pts), 1))])
    canon_transformed = (canon_h @ V_np)[:, :3]
    bbox_old = {"min": old_pts.min(axis=0).tolist(), "max": old_pts.max(axis=0).tolist()}
    bbox_new = {"min": canon_transformed.min(axis=0).tolist(), "max": canon_transformed.max(axis=0).tolist()}
    delta_min = [abs(a - b) for a, b in zip(bbox_old["min"], bbox_new["min"])]
    delta_max = [abs(a - b) for a, b in zip(bbox_old["max"], bbox_new["max"])]
    max_abs = max(max(delta_min), max(delta_max))
    cert["bbox_delta"] = {"min": delta_min, "max": delta_max, "max_abs": max_abs}
    cert["bbox_delta_available"] = True
    cert["centroid_delta"] = float(np.linalg.norm(np.mean(old_pts, axis=0) - np.mean(canon_transformed, axis=0)))

    if len(old_pts) == len(canon_transformed):
        cert["vertex_rmse"] = float(np.sqrt(np.mean(np.sum((old_pts - canon_transformed) ** 2, axis=1))))
        cert["rmse_available"] = True
    else:
        cert["vertex_rmse"] = None
        cert["rmse_available"] = False
        cert["rmse_unavailable_reason"] = "vertex_count_mismatch"

    if max_abs > 0.5:
        cert["eligible"] = False
        cert["reject_reason"] = f"bbox_precheck_failed_{mode}"
        return cert

    cert["eligible"] = True
    cert["reject_reason"] = None
    return cert


def build_transitive_pair_certificate(
    old_usd: str,
    canonical_usd: str,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
    *,
    policy: str = "bbox_primary_rmse_observe",
) -> Dict[str, Any]:
    witness = find_transitive_witness(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        group_members=group_members,
        mode_index=mode_index,
    )
    cert = build_pair_certificate_from_V(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        mode="transitive",
        V_np=witness["V"],
        policy=policy,
    )
    cert["v_source"] = "transitive_composed"
    cert["alternate_proof_kind"] = "transitive_bbox_gated_proof"
    cert["alternate_proof_passed"] = bool(cert.get("eligible"))
    cert["proof_source"] = "transitive_bbox_gated_proof" if cert.get("eligible") else None
    cert["transitive_hops"] = witness["hops"]
    cert["transitive_witness_uids"] = witness["uids"]
    cert["transitive_witness_modes"] = witness["modes"]
    cert["transitive_effective_mode"] = witness["effective_mode"]
    cert["transitive_path_hash"] = witness["path_hash"]
    cert["transitive_endpoint_verified"] = bool(cert.get("eligible"))
    return cert


def find_transitive_V(
    old_usd: str,
    canonical_usd: str,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
) -> np.ndarray:
    return find_transitive_witness(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        group_members=group_members,
        mode_index=mode_index,
    )["V"]
```

- [ ] **Step 4: Run the compute tests to verify they pass**

Run:

```bash
python -m pytest tests/test_compute_vertex_transform.py -q
```

Expected:

- PASS
- The old `mode_not_enabled_transitive` test should be replaced by the new transitive certificate tests, since `transitive` is no longer supposed to fail at the allowed-modes gate.

- [ ] **Step 5: Commit this compute-layer slice if commits are requested for the implementation session**

```bash
git add tests/test_compute_vertex_transform.py scripts/compute_vertex_transform.py
git commit -m "feat: add transitive witness-based bbox certificates"
```

### Task 2: Admit Transitive Certificates In Mapping Build

**Files:**
- Modify: `scripts/c1_build_bulk_mapping_from_dedup_report.py:303-565`
- Test: `tests/test_bbox_gated_mapping.py:22-194`

- [ ] **Step 1: Write the failing mapping tests**

Append the tests below to `tests/test_bbox_gated_mapping.py`.

```python
def test_bbox_gated_transitive_pair_is_certified_and_mapped(tmp_path, monkeypatch):
    report = tmp_path / "report.json"
    _write_report(
        report,
        [
            {
                "sig": "g1",
                "usd_paths": [
                    "/root/GRScenes_assets/chair/a/usd/a.usd",
                    "/root/GRScenes_assets/chair/b/usd/b.usd",
                    "/root/GRScenes_assets/chair/c/usd/c.usd",
                ],
            }
        ],
    )
    dataset_root = tmp_path / "dataset"
    (dataset_root / "GRScenes100").mkdir(parents=True)

    usage = Counter(
        {
            "GRScenes_assets/chair/a/usd/a.usd": 1,
            "GRScenes_assets/chair/b/usd/b.usd": 20,
            "GRScenes_assets/chair/c/usd/c.usd": 2,
        }
    )
    monkeypatch.setattr(mapping_mod, "_count_layout_asset_usage", lambda *args, **kwargs: usage)
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_mode_index",
        lambda _: {
            ("b", "a"): "geom_only",
            ("a", "b"): "geom_only",
            ("a", "c"): "topo_filesize",
            ("c", "a"): "topo_filesize",
        },
    )
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_pair_certificate",
        lambda **kwargs: {
            "eligible": True,
            "reject_reason": None,
            "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
            "footprint_extent_delta": 0.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 0.0,
            "vertex_rmse": 0.0,
            "rmse_available": True,
            "rmse_unavailable_reason": None,
            "alternate_proof_kind": "geom_only_exact_world_proof",
            "alternate_proof_passed": True,
            "proof_source": "geom_only_exact_world_proof",
        },
    )
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_transitive_pair_certificate",
        lambda **kwargs: {
            "eligible": True,
            "reject_reason": None,
            "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
            "footprint_extent_delta": 0.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 0.0,
            "vertex_rmse": 0.0,
            "rmse_available": True,
            "rmse_unavailable_reason": None,
            "alternate_proof_kind": "transitive_bbox_gated_proof",
            "alternate_proof_passed": True,
            "proof_source": "transitive_bbox_gated_proof",
            "transitive_effective_mode": "topo_filesize",
            "transitive_witness_uids": ["b", "a", "c"],
            "transitive_witness_modes": ["geom_only", "topo_filesize"],
        },
    )

    out_mapping = tmp_path / "filtered_mapping.json"
    out_stats = tmp_path / "filtered_mapping.stats.json"
    out_cert_jsonl = tmp_path / "pair_certificates.jsonl"
    out_cert_summary = tmp_path / "pair_certificate_summary.json"
    out_graph = tmp_path / "certified_graph.json"
    args = Namespace(
        report=str(report),
        dataset_root=str(dataset_root),
        category=None,
        out_mapping_json=str(out_mapping),
        out_stats_json=str(out_stats),
        progress_every=0,
        bbox_policy="bbox_primary_rmse_observe",
        dedup_mode="geom_only",
        out_certificate_jsonl=str(out_cert_jsonl),
        out_certificate_summary_json=str(out_cert_summary),
        out_certified_graph_json=str(out_graph),
        revoked_edges_jsonl=None,
        mode_reports_dir="/fake/modes",
    )

    assert mapping_mod._run_bbox_gated(args) == 0
    mapping = json.loads(out_mapping.read_text(encoding="utf-8"))
    assert mapping == {
        "GRScenes_assets/chair/a/usd/a.usd": "GRScenes_assets/chair/b/usd/b.usd",
        "GRScenes_assets/chair/c/usd/c.usd": "GRScenes_assets/chair/b/usd/b.usd",
    }
    summary = json.loads(out_cert_summary.read_text(encoding="utf-8"))
    assert summary["eligible_pairs_by_mode"]["transitive"] == 1


def test_bbox_gated_transitive_reject_is_counted(tmp_path, monkeypatch):
    report = tmp_path / "report.json"
    _write_report(
        report,
        [
            {
                "sig": "g1",
                "usd_paths": [
                    "/root/GRScenes_assets/chair/a/usd/a.usd",
                    "/root/GRScenes_assets/chair/b/usd/b.usd",
                    "/root/GRScenes_assets/chair/c/usd/c.usd",
                ],
            }
        ],
    )
    dataset_root = tmp_path / "dataset"
    (dataset_root / "GRScenes100").mkdir(parents=True)
    monkeypatch.setattr(mapping_mod, "_count_layout_asset_usage", lambda *args, **kwargs: Counter())
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_mode_index",
        lambda _: {
            ("b", "a"): "geom_only",
            ("a", "b"): "geom_only",
            ("a", "c"): "topo_filesize",
            ("c", "a"): "topo_filesize",
        },
    )
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_pair_certificate",
        lambda **kwargs: {
            "eligible": True,
            "reject_reason": None,
            "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
            "footprint_extent_delta": 0.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 0.0,
            "vertex_rmse": 0.0,
            "rmse_available": True,
            "rmse_unavailable_reason": None,
            "alternate_proof_kind": "geom_only_exact_world_proof",
            "alternate_proof_passed": True,
            "proof_source": "geom_only_exact_world_proof",
        },
    )
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_transitive_pair_certificate",
        lambda **kwargs: {
            "eligible": False,
            "reject_reason": "bbox_precheck_failed_transitive",
            "bbox_delta": {"min": [1, 0, 0], "max": [1, 0, 0], "max_abs": 1.0},
            "footprint_extent_delta": 1.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 1.0,
            "vertex_rmse": None,
            "rmse_available": False,
            "rmse_unavailable_reason": "vertex_count_mismatch",
            "alternate_proof_kind": None,
            "alternate_proof_passed": False,
            "proof_source": None,
            "transitive_effective_mode": "shape_invariant",
            "transitive_witness_uids": ["b", "a", "c"],
            "transitive_witness_modes": ["geom_only", "shape_invariant"],
        },
    )

    out_mapping = tmp_path / "filtered_mapping.json"
    out_stats = tmp_path / "filtered_mapping.stats.json"
    out_cert_jsonl = tmp_path / "pair_certificates.jsonl"
    out_cert_summary = tmp_path / "pair_certificate_summary.json"
    out_graph = tmp_path / "certified_graph.json"
    args = Namespace(
        report=str(report),
        dataset_root=str(dataset_root),
        category=None,
        out_mapping_json=str(out_mapping),
        out_stats_json=str(out_stats),
        progress_every=0,
        bbox_policy="bbox_primary_rmse_observe",
        dedup_mode="geom_only",
        out_certificate_jsonl=str(out_cert_jsonl),
        out_certificate_summary_json=str(out_cert_summary),
        out_certified_graph_json=str(out_graph),
        revoked_edges_jsonl=None,
        mode_reports_dir="/fake/modes",
    )

    assert mapping_mod._run_bbox_gated(args) == 0
    summary = json.loads(out_cert_summary.read_text(encoding="utf-8"))
    assert summary["reject_reason_counts"]["bbox_precheck_failed_transitive"] == 1
```

- [ ] **Step 2: Run the mapping tests to verify they fail**

Run:

```bash
python -m pytest tests/test_bbox_gated_mapping.py -q
```

Expected:

- FAIL because `_run_bbox_gated()` still rejects transitive pairs before the new builder can run.

- [ ] **Step 3: Implement transitive admission in the mapping builder**

Modify `scripts/c1_build_bulk_mapping_from_dedup_report.py` so transitive pairs call the dedicated builder.

```python
for old_asset in paths:
    if old_asset == initial_canonical:
        continue

    candidate_pair_count += 1
    if mode_index:
        canon_uid = _cvt._uid_from_path(str(dataset_root / initial_canonical))
        old_uid = _cvt._uid_from_path(str(dataset_root / old_asset))
        pair_mode = mode_index.get((canon_uid, old_uid))
        if pair_mode is None:
            pair_mode = "transitive"
    else:
        pair_mode = report_mode

    pair_mode_counts[pair_mode] += 1

    if pair_mode == "transitive":
        cert = _cvt.build_transitive_pair_certificate(
            old_usd=str(dataset_root / old_asset),
            canonical_usd=str(dataset_root / initial_canonical),
            group_members=[str(dataset_root / p) for p in paths],
            mode_index=mode_index,
            policy=args.bbox_policy,
        )
    else:
        cert = _cvt.build_pair_certificate(
            old_usd=str(dataset_root / old_asset),
            canonical_usd=str(dataset_root / initial_canonical),
            mode=pair_mode,
            policy=args.bbox_policy,
        )
```

Do not change the certified-graph reconstruction logic. The only behavioral change in this task is replacing the hardcoded immediate reject with a transitive certificate attempt.

- [ ] **Step 4: Run the mapping tests to verify they pass**

Run:

```bash
python -m pytest tests/test_bbox_gated_mapping.py -q
```

Expected:

- PASS
- Summary files should now count `transitive` under `pair_mode_counts`, and safe transitive rows should contribute to `eligible_pairs_by_mode`.

- [ ] **Step 5: Commit this mapping slice if commits are requested for the implementation session**

```bash
git add tests/test_bbox_gated_mapping.py scripts/c1_build_bulk_mapping_from_dedup_report.py
git commit -m "feat: admit certified transitive edges in bbox mapping"
```

### Task 3: Make Audit Certificate-Aware For Certified Transitive Pairs

**Files:**
- Modify: `scripts/placement_pairwise_compare.py:43-72`
- Modify: `scripts/placement_pairwise_compare.py:446-629`
- Modify: `scripts/placement_pairwise_compare.py:858-1039`
- Test: `tests/test_placement_pairwise_compare_bbox_gate.py:1-181`

- [ ] **Step 1: Write the failing audit tests**

Update `tests/test_placement_pairwise_compare_bbox_gate.py` with the imports and tests below.

```python
from placement_pairwise_compare import _lookup_prim_dedup_mode, compare_scene


def test_lookup_prim_mode_prefers_certificate_effective_mode():
    left_refs = ["GRScenes_assets/chair/aaa111/usd/aaa111.usd"]
    right_refs = ["GRScenes_assets/chair/bbb222/usd/bbb222.usd"]
    mode_index = {("bbb222", "aaa111"): "geom_only"}
    certificate_lookup = {
        ("bbb222", "aaa111"): {
            "pair_mode": "transitive",
            "effective_mode": "topo_filesize",
        }
    }

    assert _lookup_prim_dedup_mode(
        left_refs,
        right_refs,
        mode_index,
        certificate_lookup=certificate_lookup,
    ) == "topo_filesize"


@pxr_required
def test_certificate_lookup_makes_transitive_observe_bbox_soft_fail(tmp_path):
    left_uid = "aaa111"
    right_uid = "bbb222"
    left_asset = _make_asset(str(tmp_path / "GRScenes_assets" / "chair" / left_uid / "usd" / f"{left_uid}.usd"))
    right_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / right_uid / "usd" / f"{right_uid}.usd"),
        point_offset=(0.2, 0.0, 0.0),
    )
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
        mode_index={("zzz999", "yyy888"): "geom_only"},
        certificate_lookup={
            (right_uid, left_uid): {
                "pair_mode": "transitive",
                "effective_mode": "topo_filesize",
            }
        },
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] == 0
    assert result["ref_changed_soft_fail_count"] >= 1


@pxr_required
def test_transitive_without_certificate_lookup_stays_strict(tmp_path):
    left_uid = "aaa111"
    right_uid = "bbb222"
    left_asset = _make_asset(str(tmp_path / "GRScenes_assets" / "chair" / left_uid / "usd" / f"{left_uid}.usd"))
    right_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / right_uid / "usd" / f"{right_uid}.usd"),
        point_offset=(0.2, 0.0, 0.0),
    )
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
        mode_index={("zzz999", "yyy888"): "geom_only"},
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] >= 1
```

- [ ] **Step 2: Run the audit tests to verify they fail**

Run:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Expected:

- FAIL because `_lookup_prim_dedup_mode()` has no certificate-aware override and `compare_scene()` cannot accept certificate lookup data yet.

- [ ] **Step 3: Implement certificate-aware audit mode resolution**

Modify `scripts/placement_pairwise_compare.py` with the small helper additions below.

```python
def _load_certificate_lookup(certificate_jsonl: Optional[str]) -> Dict[Tuple[str, str], Dict[str, object]]:
    lookup: Dict[Tuple[str, str], Dict[str, object]] = {}
    if not certificate_jsonl:
        return lookup
    with open(certificate_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            row = json.loads(line)
            if not row.get("eligible"):
                continue
            old_uid = _uid_from_ref(str(row.get("old_asset") or ""))
            canon_uid = _uid_from_ref(str(row.get("canonical_asset") or ""))
            if not old_uid or not canon_uid:
                continue
            pair_mode = str(row.get("pair_mode") or row.get("mode") or "")
            effective_mode = str(row.get("transitive_effective_mode") or pair_mode)
            lookup[(canon_uid, old_uid)] = {
                "pair_mode": pair_mode,
                "effective_mode": effective_mode,
            }
    return lookup


def _lookup_prim_dedup_mode(
    left_refs: List[str],
    right_refs: List[str],
    mode_index: Optional[Dict],
    *,
    certificate_lookup: Optional[Dict[Tuple[str, str], Dict[str, object]]] = None,
) -> Optional[str]:
    if left_refs == right_refs:
        return None
    old_uid = _uid_from_ref(left_refs[0]) if left_refs else None
    canon_uid = _uid_from_ref(right_refs[0]) if right_refs else None
    if old_uid and canon_uid and certificate_lookup:
        cert = certificate_lookup.get((canon_uid, old_uid))
        if cert:
            return str(cert["effective_mode"])
    if mode_index and old_uid and canon_uid:
        return (
            mode_index.get((canon_uid, old_uid))
            or mode_index.get((old_uid, canon_uid))
            or "transitive"
        )
    return None
```

Then thread the new lookup through the scene comparison path:

```python
def compare_scene(..., mode_index: Optional[Dict] = None, certificate_lookup: Optional[Dict] = None) -> Dict[str, object]:
    ...
    prim_mode = _lookup_prim_dedup_mode(
        left_refs,
        right_refs,
        mode_index,
        certificate_lookup=certificate_lookup,
    )
```

And add the CLI plumbing:

```python
parser.add_argument(
    "--certificate-jsonl",
    default=None,
    help="Optional pair certificate JSONL for certificate-aware audit mode resolution.",
)

certificate_lookup = _load_certificate_lookup(args.certificate_jsonl)
...
compare_scene(..., mode_index=mode_index, certificate_lookup=certificate_lookup)
```

- [ ] **Step 4: Run the audit tests to verify they pass**

Run:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Expected:

- PASS
- The new certificate-aware transitive test should prove that certified transitive pairs inherit `topo_filesize` or `shape_invariant` observe semantics instead of being treated as blanket unsupported strict failures.

- [ ] **Step 5: Commit this audit slice if commits are requested for the implementation session**

```bash
git add tests/test_placement_pairwise_compare_bbox_gate.py scripts/placement_pairwise_compare.py
git commit -m "feat: make audit honor certified transitive edge semantics"
```

### Task 4: Wire Certificate JSONL Into Autorun Audit Command

**Files:**
- Modify: `scripts/c1_autorun_categories.py:54-70`
- Modify: `scripts/c1_autorun_categories.py:307-506`
- Create: `tests/test_c1_autorun_categories.py`

- [ ] **Step 1: Write the failing autorun wiring test**

Create `tests/test_c1_autorun_categories.py` with the test below.

```python
import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import c1_autorun_categories as autorun


def test_build_bbox_audit_cmd_includes_certificate_jsonl(tmp_path):
    c1_bulk_dir = tmp_path / "c1_bulk"
    plan = autorun.build_bbox_plan(
        "chair",
        c1_bulk_dir=c1_bulk_dir,
        policy_tag="bbox_primary_rmse_observe",
        out_version="v1",
        group_label="c1_v8_tier2_rollout",
    )
    changed_scene_ids = plan.apply_dir / "changed_scene_ids.json"
    args = argparse.Namespace(
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
        mode_reports_dir="check_reports/test0_rebuilt_dedup/v8_prededup",
    )

    cmd = autorun._build_bbox_audit_cmd(
        plan=plan,
        args=args,
        dataset_root=tmp_path,
        changed_scene_list_json=changed_scene_ids,
    )

    idx = cmd.index("--certificate-jsonl")
    assert cmd[idx + 1].endswith("pair_certificates.jsonl")
    assert "--mode-reports-dir" in cmd
```

- [ ] **Step 2: Run the autorun wiring test to verify it fails**

Run:

```bash
python -m pytest tests/test_c1_autorun_categories.py -q
```

Expected:

- FAIL because `_build_bbox_audit_cmd()` does not exist yet and `_run_bbox_gated()` assembles the audit command inline.

- [ ] **Step 3: Extract the audit command builder and include certificate JSONL**

Modify `scripts/c1_autorun_categories.py` as follows.

```python
def _build_bbox_audit_cmd(
    *,
    plan: BBoxCategoryPlan,
    args: argparse.Namespace,
    dataset_root: Path,
    changed_scene_list_json: Path,
) -> list[str]:
    cmd = [
        str(ISAAC_PY),
        str(SCRIPT_AUDIT.relative_to(REPO_ROOT)),
        "--left-root",
        str(dataset_root.relative_to(REPO_ROOT)) if dataset_root.is_relative_to(REPO_ROOT) else str(dataset_root),
        "--right-root",
        str(dataset_root.relative_to(REPO_ROOT)) if dataset_root.is_relative_to(REPO_ROOT) else str(dataset_root),
        "--left-mode",
        "current",
        "--right-mode",
        "current",
        "--right-layout-name",
        plan.out_name,
        "--label",
        f"{plan.category}_{args.bbox_policy}",
        "--out",
        str(plan.audit_report_json.relative_to(REPO_ROOT)) if plan.audit_report_json.is_relative_to(REPO_ROOT) else str(plan.audit_report_json),
        "--verdict-out",
        str(plan.audit_verdict_json.relative_to(REPO_ROOT)) if plan.audit_verdict_json.is_relative_to(REPO_ROOT) else str(plan.audit_verdict_json),
        "--scene-list-json",
        str(changed_scene_list_json.relative_to(REPO_ROOT)) if changed_scene_list_json.is_relative_to(REPO_ROOT) else str(changed_scene_list_json),
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
        "--certificate-jsonl",
        str(plan.certificate_jsonl.relative_to(REPO_ROOT)) if plan.certificate_jsonl.is_relative_to(REPO_ROOT) else str(plan.certificate_jsonl),
    ]
    if args.mode_reports_dir:
        cmd.extend(["--mode-reports-dir", args.mode_reports_dir])
    return cmd
```

Then replace the inline audit command assembly in `_run_bbox_gated()` with a call to `_build_bbox_audit_cmd(...)`.

- [ ] **Step 4: Run the autorun wiring test to verify it passes**

Run:

```bash
python -m pytest tests/test_c1_autorun_categories.py -q
```

Expected:

- PASS
- The generated audit command should now include the same `pair_certificates.jsonl` that the apply step already uses.

- [ ] **Step 5: Commit this autorun slice if commits are requested for the implementation session**

```bash
git add tests/test_c1_autorun_categories.py scripts/c1_autorun_categories.py
git commit -m "feat: pass certificate metadata into bbox audit runs"
```

### Task 5: Document The Feature And Run Final Verification

**Files:**
- Create: `docs/records/changes/2026-04-10_transitive_bbox_gating.md`
- Modify: `docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md`

- [ ] **Step 1: Find the docs that reference the changed code**

Run:

```bash
python scripts/doc_manager.py --find-refs scripts/compute_vertex_transform.py
python scripts/doc_manager.py --find-refs scripts/c1_build_bulk_mapping_from_dedup_report.py
python scripts/doc_manager.py --find-refs scripts/placement_pairwise_compare.py
python scripts/doc_manager.py --find-refs scripts/c1_autorun_categories.py
```

Expected:

- Output listing the existing rollout and change records that should be updated after the feature lands.

- [ ] **Step 2: Write the feature change note**

Create `docs/records/changes/2026-04-10_transitive_bbox_gating.md` with the content below.

```markdown
---
title: "Transitive BBox Gating"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/c1_autorun_categories.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: implemented
---

# Transitive BBox Gating

## Summary

The bbox-gated dedup pipeline now supports `transitive` edges by treating the
transitive path as a candidate-`V` witness and requiring direct endpoint
verification before the edge is certified.

## What Changed

- deterministic witness-path selection was added to `compute_vertex_transform.py`
- `c1_build_bulk_mapping_from_dedup_report.py` now calls the transitive
  certificate builder instead of rejecting `transitive_not_supported`
- `placement_pairwise_compare.py` now reads certificate metadata so certified
  transitive edges inherit their effective bbox semantics during audit
- `c1_autorun_categories.py` now passes `pair_certificates.jsonl` into audit

## Verification

- `python -m pytest tests/test_compute_vertex_transform.py tests/test_bbox_gated_mapping.py tests/test_c1_autorun_categories.py -q`
- `./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q`

## Follow-up

The code path is implemented and tested, but rollout metrics only change after a
new report-to-rollout rerun is executed.
```

- [ ] **Step 3: Update the rollout status note to mention the new capability**

Append this short note to `docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md` in the current-summary section.

```markdown
- Code support for certifying and auditing `transitive` edges is now landed.
- The reject bucket `transitive_not_supported` in the current totals remains a
  historical artifact of the last recorded rollout; it will only change after a
  fresh rerun using the new code path.
```

- [ ] **Step 4: Run the full targeted verification suite**

Run:

```bash
python -m pytest tests/test_compute_vertex_transform.py tests/test_bbox_gated_mapping.py tests/test_c1_autorun_categories.py -q
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
python scripts/doc_manager.py --validate
python scripts/doc_manager.py --gen-index
```

Expected:

- All selected tests PASS with `0` failures.
- Documentation validation succeeds.
- `docs/INDEX.md` regenerates cleanly.

- [ ] **Step 5: Commit the completed feature if commits are requested for the implementation session**

```bash
git add \
  scripts/compute_vertex_transform.py \
  scripts/c1_build_bulk_mapping_from_dedup_report.py \
  scripts/placement_pairwise_compare.py \
  scripts/c1_autorun_categories.py \
  tests/test_compute_vertex_transform.py \
  tests/test_bbox_gated_mapping.py \
  tests/test_placement_pairwise_compare_bbox_gate.py \
  tests/test_c1_autorun_categories.py \
  docs/records/changes/2026-04-10_transitive_bbox_gating.md \
  docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md \
  docs/INDEX.md
git commit -m "feat: support transitive edges in bbox-gated dedup"
```

## Self-Review Checklist

### Spec Coverage

- deterministic witness path selection: covered in Task 1
- transitive effective mode: covered in Task 1 and Task 3
- endpoint verification for transitive edges: covered in Task 1 and Task 2
- certificate metadata storage: covered in Task 1 and Task 2
- audit consuming certificate semantics: covered in Task 3
- autorun passing certificate jsonl to audit: covered in Task 4
- docs and verification: covered in Task 5

### Placeholder Scan

- No `TODO`, `TBD`, or deferred “implement later” instructions remain.
- Every code-changing step includes concrete code snippets.
- Every verification step includes concrete commands.

### Type And Naming Consistency

- `find_transitive_witness` is the single witness-resolution entry point.
- `build_transitive_pair_certificate` is the single transitive cert builder.
- `transitive_effective_mode` is the field used by both cert and audit.
- `certificate_lookup` is the audit-side structure threaded into `compare_scene`.
