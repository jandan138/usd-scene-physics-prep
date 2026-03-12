#!/usr/bin/env python3
"""Unit tests for union merge functions in union_dedup_reports.py.

Tests the core merge logic:
- normalize_path: path normalization
- union_merge: 2-way merge with transitive closure
- union_merge_n: N-way merge with arbitrary reports

Run with: python -m pytest tests/test_union_merge.py -v
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.union_dedup_reports import (
    normalize_path,
    extract_removable_and_groups,
    union_merge,
    union_merge_n,
    UnionFind,
)


# ---------------------------------------------------------------------------
# Helpers to build fake reports
# ---------------------------------------------------------------------------

def _make_report(mode, groups, asset_count=100):
    """Build a minimal dedup report dict.

    Args:
        mode: e.g. "geom_only", "shape_invariant", "topo_filesize"
        groups: list of lists; each inner list is [representative, *removables]
        asset_count: total asset count for meta
    """
    duplicates = []
    for i, paths in enumerate(groups):
        duplicates.append({
            "sig": f"{mode}_{i}",
            "count": len(paths),
            "usd_paths": paths,
        })
    return {
        "meta": {
            "dataset": "test_dataset",
            "mode": mode,
            "asset_usd_count": asset_count,
        },
        "duplicates": duplicates,
        "assets": [{"path": f"asset_{j}"} for j in range(asset_count)],
        "errors": [],
    }


# ---------------------------------------------------------------------------
# test_normalize_path
# ---------------------------------------------------------------------------

class TestNormalizePath:
    def test_strips_dot_slash(self):
        assert normalize_path("./foo/bar") == "foo/bar"

    def test_no_change_without_prefix(self):
        assert normalize_path("foo/bar") == "foo/bar"

    def test_only_dot_slash(self):
        assert normalize_path("./") == ""

    def test_double_dot_slash_not_stripped(self):
        assert normalize_path("../foo") == "../foo"

    def test_absolute_path_to_relative(self):
        """Absolute path with GRScenes_assets marker is converted to relative."""
        abs_path = "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1-normalized/GRScenes_assets/book/abc123/usd/abc123.usd"
        expected = "GRScenes-test1-normalized/GRScenes_assets/book/abc123/usd/abc123.usd"
        assert normalize_path(abs_path) == expected

    def test_relative_path_unchanged(self):
        """Relative path with GRScenes_assets stays the same."""
        rel_path = "GRScenes-test1-normalized/GRScenes_assets/book/abc123/usd/abc123.usd"
        assert normalize_path(rel_path) == rel_path

    def test_abs_and_rel_normalize_to_same(self):
        """Absolute and relative forms of the same asset normalize identically."""
        abs_path = "/some/deep/path/MyDataset/GRScenes_assets/bottle/xyz/usd/xyz.usd"
        rel_path = "MyDataset/GRScenes_assets/bottle/xyz/usd/xyz.usd"
        assert normalize_path(abs_path) == normalize_path(rel_path)

    def test_backslash_normalized(self):
        p = "GRScenes-test1-normalized\\GRScenes_assets\\book\\abc\\usd\\abc.usd"
        assert "/" in normalize_path(p)
        assert "\\" not in normalize_path(p)


# ---------------------------------------------------------------------------
# test_union_merge_basic
# ---------------------------------------------------------------------------

class TestUnionMergeBasic:
    def test_non_overlapping_groups(self):
        """Two reports with non-overlapping groups -> both preserved."""
        geom = _make_report("geom_only", [["A", "B"]])
        shape = _make_report("shape_invariant", [["C", "D"]])
        result = union_merge(geom, shape)
        removable = {p for g in result["duplicates"]
                     for p in g["usd_paths"][1:]}
        assert "B" in removable
        assert "D" in removable
        assert result["meta"]["union_removable"] == 2

    def test_overlapping_groups(self):
        """Two reports that share an asset -> groups merged."""
        geom = _make_report("geom_only", [["A", "B"]])
        shape = _make_report("shape_invariant", [["B", "C"]])
        result = union_merge(geom, shape)
        # A, B, C should be in one group
        assert len(result["duplicates"]) == 1
        group = result["duplicates"][0]
        assert set(group["usd_paths"]) == {"A", "B", "C"}


# ---------------------------------------------------------------------------
# test_union_merge_transitive_closure
# ---------------------------------------------------------------------------

class TestUnionMergeTransitiveClosure:
    def test_shape_group_spanning_two_geom_groups(self):
        """Shape group {X, A, C} links geom groups {A, B} and {C, D} -> single merged group."""
        geom = _make_report("geom_only", [["A", "B"], ["C", "D"]])
        shape = _make_report("shape_invariant", [["X", "A", "C"]])
        result = union_merge(geom, shape)
        # All of A, B, C, D, X should end up in one component
        all_paths = set()
        for g in result["duplicates"]:
            all_paths.update(g["usd_paths"])
        assert {"A", "B", "C", "D", "X"} == all_paths
        # Should be a single group
        assert len(result["duplicates"]) == 1


# ---------------------------------------------------------------------------
# test_union_merge_no_canonical_conflicts
# ---------------------------------------------------------------------------

class TestUnionMergeNoCanonicalConflicts:
    def test_representative_not_in_removable(self):
        """After merge, the representative of each group must not be removable."""
        geom = _make_report("geom_only", [["A", "B", "C"]])
        shape = _make_report("shape_invariant", [["A", "D", "E"]])
        result = union_merge(geom, shape)
        for g in result["duplicates"]:
            rep = g["usd_paths"][0]
            removables = set(g["usd_paths"][1:])
            assert rep not in removables


# ---------------------------------------------------------------------------
# test_union_merge_n_three_reports
# ---------------------------------------------------------------------------

class TestUnionMergeNThreeReports:
    def test_three_reports_merge(self):
        """Three reports merge correctly via union-find."""
        r1 = _make_report("geom_only", [["A", "B"]])
        r2 = _make_report("shape_invariant", [["C", "D"]])
        r3 = _make_report("topo_filesize", [["B", "C"]])
        result = union_merge_n([r1, r2, r3])

        # B links r1 and r3; C links r3 and r2; so all {A,B,C,D} in one group
        assert len(result["duplicates"]) == 1
        paths = set(result["duplicates"][0]["usd_paths"])
        assert paths == {"A", "B", "C", "D"}

        meta = result["meta"]
        assert set(meta["source_modes"]) == {"geom_only", "shape_invariant", "topo_filesize"}
        assert "per_mode_stats" in meta
        assert meta["union_removable"] == 3  # one keeper, three removable

    def test_three_disjoint_reports(self):
        """Three non-overlapping reports -> three separate groups."""
        r1 = _make_report("geom_only", [["A", "B"]])
        r2 = _make_report("shape_invariant", [["C", "D"]])
        r3 = _make_report("topo_filesize", [["E", "F"]])
        result = union_merge_n([r1, r2, r3])
        assert len(result["duplicates"]) == 3
        assert result["meta"]["union_removable"] == 3


# ---------------------------------------------------------------------------
# test_union_merge_n_empty_report
# ---------------------------------------------------------------------------

class TestUnionMergeNEmptyReport:
    def test_empty_report_no_crash(self):
        """An empty report mixed with a real one should not break merge."""
        r1 = _make_report("geom_only", [["A", "B"]])
        r_empty = _make_report("shape_invariant", [], asset_count=0)
        result = union_merge_n([r1, r_empty])
        assert len(result["duplicates"]) == 1
        assert result["meta"]["union_removable"] == 1

    def test_all_empty_reports(self):
        """All empty reports -> no duplicates."""
        r1 = _make_report("geom_only", [], asset_count=0)
        r2 = _make_report("shape_invariant", [], asset_count=0)
        result = union_merge_n([r1, r2])
        assert len(result["duplicates"]) == 0
        assert result["meta"]["union_removable"] == 0


# ---------------------------------------------------------------------------
# test_union_merge_n_single_report
# ---------------------------------------------------------------------------

class TestUnionMergeNSingleReport:
    def test_single_report_preserves_groups(self):
        """Single report passed to union_merge_n returns equivalent groups."""
        r1 = _make_report("geom_only", [["A", "B", "C"], ["D", "E"]])
        result = union_merge_n([r1])
        assert len(result["duplicates"]) == 2
        assert result["meta"]["union_removable"] == 3  # B, C, E

        # Check groups match (order may differ)
        all_groups = {frozenset(g["usd_paths"]) for g in result["duplicates"]}
        assert frozenset({"A", "B", "C"}) in all_groups
        assert frozenset({"D", "E"}) in all_groups


# ---------------------------------------------------------------------------
# test_transitive_canonical_conflict (the bug this fix addresses)
# ---------------------------------------------------------------------------

class TestTransitiveCanonicalConflict:
    """Test that abs/rel path mixing doesn't cause transitive canonical conflicts.

    This is the exact scenario that caused C1 soft-delete to abort at book (7/79):
    - geom_only reports use relative paths
    - shape_invariant reports use absolute paths
    - Union merge must normalize both to the same form so union-find correctly
      merges groups that share the same physical asset.
    """

    def _abs(self, rel):
        """Convert relative path to absolute (simulating shape_invariant report style)."""
        return f"/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/{rel}"

    def test_same_asset_abs_rel_merged_into_one_group(self):
        """Same asset in abs and rel form must merge into a single group."""
        asset_A_rel = "DS/GRScenes_assets/book/aaaa/usd/aaaa.usd"
        asset_A_abs = self._abs(asset_A_rel)
        asset_B_rel = "DS/GRScenes_assets/book/bbbb/usd/bbbb.usd"
        asset_C_rel = "DS/GRScenes_assets/book/cccc/usd/cccc.usd"
        asset_C_abs = self._abs(asset_C_rel)

        # geom_only: group {A_rel, B_rel} with A as canonical
        geom = _make_report("geom_only", [[asset_A_rel, asset_B_rel]])
        # shape_invariant: group {A_abs, C_abs} with A as canonical
        shape = _make_report("shape_invariant", [[asset_A_abs, asset_C_abs]])

        result = union_merge(geom, shape)

        # After normalization, A_rel and A_abs are the same -> single group {A, B, C}
        assert len(result["duplicates"]) == 1
        group = result["duplicates"][0]
        normalized_paths = {normalize_path(p) for p in group["usd_paths"]}
        expected = {normalize_path(asset_A_rel), normalize_path(asset_B_rel), normalize_path(asset_C_rel)}
        assert normalized_paths == expected

        # The representative must not appear in the removable set
        rep = group["usd_paths"][0]
        removables = set(group["usd_paths"][1:])
        assert rep not in removables, f"Representative {rep} is also removable!"

    def test_no_canonical_in_old_after_merge(self):
        """Build old->canonical mapping from merge result: no asset should be both."""
        asset_X_rel = "DS/GRScenes_assets/book/xxxx/usd/xxxx.usd"
        asset_X_abs = self._abs(asset_X_rel)
        asset_Y = "DS/GRScenes_assets/book/yyyy/usd/yyyy.usd"
        asset_Z = "DS/GRScenes_assets/book/zzzz/usd/zzzz.usd"
        asset_Z_abs = self._abs(asset_Z)

        # geom_only: {X_rel, Y} -> X is canonical
        geom = _make_report("geom_only", [[asset_X_rel, asset_Y]])
        # shape_invariant: {X_abs, Z_abs} -> X is canonical
        shape = _make_report("shape_invariant", [[asset_X_abs, asset_Z_abs]])

        result = union_merge(geom, shape)

        # Simulate mapping builder: paths[0] = canonical, paths[1:] = old
        mapping = {}
        for g in result["duplicates"]:
            canonical = g["usd_paths"][0]
            for p in g["usd_paths"][1:]:
                mapping[p] = canonical

        # No asset should be both key (old) and value (canonical)
        canonical_set = set(mapping.values())
        conflict = set(mapping.keys()) & canonical_set
        assert len(conflict) == 0, f"Transitive canonical conflicts found: {conflict}"

    def test_n_way_merge_abs_rel_mixed(self):
        """N-way merge with 3 reports using mixed path forms."""
        base = "DS/GRScenes_assets/bottle"
        a_rel = f"{base}/aaa/usd/aaa.usd"
        a_abs = self._abs(a_rel)
        b_rel = f"{base}/bbb/usd/bbb.usd"
        c_rel = f"{base}/ccc/usd/ccc.usd"
        c_abs = self._abs(c_rel)
        d_rel = f"{base}/ddd/usd/ddd.usd"

        r1 = _make_report("geom_only", [[a_rel, b_rel]])
        r2 = _make_report("shape_invariant", [[a_abs, c_abs]])
        r3 = _make_report("topo_filesize", [[c_rel, d_rel]])

        result = union_merge_n([r1, r2, r3])

        # a links r1 and r2; c links r2 and r3; so {a,b,c,d} in one group
        assert len(result["duplicates"]) == 1
        paths = set(result["duplicates"][0]["usd_paths"])
        expected = {normalize_path(p) for p in [a_rel, b_rel, c_rel, d_rel]}
        assert paths == expected
