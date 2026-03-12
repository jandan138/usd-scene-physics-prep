#!/usr/bin/env python3
"""Unit tests for topo_filesize dedup mode helper functions.

Tests the core functions used by topo_filesize dedup mode:
- _filesize_match: File size tolerance comparison
- _topo_filesize_merge: End-to-end merge algorithm

Run with: python -m pytest tests/test_topo_filesize.py -v
"""

import sys
import os
import pytest

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.report_asset_mesh_dedup import (
    _filesize_match,
    _topo_filesize_merge,
    AssetRecord,
)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _make_record(usd_path, topo_inv_sig, usd_size=None, glb_size=None, mesh_count=1):
    """Create a minimal AssetRecord for testing merge logic."""
    return AssetRecord(
        usd_path=usd_path,
        category="test",
        uid="test_uid",
        mesh_count=mesh_count,
        asset_geom_sig_hex="",
        asset_scale_sig_hex="",
        asset_full_matrix_sig_hex="",
        asset_topo_sig_hex="",
        meshes=[],
        usd_file_size=usd_size,
        glb_file_size=glb_size,
        asset_topo_invariant_sig_hex=topo_inv_sig,
    )


# ===========================================================================
# _filesize_match tests
# ===========================================================================

class TestFilesizeMatch:
    """Tests for _filesize_match function."""

    def test_within_tolerance(self):
        """Sizes 100 and 101 within 2% tolerance -> True."""
        assert _filesize_match(100, 101, 0.02) is True

    def test_outside_tolerance(self):
        """Sizes 100 and 120 outside 2% tolerance -> False."""
        assert _filesize_match(100, 120, 0.02) is False

    def test_none_a(self):
        """None first arg -> False."""
        assert _filesize_match(None, 100, 0.02) is False

    def test_none_b(self):
        """None second arg -> False."""
        assert _filesize_match(100, None, 0.02) is False

    def test_both_none(self):
        """Both None -> False."""
        assert _filesize_match(None, None, 0.02) is False

    def test_both_zero(self):
        """Both zero -> True."""
        assert _filesize_match(0, 0, 0.02) is True

    def test_exact_equal(self):
        """Identical sizes -> True."""
        assert _filesize_match(310080, 310080, 0.02) is True

    def test_bottle_triplet_glb(self):
        """Real bottle triplet GLB sizes (310084 vs 310080) within 2% -> True."""
        assert _filesize_match(310084, 310080, 0.02) is True

    def test_bottle_triplet_usd(self):
        """Real bottle triplet USD sizes (103555 vs 103819) within 2% -> True."""
        assert _filesize_match(103555, 103819, 0.02) is True

    def test_edge_case_tolerance_boundary(self):
        """Exact boundary: diff/max == tolerance should match (<=)."""
        # 100 and 102: diff=2, max=102, ratio=2/102 ~= 0.01961
        assert _filesize_match(100, 102, 0.02) is True
        # 100 and 103: diff=3, max=103, ratio=3/103 ~= 0.02913
        assert _filesize_match(100, 103, 0.02) is False
        # Exact boundary: size_a=98, size_b=100, diff=2, max=100, ratio=0.02
        assert _filesize_match(98, 100, 0.02) is True


# ===========================================================================
# _topo_filesize_merge tests
# ===========================================================================

class TestTopoFilesizeMerge:
    """Tests for _topo_filesize_merge function."""

    def test_basic_grouping(self):
        """3 records with same topo_inv sig and close GLB sizes -> one group."""
        records = [
            _make_record("/a.usd", "sig_aaa", glb_size=310080),
            _make_record("/b.usd", "sig_aaa", glb_size=310084),
            _make_record("/c.usd", "sig_aaa", glb_size=310082),
        ]
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 1
        group = list(result.values())[0]
        assert sorted(group) == ["/a.usd", "/b.usd", "/c.usd"]

    def test_different_sig_no_group(self):
        """2 records with different topo_inv sig -> no groups."""
        records = [
            _make_record("/a.usd", "sig_aaa", glb_size=310080),
            _make_record("/b.usd", "sig_bbb", glb_size=310084),
        ]
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0

    def test_singleton_no_group(self):
        """1 record -> no groups (singletons dropped)."""
        records = [
            _make_record("/a.usd", "sig_aaa", glb_size=310080),
        ]
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0

    def test_glb_size_mismatch(self):
        """Same sig but GLB sizes too far apart -> not grouped."""
        records = [
            _make_record("/a.usd", "sig_aaa", glb_size=100000),
            _make_record("/b.usd", "sig_aaa", glb_size=200000),
        ]
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0

    def test_usd_fallback_no_glb(self):
        """Both missing GLB, close USD sizes -> grouped via USD fallback."""
        records = [
            _make_record("/a.usd", "sig_aaa", usd_size=103555, glb_size=None),
            _make_record("/b.usd", "sig_aaa", usd_size=103600, glb_size=None),
        ]
        # USD fallback uses tolerance/2 = 0.01
        # diff=45, max=103600, ratio ~= 0.000434 < 0.01
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 1
        group = list(result.values())[0]
        assert sorted(group) == ["/a.usd", "/b.usd"]

    def test_usd_fallback_too_far(self):
        """Both missing GLB, USD sizes too far apart -> not grouped."""
        records = [
            _make_record("/a.usd", "sig_aaa", usd_size=100000, glb_size=None),
            _make_record("/b.usd", "sig_aaa", usd_size=102000, glb_size=None),
        ]
        # USD fallback tolerance = 0.02/2 = 0.01
        # diff=2000, max=102000, ratio ~= 0.0196 > 0.01
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0

    def test_mixed_glb_present(self):
        """One has GLB, one doesn't -> GLB match fails (None), no USD fallback."""
        records = [
            _make_record("/a.usd", "sig_aaa", usd_size=103555, glb_size=310080),
            _make_record("/b.usd", "sig_aaa", usd_size=103600, glb_size=None),
        ]
        # _filesize_match(310080, None, ...) -> False
        # USD fallback only triggers when BOTH have None GLB
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0

    def test_empty_records(self):
        """Empty list -> empty dict."""
        result = _topo_filesize_merge([], filesize_tolerance=0.02)
        assert result == {}

    def test_zero_mesh_count_skipped(self):
        """Records with mesh_count=0 are skipped."""
        records = [
            _make_record("/a.usd", "sig_aaa", glb_size=310080, mesh_count=0),
            _make_record("/b.usd", "sig_aaa", glb_size=310084, mesh_count=0),
        ]
        result = _topo_filesize_merge(records, filesize_tolerance=0.02)
        assert len(result) == 0
