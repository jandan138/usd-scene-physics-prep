#!/usr/bin/env python3
"""生成 GRScenes 资产 Mesh 去重报告（仅分析，不修改数据）。

你可以把它理解为：给每个资产 USD 计算一个“签名（hash）”，再按签名分组。
同一个组里的资产，就是“高度疑似重复/冗余”的候选。

本脚本会生成三份报告（同一批资产，三种口径）：
    1) geom_only   : 只看几何（忽略 transform）
    2) scale_only  : 几何 + scale（忽略平移/旋转；scale 不同视为不同资产）
    3) full_matrix : 几何 + world 4×4 矩阵（平移/旋转/缩放都敏感）

推荐用 Isaac Sim 自带 python 跑（确保 pxr 可用）：
    ./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
        --assets-root GRScenes-test1/GRScenes_assets \
        --out-dir check_reports \
        --dataset test1

进度与日志（适合后台运行）：
- `check_reports/<dataset>_asset_mesh_dedup_progress.json`：进度快照（配合 watch）
- `check_reports/<dataset>_asset_mesh_dedup_progress.jsonl`：进度历史（逐条）
- stdout/stderr 建议重定向到 log 文件。

阅读代码时，若你需要“逐行中文注释版”，请看同目录：
- `scripts/report_asset_mesh_dedup_zh_annotated.py`

实现要点（简述）：
- 读取 `UsdGeom.Mesh` 的 points / 拓扑 / normals / UV(st) 等，构建几何签名。
- 再叠加 transform（scale-only 或 full-matrix），构建更严格的签名。
- 将一个资产里所有 mesh 的签名排序聚合，得到 asset 级签名。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdShade

SHAPE_DESCRIPTOR_EPS = 0.01


@dataclass(frozen=True)
class MeshSig:
    prim_path: str
    geom_sig_hex: str
    scale_sig_hex: str
    full_matrix_sig_hex: str
    topo_sig_hex: str  # topology-only hash (no vertex coords)
    vertex_count: int
    face_count: int
    has_normals: bool
    has_st: bool
    shape_invariant_sig_hex: str = ""
    shape_descriptor_key: str = ""
    material_binding: Optional[str] = None


@dataclass(frozen=True)
class TopoInvariantDescriptor:
    """Per-mesh topology invariant (order-independent)."""
    vertex_count: int
    face_count: int
    material_binding: Optional[str]  # bound material path relative to asset root


@dataclass(frozen=True)
class AssetTopoInvariant:
    """Asset-level topology invariant signature."""
    mesh_count: int
    mesh_descriptors: Tuple[TopoInvariantDescriptor, ...]  # sorted
    sig_hex: str  # SHA256 of the above


@dataclass(frozen=True)
class AssetRecord:
    usd_path: str
    category: Optional[str]
    uid: Optional[str]
    mesh_count: int
    asset_geom_sig_hex: str
    asset_scale_sig_hex: str
    asset_full_matrix_sig_hex: str
    asset_topo_sig_hex: str  # topology-only hash (for tolerance merge)
    meshes: List[MeshSig]
    asset_shape_invariant_sig_hex: str = ""
    asset_shape_descriptor_key: str = ""
    usd_file_size: Optional[int] = None
    glb_file_size: Optional[int] = None
    asset_topo_invariant_sig_hex: str = ""


@dataclass(frozen=True)
class ShapeDescriptor:
    """Shape descriptor for shape-invariant dedup pre-filtering.

    Fields:
        vertex_count: Number of vertices in mesh
        face_count: Number of faces in mesh
        aspect_ratio_hash: Quantized sorted bbox aspect ratios (16 hex chars)
    """
    vertex_count: int
    face_count: int
    aspect_ratio_hash: str


def _sha256_init(tag: str) -> "hashlib._Hash":
    h = hashlib.sha256()
    h.update(tag.encode("utf-8"))
    h.update(b"\0")
    return h


def _hash_update_str(h: "hashlib._Hash", s: str) -> None:
    data = s.encode("utf-8")
    h.update(struct.pack("<I", len(data)))
    h.update(data)


def _hash_update_token(h: "hashlib._Hash", token: Optional[str]) -> None:
    _hash_update_str(h, "" if token is None else str(token))


def _hash_update_ints(h: "hashlib._Hash", values: Sequence[int]) -> None:
    h.update(struct.pack("<I", len(values)))
    for v in values:
        h.update(struct.pack("<i", int(v)))


def _quantize(v: float, eps: float) -> float:
    if eps <= 0:
        return float(v)
    return round(float(v) / eps) * eps


def _hash_update_floats(h: "hashlib._Hash", values: Sequence[float], *, eps: float) -> None:
    h.update(struct.pack("<I", len(values)))
    for v in values:
        h.update(struct.pack("<d", _quantize(float(v), eps)))


def _matrix_to_row_major16(m: Gf.Matrix4d) -> List[float]:
    # Gf.Matrix4d is indexable as m[row][col]
    return [float(m[r][c]) for r in range(4) for c in range(4)]


def _matrix_scale_xyz(m: Gf.Matrix4d) -> Tuple[float, float, float]:
    # Use column vectors of the 3x3 part: scale = ||col||
    # Handles typical TRS matrices; shear is folded into the scale.
    c0 = (float(m[0][0]), float(m[1][0]), float(m[2][0]))
    c1 = (float(m[0][1]), float(m[1][1]), float(m[2][1]))
    c2 = (float(m[0][2]), float(m[1][2]), float(m[2][2]))

    def _len3(v: Tuple[float, float, float]) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    return (_len3(c0), _len3(c1), _len3(c2))


def _get_st_primvar(mesh_prim: Usd.Prim) -> Optional[UsdGeom.Primvar]:
    pv_api = UsdGeom.PrimvarsAPI(mesh_prim)
    pv = pv_api.GetPrimvar("st")
    if not pv or not pv.HasValue():
        return None
    return pv


# ---------------------------------------------------------------------------
# Shape-invariant dedup helper functions
# ---------------------------------------------------------------------------

def _normalize_to_unit_bbox(points: Sequence) -> np.ndarray:
    """Normalize vertices to unit bounding box (preserving aspect ratio).

    Args:
        points: Sequence of vertices (each vertex is a 3-element array-like).

    Returns:
        np.ndarray of shape (N, 3) with vertices normalized to [0, ~1] range.
        Uses uniform scale (max extent) to preserve aspect ratio.
    """
    pts = np.array(points, dtype=np.float64)  # shape (N, 3)
    if len(pts) == 0:
        return pts

    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    extent = bbox_max - bbox_min

    # Use max extent for uniform scale (preserves aspect ratio)
    # Clamp to 1e-10 to avoid division by zero on degenerate meshes
    scale = max(float(extent.max()), 1e-10)

    return (pts - bbox_min) / scale


def _compute_shape_descriptor(
    points: Sequence,
    face_count: int,
    *,
    eps: float = 0.01,
) -> ShapeDescriptor:
    """Compute shape descriptor for shape-invariant dedup pre-filtering.

    Args:
        points: Sequence of vertices.
        face_count: Number of faces.
        eps: Quantization epsilon for aspect ratio hashing.

    Returns:
        ShapeDescriptor with vertex count, face count, and aspect ratio hash.
    """
    vertex_count = len(points)

    # Compute aspect ratios from normalized bounding box
    pts = np.array(points, dtype=np.float64)
    if len(pts) == 0:
        aspect_ratio_hash = "0" * 16
    else:
        bbox_min = pts.min(axis=0)
        bbox_max = pts.max(axis=0)
        extent = bbox_max - bbox_min

        # Normalize to longest axis, sort descending
        max_ext = max(float(extent.max()), 1e-10)
        aspect_ratios = sorted([float(e) / max_ext for e in extent], reverse=True)

        # Quantize and hash
        quantized = tuple(_quantize(ar, eps) for ar in aspect_ratios)
        h = _sha256_init("aspect_ratio_v1")
        for ar in quantized:
            h.update(struct.pack("<d", ar))
        aspect_ratio_hash = h.hexdigest()[:16]

    return ShapeDescriptor(
        vertex_count=vertex_count,
        face_count=face_count,
        aspect_ratio_hash=aspect_ratio_hash,
    )


def _shape_invariant_mesh_sig(normalized_points: np.ndarray, face_count: int, eps: float = 0.01) -> str:
    """Compute shape-invariant mesh signature from pre-normalized points.

    Assumes points are already normalized to unit bounding box (caller responsibility).
    Rounds coordinates to eps precision, lexicographically sorts, then SHA256 hashes.

    Args:
        normalized_points: np.ndarray of shape (N, 3), already normalized.
        face_count: Number of faces in the mesh.
        eps: Quantization epsilon for coordinate rounding.

    Returns:
        64-char hex SHA256 digest.
    """
    pts = np.array(normalized_points, dtype=np.float64)
    if len(pts) == 0:
        h = hashlib.sha256(f"empty_{face_count}".encode("utf-8"))
        return h.hexdigest()

    # Round to eps precision
    rounded = np.round(pts / eps) * eps

    # Lexicographic sort
    sorted_pts = rounded[np.lexsort(rounded.T[::-1])]

    # Hash sorted points + face_count
    h = hashlib.sha256()
    h.update(sorted_pts.tobytes())
    h.update(struct.pack("<I", face_count))
    return h.hexdigest()


def _hausdorff_distance(
    pts_a: np.ndarray,
    pts_b: np.ndarray,
) -> float:
    """Compute bidirectional Hausdorff distance between two point sets.

    Assumes both point sets are normalized to unit bounding box.

    Args:
        pts_a: Array of shape (N, 3), normalized vertex positions.
        pts_b: Array of shape (M, 3), normalized vertex positions.

    Returns:
        Hausdorff distance (max of min distances both directions).
        Returns float('inf') if shapes are incompatible (different vertex counts).
    """
    if len(pts_a) == len(pts_b) and len(pts_a) > 0:
        # Fast path: same vertex count, use lexicographic sort + pairing
        sorted_a = pts_a[np.lexsort(pts_a.T[::-1])]
        sorted_b = pts_b[np.lexsort(pts_b.T[::-1])]
        distances = np.linalg.norm(sorted_a - sorted_b, axis=1)
        return float(np.max(distances))

    if len(pts_a) == 0 or len(pts_b) == 0:
        return float("inf")

    # Slow path: use KD-tree for bidirectional Hausdorff
    try:
        from scipy.spatial import KDTree
        tree_b = KDTree(pts_b)
        d_ab = tree_b.query(pts_a)[0].max()  # max of min distances A→B

        tree_a = KDTree(pts_a)
        d_ba = tree_a.query(pts_b)[0].max()  # max of min distances B→A

        return float(max(d_ab, d_ba))
    except ImportError:
        # Fallback: O(V^2) brute force if scipy unavailable
        max_dist = 0.0
        for pa in pts_a:
            min_dist = float("inf")
            for pb in pts_b:
                d = float(np.linalg.norm(pa - pb))
                if d < min_dist:
                    min_dist = d
            if min_dist > max_dist:
                max_dist = min_dist

        for pb in pts_b:
            min_dist = float("inf")
            for pa in pts_a:
                d = float(np.linalg.norm(pb - pa))
                if d < min_dist:
                    min_dist = d
            if min_dist > max_dist:
                max_dist = min_dist

        return max_dist


def _compute_mesh_sigs(
    mesh: UsdGeom.Mesh,
    xform_cache: UsdGeom.XformCache,
    *,
    float_eps: float,
) -> MeshSig:
    prim = mesh.GetPrim()

    # --- Geometry signature (ignores transforms)
    h_geom = _sha256_init("mesh_geom_v1")

    points = mesh.GetPointsAttr().Get() or []
    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get() or []
    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get() or []

    _hash_update_ints(h_geom, face_vertex_counts)
    _hash_update_ints(h_geom, face_vertex_indices)

    h_geom.update(struct.pack("<I", len(points)))
    for p in points:
        # p is typically Gf.Vec3f/Vec3d
        _hash_update_floats(h_geom, (float(p[0]), float(p[1]), float(p[2])), eps=float_eps)

    subdivision_scheme = mesh.GetSubdivisionSchemeAttr().Get()
    _hash_update_token(h_geom, subdivision_scheme)

    double_sided = prim.GetAttribute("doubleSided").Get()
    if double_sided is None:
        double_sided = False
    h_geom.update(struct.pack("<?", bool(double_sided)))

    has_normals = False
    normals = mesh.GetNormalsAttr().Get() or []
    if normals:
        has_normals = True
        normals_interp = None
        if hasattr(mesh, "GetNormalsInterpolation"):
            try:
                normals_interp = mesh.GetNormalsInterpolation()
            except Exception:
                normals_interp = None
        _hash_update_token(h_geom, normals_interp)
        h_geom.update(struct.pack("<I", len(normals)))
        for n in normals:
            _hash_update_floats(h_geom, (float(n[0]), float(n[1]), float(n[2])), eps=float_eps)
    else:
        _hash_update_token(h_geom, "<no_normals>")

    has_st = False
    st_pv = _get_st_primvar(prim)
    if st_pv is not None:
        has_st = True
        _hash_update_token(h_geom, st_pv.GetInterpolation())
        _hash_update_ints(h_geom, [int(st_pv.GetElementSize())])
        st_vals = st_pv.Get() or []
        h_geom.update(struct.pack("<I", len(st_vals)))
        for uv in st_vals:
            _hash_update_floats(h_geom, (float(uv[0]), float(uv[1])), eps=float_eps)
        st_indices = []
        try:
            if st_pv.IsIndexed():
                st_indices = st_pv.GetIndices() or []
        except Exception:
            st_indices = []
        _hash_update_ints(h_geom, st_indices)
    else:
        _hash_update_token(h_geom, "<no_st>")

    geom_sig_hex = h_geom.hexdigest()

    # --- Topology-only signature (excludes vertex/normal coordinate values)
    # Used by --merge-tolerance to pre-group assets by topology before
    # doing pairwise vertex comparison.  Includes face counts, indices,
    # subdivision scheme, doubleSided, normal/UV presence and counts,
    # but NOT the actual coordinate values.
    h_topo = _sha256_init("mesh_topo_v1")
    _hash_update_ints(h_topo, face_vertex_counts)
    _hash_update_ints(h_topo, face_vertex_indices)
    h_topo.update(struct.pack("<I", len(points)))  # vertex count
    _hash_update_token(h_topo, subdivision_scheme)
    h_topo.update(struct.pack("<?", bool(double_sided)))
    if normals:
        # normals_interp is set above in the geom hash section when normals exist
        _hash_update_token(h_topo, normals_interp)
        h_topo.update(struct.pack("<I", len(normals)))
    else:
        _hash_update_token(h_topo, "<no_normals>")
    if st_pv is not None:
        _hash_update_token(h_topo, st_pv.GetInterpolation())
        _hash_update_ints(h_topo, [int(st_pv.GetElementSize())])
        st_vals_topo = st_pv.Get() or []
        h_topo.update(struct.pack("<I", len(st_vals_topo)))
        # Include UV values (they're exact, no float noise from bake/unbake)
        for uv in st_vals_topo:
            _hash_update_floats(h_topo, (float(uv[0]), float(uv[1])), eps=float_eps)
        st_idx_topo = []
        try:
            if st_pv.IsIndexed():
                st_idx_topo = st_pv.GetIndices() or []
        except Exception:
            pass
        _hash_update_ints(h_topo, st_idx_topo)
    else:
        _hash_update_token(h_topo, "<no_st>")

    topo_sig_hex = h_topo.hexdigest()

    # --- Transform signatures
    # World transform at the mesh prim.
    world_m = xform_cache.GetLocalToWorldTransform(prim)
    m16 = _matrix_to_row_major16(world_m)
    sx, sy, sz = _matrix_scale_xyz(world_m)

    h_scale = _sha256_init("mesh_scale_only_v1")
    _hash_update_str(h_scale, geom_sig_hex)
    _hash_update_floats(h_scale, (sx, sy, sz), eps=float_eps)

    h_full = _sha256_init("mesh_full_matrix_v1")
    _hash_update_str(h_full, geom_sig_hex)
    _hash_update_floats(h_full, m16, eps=float_eps)

    face_count = int(len(face_vertex_counts))

    # Material binding (for topo_invariant mode)
    try:
        binding_api = UsdShade.MaterialBindingAPI(prim)
        direct_binding = binding_api.GetDirectBinding()
        mat_path = direct_binding.GetMaterialPath()
        material_binding = str(mat_path) if mat_path and not mat_path.isEmpty else None
    except Exception:
        material_binding = None

    # --- Shape-invariant signature
    normalized_pts = _normalize_to_unit_bbox(points)
    shape_inv_sig = _shape_invariant_mesh_sig(normalized_pts, face_count, eps=float_eps if float_eps > 0 else SHAPE_DESCRIPTOR_EPS)
    shape_desc = _compute_shape_descriptor(points, face_count, eps=SHAPE_DESCRIPTOR_EPS)
    shape_desc_key = f"{shape_desc.vertex_count}_{shape_desc.aspect_ratio_hash}"

    return MeshSig(
        prim_path=str(prim.GetPath()),
        geom_sig_hex=geom_sig_hex,
        scale_sig_hex=h_scale.hexdigest(),
        full_matrix_sig_hex=h_full.hexdigest(),
        topo_sig_hex=topo_sig_hex,
        vertex_count=int(len(points)),
        face_count=face_count,
        has_normals=has_normals,
        has_st=has_st,
        shape_invariant_sig_hex=shape_inv_sig,
        shape_descriptor_key=shape_desc_key,
        material_binding=material_binding,
    )


def _aggregate_asset_sig(mesh_sigs_hex: List[str], *, tag: str) -> str:
    h = _sha256_init(tag)
    # Sort to make it order-independent (multiset preserved via duplicates in list)
    for sig in sorted(mesh_sigs_hex):
        _hash_update_str(h, sig)
    return h.hexdigest()


def _parse_category_uid(assets_root: str, usd_path: str) -> Tuple[Optional[str], Optional[str]]:
    # Expected: <assets_root>/<category>/<uid>/usd/<uid>.usd
    rel = os.path.relpath(usd_path, assets_root)
    parts = rel.split(os.sep)
    if len(parts) >= 4 and parts[2] == "usd":
        return parts[0], parts[1]
    return None, None


def _get_asset_file_sizes(usd_path: str) -> Tuple[Optional[int], Optional[int]]:
    """Get USD and GLB file sizes for an asset.
    Returns (usd_size_bytes, glb_size_bytes) -- None if file doesn't exist.
    """
    usd_size = os.path.getsize(usd_path) if os.path.isfile(usd_path) else None
    uid = os.path.splitext(os.path.basename(usd_path))[0]
    glb_dir = os.path.join(os.path.dirname(os.path.dirname(usd_path)), "glb")
    glb_path = os.path.join(glb_dir, f"{uid}.glb")
    glb_size = os.path.getsize(glb_path) if os.path.isfile(glb_path) else None
    return usd_size, glb_size


def _iter_asset_usd_files(assets_root: str) -> Iterable[str]:
    # GRScenes assets are typically laid out as:
    #   <assets_root>/<category>/<uid>/usd/<uid>.usd
    # Scanning explicitly is much faster than a full os.walk on large datasets.
    try:
        with os.scandir(assets_root) as cat_it:
            categories = [e for e in cat_it if e.is_dir()]
    except FileNotFoundError:
        return

    saw_any = False
    for cat in sorted(categories, key=lambda e: e.name):
        try:
            with os.scandir(cat.path) as uid_it:
                uids = [e for e in uid_it if e.is_dir()]
        except (FileNotFoundError, PermissionError):
            continue

        for uid in sorted(uids, key=lambda e: e.name):
            usd_dir = os.path.join(uid.path, "usd")
            if not os.path.isdir(usd_dir):
                continue
            saw_any = True

            preferred = os.path.join(usd_dir, f"{uid.name}.usd")
            if os.path.isfile(preferred):
                yield preferred
                continue

            try:
                with os.scandir(usd_dir) as usd_it:
                    for f in usd_it:
                        if f.is_file() and f.name.lower().endswith(".usd"):
                            yield f.path
            except (FileNotFoundError, PermissionError):
                continue

    # Fallback: if the dataset layout differs, use a full walk.
    if not saw_any:
        for dirpath, _, filenames in os.walk(assets_root):
            if os.path.basename(dirpath) != "usd":
                continue
            for fn in filenames:
                if fn.lower().endswith(".usd"):
                    yield os.path.join(dirpath, fn)


def _make_duplicates_map(records: List[AssetRecord], key: str) -> Dict[str, List[str]]:
    out: Dict[str, List[str]] = {}
    for r in records:
        sig = getattr(r, key)
        out.setdefault(sig, []).append(r.usd_path)
    return {sig: paths for sig, paths in out.items() if len(paths) > 1}


# ---------------------------------------------------------------------------
# Tolerance-based merge (post-processing pass)
# ---------------------------------------------------------------------------

def _read_mesh_points(usd_path: str) -> List[List[Tuple[float, float, float]]]:
    """Read all mesh vertex positions from a USD, sorted by (vertex_count, face_count).

    Returns a list of meshes, each mesh is a list of (x, y, z) tuples.
    """
    stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    if stage is None:
        return []
    meshes = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get() or []
        fvc = mesh.GetFaceVertexCountsAttr().Get() or []
        pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        meshes.append((len(pts), len(fvc), pts))
    # Sort by (vertex_count, face_count) for consistent pairing
    meshes.sort(key=lambda m: (m[0], m[1]))
    return [m[2] for m in meshes]


def _max_vertex_distance(meshes_a: List[List[Tuple[float, float, float]]],
                          meshes_b: List[List[Tuple[float, float, float]]]) -> float:
    """Compute max per-vertex distance between two assets (paired by mesh size).

    Returns float('inf') if meshes can't be paired (different counts or sizes).
    """
    if len(meshes_a) != len(meshes_b):
        return float("inf")
    max_dist = 0.0
    for pts_a, pts_b in zip(meshes_a, meshes_b):
        if len(pts_a) != len(pts_b):
            return float("inf")
        for (ax, ay, az), (bx, by, bz) in zip(pts_a, pts_b):
            dx, dy, dz = ax - bx, ay - by, az - bz
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d > max_dist:
                max_dist = d
        # Early exit if already above any reasonable tolerance
        if max_dist > 1.0:
            return max_dist
    return max_dist


# ---------------------------------------------------------------------------
# Shape-invariant merge (for shape-invariant dedup mode)
# ---------------------------------------------------------------------------

def _shape_invariant_merge(
    records: List[AssetRecord],
    asset_mesh_points: Optional[Dict[str, List[Tuple[np.ndarray, int, int]]]] = None,
    hausdorff_threshold: float = 0.05,
    *,
    tolerance: Optional[float] = None,
) -> Dict[str, List[str]]:
    """Find duplicate groups using shape-invariant Hausdorff distance metric.

    Algorithm:
    1. Pre-filter: Group records by asset_shape_descriptor_key
       (aggregated per-mesh vertex_count + aspect_ratio_hash). Skip singletons.
    2. Within each pre-filter group: normalize meshes to unit bbox,
       sort by (vertex_count, face_count), pair 1:1.
    3. Compute max Hausdorff distance across all mesh pairs.
    4. Union-Find to merge assets where max Hausdorff < hausdorff_threshold.

    Args:
        records: List of AssetRecord with asset_shape_descriptor_key populated.
        asset_mesh_points: Optional dict mapping usd_path to list of
            (points_array, vertex_count, face_count) tuples. If None, meshes
            are loaded on demand from USD files.
        hausdorff_threshold: Hausdorff distance threshold on unit-normalized
            meshes. Default 0.05 = 5% of bounding box.
        tolerance: Alias for hausdorff_threshold (backward compatibility).

    Returns:
        Dict[str, List[str]]: {group_sig: [usd_paths]} for groups with >= 2 members.
    """
    import logging

    # Support 'tolerance' kwarg as alias for hausdorff_threshold
    if tolerance is not None:
        hausdorff_threshold = tolerance

    # Pre-filter grouping by shape descriptor key
    shape_groups: Dict[str, List[AssetRecord]] = {}
    skipped = 0
    for r in records:
        if r.mesh_count == 0 or not r.asset_shape_descriptor_key:
            skipped += 1
            continue
        shape_groups.setdefault(r.asset_shape_descriptor_key, []).append(r)

    # Drop singletons
    shape_groups = {k: v for k, v in shape_groups.items() if len(v) >= 2}

    total_candidates = sum(len(g) for g in shape_groups.values())
    logging.info(
        "shape_invariant_merge: %d pre-filter groups, %d candidate assets, "
        "%d skipped (empty mesh or no descriptor key)",
        len(shape_groups), total_candidates, skipped,
    )

    # Cache of normalized meshes per asset
    norm_cache: Dict[str, Optional[List[np.ndarray]]] = {}

    def _load_normalized_from_usd(usd_path: str) -> Optional[List[np.ndarray]]:
        """Fallback: load and normalize meshes directly from USD file."""
        try:
            stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
            if stage is None:
                return None
            meshes = []
            for prim in stage.Traverse():
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(prim)
                points = mesh.GetPointsAttr().Get() or []
                fvc = mesh.GetFaceVertexCountsAttr().Get() or []
                if points:
                    normalized = _normalize_to_unit_bbox(points)
                    meshes.append((len(points), len(fvc), normalized))
            meshes.sort(key=lambda m: (m[0], m[1]))
            return [m[2] for m in meshes] if meshes else None
        except Exception:
            return None

    def _get_normalized(usd_path: str) -> Optional[List[np.ndarray]]:
        if usd_path in norm_cache:
            return norm_cache[usd_path]

        if asset_mesh_points is not None:
            raw = asset_mesh_points.get(usd_path)
            if raw is None:
                norm_cache[usd_path] = None
                return None
            sorted_meshes = sorted(raw, key=lambda m: (m[1], m[2]))
            result = []
            for pts, _vc, _fc in sorted_meshes:
                if len(pts) == 0:
                    continue
                result.append(_normalize_to_unit_bbox(pts))
            norm_cache[usd_path] = result if result else None
        else:
            norm_cache[usd_path] = _load_normalized_from_usd(usd_path)

        return norm_cache[usd_path]

    new_group_id = 0
    merged: Dict[str, List[str]] = {}
    groups_processed = 0

    for desc_key, group in shape_groups.items():
        groups_processed += 1
        if groups_processed % 500 == 0:
            logging.info(
                "shape_invariant_merge: processed %d/%d pre-filter groups",
                groups_processed, len(shape_groups),
            )

        n = len(group)

        # Union-Find
        parent: Dict[int, int] = {i: i for i in range(n)}

        def _find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def _union(a: int, b: int) -> None:
            ra, rb = _find(a), _find(b)
            if ra != rb:
                parent[rb] = ra

        # Pairwise Hausdorff comparison within pre-filter group.
        # Pre-filtering by shape_descriptor_key reduces group size to typically <50.
        # Early exit on threshold breach further limits actual comparisons.
        # Acceptable O(n²) within small pre-filtered groups.
        for i in range(n):
            meshes_i = _get_normalized(group[i].usd_path)
            if meshes_i is None:
                continue
            for j in range(i + 1, n):
                if _find(i) == _find(j):
                    continue

                meshes_j = _get_normalized(group[j].usd_path)
                if meshes_j is None:
                    continue

                if len(meshes_i) != len(meshes_j):
                    continue

                max_hausdorff = 0.0
                exceeded = False
                for pts_i, pts_j in zip(meshes_i, meshes_j):
                    h = _hausdorff_distance(pts_i, pts_j)
                    if h == float("inf"):
                        exceeded = True
                        break
                    if h > max_hausdorff:
                        max_hausdorff = h
                    if max_hausdorff > hausdorff_threshold:
                        exceeded = True
                        break

                if not exceeded and max_hausdorff <= hausdorff_threshold:
                    _union(i, j)

        # Collect merged clusters
        clusters: Dict[int, List[str]] = {}
        for i in range(n):
            root = _find(i)
            clusters.setdefault(root, []).append(group[i].usd_path)

        for cluster_paths in clusters.values():
            if len(cluster_paths) >= 2:
                sig_key = f"shape_invariant_merge_{new_group_id}"
                new_group_id += 1
                merged[sig_key] = sorted(cluster_paths)

    logging.info(
        "shape_invariant_merge: done. %d duplicate groups found.",
        len(merged),
    )
    return merged


def _filesize_match(size_a: Optional[int], size_b: Optional[int], tolerance: float) -> bool:
    """Check if two file sizes are within tolerance of each other."""
    if size_a is None or size_b is None:
        return False
    if size_a == 0 and size_b == 0:
        return True
    max_size = max(size_a, size_b)
    if max_size == 0:
        return True
    return abs(size_a - size_b) / max_size <= tolerance


def _topo_filesize_merge(
    records: List[AssetRecord],
    filesize_tolerance: float = 0.02,
) -> Dict[str, List[str]]:
    """Find duplicate groups using topology-invariant sig + file-size matching.

    Algorithm:
    1. Pre-filter: group by asset_topo_invariant_sig_hex, drop singletons
    2. Within each group: pairwise GLB file-size match (primary), USD fallback
    3. Union-Find merge, collect connected components
    """
    import logging

    topo_groups: Dict[str, List[AssetRecord]] = {}
    skipped = 0
    for r in records:
        if r.mesh_count == 0 or not r.asset_topo_invariant_sig_hex:
            skipped += 1
            continue
        topo_groups.setdefault(r.asset_topo_invariant_sig_hex, []).append(r)

    # Drop singletons
    topo_groups = {k: v for k, v in topo_groups.items() if len(v) >= 2}

    total_candidates = sum(len(g) for g in topo_groups.values())
    logging.info(
        "topo_filesize_merge: %d pre-filter groups, %d candidate assets, %d skipped",
        len(topo_groups), total_candidates, skipped,
    )

    new_group_id = 0
    merged: Dict[str, List[str]] = {}

    for sig_key, group in topo_groups.items():
        n = len(group)
        parent: Dict[int, int] = {i: i for i in range(n)}

        def _find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def _union(a: int, b: int) -> None:
            ra, rb = _find(a), _find(b)
            if ra != rb:
                parent[rb] = ra

        for i in range(n):
            for j in range(i + 1, n):
                if _find(i) == _find(j):
                    continue

                glb_ok = _filesize_match(
                    group[i].glb_file_size, group[j].glb_file_size, filesize_tolerance
                )
                if glb_ok:
                    _union(i, j)
                elif group[i].glb_file_size is None and group[j].glb_file_size is None:
                    # Fallback: both missing GLB, match on USD with tighter tolerance
                    usd_ok = _filesize_match(
                        group[i].usd_file_size, group[j].usd_file_size,
                        filesize_tolerance / 2,
                    )
                    if usd_ok:
                        _union(i, j)

        # Collect clusters
        clusters: Dict[int, List[str]] = {}
        for i in range(n):
            root = _find(i)
            clusters.setdefault(root, []).append(group[i].usd_path)

        for cluster_paths in clusters.values():
            if len(cluster_paths) >= 2:
                sig = f"topo_filesize_merge_{new_group_id}"
                new_group_id += 1
                merged[sig] = sorted(cluster_paths)

    logging.info("topo_filesize_merge: done. %d duplicate groups found.", len(merged))
    return merged


def _tolerance_merge(
    records: List[AssetRecord],
    existing_dups: Dict[str, List[str]],
    tolerance: float,
) -> Dict[str, List[str]]:
    """Find additional duplicate groups by tolerance-based vertex comparison.

    1. Group records by topology hash (asset_topo_sig_hex).
    2. Within each topology group, skip assets already in existing_dups.
    3. For remaining assets, do pairwise max-vertex-distance comparison.
    4. Merge assets within tolerance into new groups.

    Returns merged duplicates map (existing + new groups).
    """
    already_grouped: set = set()
    for paths in existing_dups.values():
        already_grouped.update(paths)

    # Group by topology
    topo_groups: Dict[str, List[AssetRecord]] = {}
    for r in records:
        topo_groups.setdefault(r.asset_topo_sig_hex, []).append(r)

    new_group_id = 0
    merged = dict(existing_dups)  # copy

    for topo_sig, group in topo_groups.items():
        if len(group) < 2:
            continue

        # Load mesh points for all candidates (lazy, cached)
        points_cache: Dict[str, Optional[List[List[Tuple[float, float, float]]]]] = {}

        def _get_points(usd_path: str):
            if usd_path not in points_cache:
                try:
                    points_cache[usd_path] = _read_mesh_points(usd_path)
                except Exception:
                    points_cache[usd_path] = None
            return points_cache[usd_path]

        # Union-Find for merging
        parent: Dict[int, int] = {i: i for i in range(len(group))}

        def _find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def _union(a: int, b: int) -> None:
            ra, rb = _find(a), _find(b)
            if ra != rb:
                parent[rb] = ra

        for i in range(len(group)):
            for j in range(i + 1, len(group)):
                # Skip if both already in same existing group
                if (group[i].usd_path in already_grouped and
                    group[j].usd_path in already_grouped):
                    continue
                # Skip if already same hash (already grouped)
                if group[i].asset_geom_sig_hex == group[j].asset_geom_sig_hex:
                    _union(i, j)
                    continue
                pts_i = _get_points(group[i].usd_path)
                pts_j = _get_points(group[j].usd_path)
                if pts_i is None or pts_j is None:
                    continue
                dist = _max_vertex_distance(pts_i, pts_j)
                if dist <= tolerance:
                    _union(i, j)

        # Collect merged groups
        clusters: Dict[int, List[str]] = {}
        for i in range(len(group)):
            root = _find(i)
            clusters.setdefault(root, []).append(group[i].usd_path)

        for paths in clusters.values():
            if len(paths) < 2:
                continue
            # Check if this is a genuinely new group (not already in merged)
            paths_set = set(paths)
            is_new = True
            for existing_paths in merged.values():
                if paths_set == set(existing_paths):
                    is_new = False
                    break
                if paths_set.issubset(set(existing_paths)):
                    is_new = False
                    break
            if is_new:
                sig_key = f"tolerance_merge_{new_group_id}"
                new_group_id += 1
                # Merge with any overlapping existing group
                final_paths = set(paths)
                keys_to_remove = []
                for k, v in merged.items():
                    if final_paths & set(v):
                        final_paths.update(v)
                        keys_to_remove.append(k)
                for k in keys_to_remove:
                    del merged[k]
                merged[sig_key] = sorted(final_paths)

    return merged


def _write_report(
    out_path: str,
    *,
    mode: str,
    assets_root: str,
    dataset: str,
    float_eps: float,
    records: List[AssetRecord],
    errors: List[Dict[str, str]],
    started_at: float,
    merge_tolerance: float = 0.0,
    **kwargs,
) -> None:
    duplicates_key = {
        "geom_only": "asset_geom_sig_hex",
        "scale_only": "asset_scale_sig_hex",
        "full_matrix": "asset_full_matrix_sig_hex",
        "shape_invariant": "asset_shape_invariant_sig_hex",
    }.get(mode)

    if mode == "shape_invariant":
        # Use Hausdorff-based merge instead of simple hash grouping
        hausdorff_threshold = kwargs.get("hausdorff_threshold", 0.05)
        dups = _shape_invariant_merge(records, tolerance=hausdorff_threshold)
    elif mode == "topo_filesize":
        filesize_tol = kwargs.get("filesize_tolerance", 0.02)
        dups = _topo_filesize_merge(records, filesize_tolerance=filesize_tol)
    else:
        assert duplicates_key is not None, f"Unknown mode: {mode}"
        dups = _make_duplicates_map(records, duplicates_key)

    # tolerance_merged_count kept at 0 — tolerance merge was removed from
    # geom_only mode because it contaminated hash-identical reports with
    # near-miss groups that have different geometry hashes.
    tolerance_merged_count = 0

    meta = {
        "dataset": dataset,
        "mode": mode,
        "assets_root": os.path.abspath(assets_root),
        "float_quantize_eps": float_eps,
        "merge_tolerance": merge_tolerance,
        "tolerance_merged_groups": tolerance_merged_count,
        "asset_usd_count": len(records),
        "error_count": len(errors),
        "duplicate_group_count": len(dups),
        "generated_at_unix": int(time.time()),
        "elapsed_sec": time.time() - started_at,
    }
    if mode == "shape_invariant":
        meta["hausdorff_threshold"] = kwargs.get("hausdorff_threshold", 0.05)
    if mode == "topo_filesize":
        meta["filesize_tolerance"] = kwargs.get("filesize_tolerance", 0.02)

    payload = {
        "meta": meta,
        "duplicates": [
            {"sig": sig, "count": len(paths), "usd_paths": paths}
            for sig, paths in sorted(dups.items(), key=lambda kv: (-len(kv[1]), kv[0]))
        ],
        "assets": [
            {
                "usd_path": r.usd_path,
                "category": r.category,
                "uid": r.uid,
                "mesh_count": r.mesh_count,
                "asset_geom_sig": r.asset_geom_sig_hex,
                "asset_scale_sig": r.asset_scale_sig_hex,
                "asset_full_matrix_sig": r.asset_full_matrix_sig_hex,
                "asset_topo_sig": r.asset_topo_sig_hex,
                "asset_topo_invariant_sig": getattr(r, 'asset_topo_invariant_sig_hex', ''),
                "usd_file_size": getattr(r, 'usd_file_size', None),
                "glb_file_size": getattr(r, 'glb_file_size', None),
                "meshes": [
                    {
                        "prim_path": m.prim_path,
                        "geom_sig": m.geom_sig_hex,
                        "scale_sig": m.scale_sig_hex,
                        "full_matrix_sig": m.full_matrix_sig_hex,
                        "topo_sig": m.topo_sig_hex,
                        "vertex_count": m.vertex_count,
                        "face_count": m.face_count,
                        "has_normals": m.has_normals,
                        "has_st": m.has_st,
                    }
                    for m in r.meshes
                ],
            }
            for r in records
        ],
        "errors": errors,
    }

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--assets-root",
        required=True,
        help="Assets root directory (e.g. GRScenes-test1/GRScenes_assets)",
    )
    parser.add_argument(
        "--out-dir",
        default="check_reports",
        help="Output directory for JSON reports",
    )
    parser.add_argument(
        "--dataset",
        default="test1",
        help="Dataset label to embed in reports/filenames",
    )
    parser.add_argument(
        "--float-quantize-eps",
        type=float,
        default=0.0,
        help="Quantize floats to this epsilon before hashing (0 disables)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Optional limit on number of USD files (0 = no limit)",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=100,
        help="Print/write progress every N files (0 disables)",
    )
    parser.add_argument(
        "--merge-tolerance",
        type=float,
        default=0.0,
        help="Post-hash tolerance merge: assets with identical topology and max "
             "per-vertex distance <= this value are merged as duplicates. "
             "Fixes hash bucket-boundary misses from floating-point noise. "
             "Recommended: 0.005 for normalized assets (0 disables).",
    )
    parser.add_argument(
        "--mode",
        choices=["all", "shape_invariant", "topo_filesize"],
        default="all",
        help="Dedup mode. 'all' generates 3 standard reports. "
             "'shape_invariant' generates a scale/order-invariant report. "
             "'topo_filesize' generates a topology+file-size invariant report.",
    )
    parser.add_argument(
        "--hausdorff-threshold",
        type=float,
        default=0.05,
        help="Hausdorff distance threshold for shape_invariant mode "
             "(fraction of unit bbox, default 0.05)",
    )
    parser.add_argument(
        "--filesize-tolerance",
        type=float,
        default=0.02,
        help="File size tolerance for topo_filesize mode (default 0.02 = 2%%)",
    )
    parser.add_argument(
        "--progress-jsonl",
        default=None,
        help="Optional JSONL progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.jsonl)",
    )
    parser.add_argument(
        "--progress-json",
        default=None,
        help="Optional JSON progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.json)",
    )
    args = parser.parse_args()

    if args.hausdorff_threshold < 0 or math.isinf(args.hausdorff_threshold):
        parser.error("--hausdorff-threshold must be non-negative and finite")

    assets_root = args.assets_root
    out_dir = args.out_dir
    dataset = args.dataset
    float_eps = float(args.float_quantize_eps)

    started_at = time.time()

    print("Starting asset mesh dedup report...", flush=True)

    progress_jsonl = args.progress_jsonl
    if not progress_jsonl:
        progress_jsonl = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.jsonl")
    progress_json = args.progress_json
    if not progress_json:
        progress_json = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.json")
    os.makedirs(os.path.dirname(progress_jsonl), exist_ok=True)

    records: List[AssetRecord] = []
    errors: List[Dict[str, str]] = []

    total = 0

    def _emit_progress(processed: int, *, phase: str, current_usd: Optional[str] = None) -> None:
        if not args.progress_every or args.progress_every <= 0:
            return
        elapsed = max(1e-9, time.time() - started_at)
        rate = processed / elapsed
        if total <= 0:
            eta_sec = None
        else:
            remaining = max(0, total - processed)
            eta_sec = remaining / rate if rate > 0 else None
        payload = {
            "dataset": dataset,
            "assets_root": os.path.abspath(assets_root),
            "phase": phase,
            "processed": processed,
            "total": total,
            "errors": len(errors),
            "current_usd": current_usd,
            "elapsed_sec": elapsed,
            "rate_files_per_sec": rate,
            "eta_sec": eta_sec,
            "timestamp_unix": int(time.time()),
        }
        with open(progress_jsonl, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
        with open(progress_json, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"
        cur = "" if not current_usd else f" | cur={os.path.basename(current_usd)}"
        print(
            f"  {phase} {processed}/{total} | errors={len(errors)} | rate={rate:.2f}/s | eta={eta_msg}{cur}",
            flush=True,
        )

    # Discover asset USDs
    print(f"Listing asset USDs under: {os.path.abspath(assets_root)}", flush=True)
    _emit_progress(0, phase="discover")
    usd_files: List[str] = []
    discover_every = 0
    if args.progress_every and args.progress_every > 0:
        # Don't spam during discovery; discovery can be very long on large datasets.
        discover_every = max(args.progress_every * 20, 2000)
    for i, p in enumerate(_iter_asset_usd_files(assets_root), start=1):
        usd_files.append(p)
        if discover_every and i % discover_every == 0:
            _emit_progress(i, phase="discover")
    usd_files.sort()
    if args.limit and args.limit > 0:
        usd_files = usd_files[: args.limit]

    total = len(usd_files)
    print(f"Scanning {total} asset USDs...", flush=True)
    # Emit an initial progress entry so users can 'watch' immediately.
    _emit_progress(0, phase="scan")

    for idx, usd_path in enumerate(usd_files, start=1):
        # Emit an early progress record when starting the first file so users
        # can see which file we're on even if Stage.Open is slow.
        if idx == 1:
            _emit_progress(1, phase="scan", current_usd=usd_path)
        if args.progress_every and args.progress_every > 0 and idx % args.progress_every == 0:
            _emit_progress(idx, phase="scan", current_usd=usd_path)

        try:
            stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
            if stage is None:
                raise RuntimeError("Usd.Stage.Open returned None")

            xform_cache = UsdGeom.XformCache()
            mesh_sigs: List[MeshSig] = []
            for prim in stage.Traverse():
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(prim)
                mesh_sigs.append(_compute_mesh_sigs(mesh, xform_cache, float_eps=float_eps))

            category, uid = _parse_category_uid(assets_root, usd_path)

            asset_geom_sig = _aggregate_asset_sig([m.geom_sig_hex for m in mesh_sigs], tag="asset_geom_only_v1")
            asset_scale_sig = _aggregate_asset_sig(
                [m.scale_sig_hex for m in mesh_sigs], tag="asset_scale_only_v1"
            )
            asset_full_sig = _aggregate_asset_sig(
                [m.full_matrix_sig_hex for m in mesh_sigs], tag="asset_full_matrix_v1"
            )
            asset_topo_sig = _aggregate_asset_sig(
                [m.topo_sig_hex for m in mesh_sigs], tag="asset_topo_only_v1"
            )
            asset_shape_inv_sig = _aggregate_asset_sig(
                [m.shape_invariant_sig_hex for m in mesh_sigs], tag="asset_shape_invariant_v1"
            )
            # Aggregate shape descriptor key: sort per-mesh keys and join
            per_mesh_desc_keys = sorted(m.shape_descriptor_key for m in mesh_sigs)
            asset_shape_desc_key = "|".join(per_mesh_desc_keys) if per_mesh_desc_keys else ""

            # Topo-invariant sig: order-independent topology + material bindings
            topo_inv_descs = sorted(
                [(m.vertex_count, m.face_count, m.material_binding or "<no_material>") for m in mesh_sigs],
                key=lambda t: (t[0], t[1], t[2])
            )
            h_topo_inv = hashlib.sha256()
            h_topo_inv.update(f"topo_invariant_v1:{len(mesh_sigs)}".encode())
            for vc, fc, mat in topo_inv_descs:
                h_topo_inv.update(struct.pack("<II", vc, fc))
                h_topo_inv.update(mat.encode("utf-8"))
            asset_topo_inv_sig = h_topo_inv.hexdigest()

            # File sizes for topo_filesize mode
            usd_file_size, glb_file_size = _get_asset_file_sizes(usd_path)

            records.append(
                AssetRecord(
                    usd_path=usd_path,
                    category=category,
                    uid=uid,
                    mesh_count=len(mesh_sigs),
                    asset_geom_sig_hex=asset_geom_sig,
                    asset_scale_sig_hex=asset_scale_sig,
                    asset_full_matrix_sig_hex=asset_full_sig,
                    asset_topo_sig_hex=asset_topo_sig,
                    meshes=mesh_sigs,
                    asset_shape_invariant_sig_hex=asset_shape_inv_sig,
                    asset_shape_descriptor_key=asset_shape_desc_key,
                    asset_topo_invariant_sig_hex=asset_topo_inv_sig,
                    usd_file_size=usd_file_size,
                    glb_file_size=glb_file_size,
                )
            )
        except Exception as e:
            errors.append({"usd_path": usd_path, "error": str(e)})

    # Write three reports (same asset list, different duplicates perspective)
    out_geom = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_geom_only.json")
    out_scale = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_scale_only.json")
    out_full = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_full_matrix.json")

    merge_tol = float(args.merge_tolerance)

    _write_report(
        out_geom,
        mode="geom_only",
        assets_root=assets_root,
        dataset=dataset,
        float_eps=float_eps,
        records=records,
        errors=errors,
        started_at=started_at,
        merge_tolerance=merge_tol,
    )
    _write_report(
        out_scale,
        mode="scale_only",
        assets_root=assets_root,
        dataset=dataset,
        float_eps=float_eps,
        records=records,
        errors=errors,
        started_at=started_at,
    )
    _write_report(
        out_full,
        mode="full_matrix",
        assets_root=assets_root,
        dataset=dataset,
        float_eps=float_eps,
        records=records,
        errors=errors,
        started_at=started_at,
    )

    # Shape-invariant report (only when --mode shape_invariant)
    out_shape_inv = None
    if args.mode == "shape_invariant":
        out_shape_inv = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_shape_invariant.json")
        hausdorff_thr = float(args.hausdorff_threshold)
        _write_report(
            out_shape_inv,
            mode="shape_invariant",
            assets_root=assets_root,
            dataset=dataset,
            float_eps=float_eps,
            records=records,
            errors=errors,
            started_at=started_at,
            hausdorff_threshold=hausdorff_thr,
        )

    # Topo-filesize report (only when --mode topo_filesize)
    out_topo_fs = None
    if args.mode == "topo_filesize":
        out_topo_fs = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_topo_filesize.json")
        _write_report(
            out_topo_fs,
            mode="topo_filesize",
            assets_root=assets_root,
            dataset=dataset,
            float_eps=float_eps,
            records=records,
            errors=errors,
            started_at=started_at,
            filesize_tolerance=float(args.filesize_tolerance),
        )

    _emit_progress(total, phase="finalize")

    print("Wrote reports:", flush=True)
    print(f"  {out_geom}", flush=True)
    print(f"  {out_scale}", flush=True)
    print(f"  {out_full}", flush=True)
    if out_shape_inv:
        print(f"  {out_shape_inv}", flush=True)
    if out_topo_fs:
        print(f"  {out_topo_fs}", flush=True)
    if errors:
        print(f"WARNING: {len(errors)} files had errors (see report errors[])", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
