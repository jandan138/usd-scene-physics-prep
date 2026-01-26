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

from pxr import Gf, Usd, UsdGeom


@dataclass(frozen=True)
class MeshSig:
    prim_path: str
    geom_sig_hex: str
    scale_sig_hex: str
    full_matrix_sig_hex: str
    vertex_count: int
    face_count: int
    has_normals: bool
    has_st: bool


@dataclass(frozen=True)
class AssetRecord:
    usd_path: str
    category: Optional[str]
    uid: Optional[str]
    mesh_count: int
    asset_geom_sig_hex: str
    asset_scale_sig_hex: str
    asset_full_matrix_sig_hex: str
    meshes: List[MeshSig]


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

    return MeshSig(
        prim_path=str(prim.GetPath()),
        geom_sig_hex=geom_sig_hex,
        scale_sig_hex=h_scale.hexdigest(),
        full_matrix_sig_hex=h_full.hexdigest(),
        vertex_count=int(len(points)),
        face_count=face_count,
        has_normals=has_normals,
        has_st=has_st,
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
) -> None:
    duplicates_key = {
        "geom_only": "asset_geom_sig_hex",
        "scale_only": "asset_scale_sig_hex",
        "full_matrix": "asset_full_matrix_sig_hex",
    }[mode]

    dups = _make_duplicates_map(records, duplicates_key)

    payload = {
        "meta": {
            "dataset": dataset,
            "mode": mode,
            "assets_root": os.path.abspath(assets_root),
            "float_quantize_eps": float_eps,
            "asset_usd_count": len(records),
            "error_count": len(errors),
            "duplicate_group_count": len(dups),
            "generated_at_unix": int(time.time()),
            "elapsed_sec": time.time() - started_at,
        },
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
                "meshes": [
                    {
                        "prim_path": m.prim_path,
                        "geom_sig": m.geom_sig_hex,
                        "scale_sig": m.scale_sig_hex,
                        "full_matrix_sig": m.full_matrix_sig_hex,
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

            records.append(
                AssetRecord(
                    usd_path=usd_path,
                    category=category,
                    uid=uid,
                    mesh_count=len(mesh_sigs),
                    asset_geom_sig_hex=asset_geom_sig,
                    asset_scale_sig_hex=asset_scale_sig,
                    asset_full_matrix_sig_hex=asset_full_sig,
                    meshes=mesh_sigs,
                )
            )
        except Exception as e:
            errors.append({"usd_path": usd_path, "error": str(e)})

    # Write three reports (same asset list, different duplicates perspective)
    out_geom = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_geom_only.json")
    out_scale = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_scale_only.json")
    out_full = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_full_matrix.json")

    _write_report(
        out_geom,
        mode="geom_only",
        assets_root=assets_root,
        dataset=dataset,
        float_eps=float_eps,
        records=records,
        errors=errors,
        started_at=started_at,
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

    _emit_progress(total, phase="finalize")

    print("Wrote reports:", flush=True)
    print(f"  {out_geom}", flush=True)
    print(f"  {out_scale}", flush=True)
    print(f"  {out_full}", flush=True)
    if errors:
        print(f"WARNING: {len(errors)} files had errors (see report errors[])", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
