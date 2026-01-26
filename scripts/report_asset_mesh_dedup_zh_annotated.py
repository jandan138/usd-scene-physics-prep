# -*- coding: utf-8 -*-
# 说明：这是 scripts/report_asset_mesh_dedup.py 的“逐行中文注释版”（自动生成）。
# 目的：方便阅读理解；实际运行建议使用原脚本 report_asset_mesh_dedup.py。
# 对应导读文档：docs/operations/asset_mesh_dedup_code_guide.md

#!/usr/bin/env python3  # shebang：指定解释器
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

from __future__ import annotations  # 启用未来特性（类型注解等）

import argparse  # 导入依赖模块
import hashlib  # 导入依赖模块
import json  # 导入依赖模块
import math  # 导入依赖模块
import os  # 导入依赖模块
import struct  # 导入依赖模块
import time  # 导入依赖模块
from dataclasses import dataclass  # 导入依赖模块
from typing import Dict, Iterable, List, Optional, Sequence, Tuple  # 导入依赖模块

from pxr import Gf, Usd, UsdGeom  # 导入依赖模块


@dataclass(frozen=True)  # 装饰器：用于 dataclass 等
class MeshSig:  # 定义类/数据结构
    prim_path: str  # 执行语句
    geom_sig_hex: str  # 执行语句
    scale_sig_hex: str  # 执行语句
    full_matrix_sig_hex: str  # 执行语句
    vertex_count: int  # 执行语句
    face_count: int  # 执行语句
    has_normals: bool  # 执行语句
    has_st: bool  # 执行语句


@dataclass(frozen=True)  # 装饰器：用于 dataclass 等
class AssetRecord:  # 定义类/数据结构
    usd_path: str  # 执行语句
    category: Optional[str]  # 执行语句
    uid: Optional[str]  # 执行语句
    mesh_count: int  # 执行语句
    asset_geom_sig_hex: str  # 执行语句
    asset_scale_sig_hex: str  # 执行语句
    asset_full_matrix_sig_hex: str  # 执行语句
    meshes: List[MeshSig]  # 执行语句


def _sha256_init(tag: str) -> "hashlib._Hash":  # 定义函数
    h = hashlib.sha256()  # 赋值/初始化变量
    h.update(tag.encode("utf-8"))  # 执行语句
    h.update(b"\0")  # 执行语句
    return h  # 返回结果


def _hash_update_str(h: "hashlib._Hash", s: str) -> None:  # 定义函数
    data = s.encode("utf-8")  # 赋值/初始化变量
    h.update(struct.pack("<I", len(data)))  # 执行语句
    h.update(data)  # 执行语句


def _hash_update_token(h: "hashlib._Hash", token: Optional[str]) -> None:  # 定义函数
    _hash_update_str(h, "" if token is None else str(token))  # 执行语句


def _hash_update_ints(h: "hashlib._Hash", values: Sequence[int]) -> None:  # 定义函数
    h.update(struct.pack("<I", len(values)))  # 执行语句
    for v in values:  # 循环
        h.update(struct.pack("<i", int(v)))  # 执行语句


def _quantize(v: float, eps: float) -> float:  # 定义函数
    if eps <= 0:  # 条件分支
        return float(v)  # 返回结果
    return round(float(v) / eps) * eps  # 返回结果


def _hash_update_floats(h: "hashlib._Hash", values: Sequence[float], *, eps: float) -> None:  # 定义函数
    h.update(struct.pack("<I", len(values)))  # 执行语句
    for v in values:  # 循环
        h.update(struct.pack("<d", _quantize(float(v), eps)))  # 执行语句


def _matrix_to_row_major16(m: Gf.Matrix4d) -> List[float]:  # 定义函数
    # Gf.Matrix4d is indexable as m[row][col]  # 执行语句
    return [float(m[r][c]) for r in range(4) for c in range(4)]  # 返回结果


def _matrix_scale_xyz(m: Gf.Matrix4d) -> Tuple[float, float, float]:  # 定义函数
    # Use column vectors of the 3x3 part: scale = ||col||  # 赋值/初始化变量
    # Handles typical TRS matrices; shear is folded into the scale.  # 执行语句
    c0 = (float(m[0][0]), float(m[1][0]), float(m[2][0]))  # 赋值/初始化变量
    c1 = (float(m[0][1]), float(m[1][1]), float(m[2][1]))  # 赋值/初始化变量
    c2 = (float(m[0][2]), float(m[1][2]), float(m[2][2]))  # 赋值/初始化变量

    def _len3(v: Tuple[float, float, float]) -> float:  # 定义函数
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])  # 返回结果

    return (_len3(c0), _len3(c1), _len3(c2))  # 返回结果


def _get_st_primvar(mesh_prim: Usd.Prim) -> Optional[UsdGeom.Primvar]:  # 定义函数
    pv_api = UsdGeom.PrimvarsAPI(mesh_prim)  # 赋值/初始化变量
    pv = pv_api.GetPrimvar("st")  # 赋值/初始化变量
    if not pv or not pv.HasValue():  # 条件分支
        return None  # 返回结果
    return pv  # 返回结果


def _compute_mesh_sigs(  # 定义函数
    mesh: UsdGeom.Mesh,  # 执行语句
    xform_cache: UsdGeom.XformCache,  # 执行语句
    *,  # 执行语句
    float_eps: float,  # 执行语句
) -> MeshSig:  # 执行语句
    prim = mesh.GetPrim()  # 赋值/初始化变量

    # --- Geometry signature (ignores transforms)  # 执行语句
    h_geom = _sha256_init("mesh_geom_v1")  # 赋值/初始化变量

    points = mesh.GetPointsAttr().Get() or []  # 赋值/初始化变量
    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get() or []  # 赋值/初始化变量
    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get() or []  # 赋值/初始化变量

    _hash_update_ints(h_geom, face_vertex_counts)  # 执行语句
    _hash_update_ints(h_geom, face_vertex_indices)  # 执行语句

    h_geom.update(struct.pack("<I", len(points)))  # 执行语句
    for p in points:  # 循环
        # p is typically Gf.Vec3f/Vec3d  # 执行语句
        _hash_update_floats(h_geom, (float(p[0]), float(p[1]), float(p[2])), eps=float_eps)  # 赋值/初始化变量

    subdivision_scheme = mesh.GetSubdivisionSchemeAttr().Get()  # 赋值/初始化变量
    _hash_update_token(h_geom, subdivision_scheme)  # 执行语句

    double_sided = prim.GetAttribute("doubleSided").Get()  # 赋值/初始化变量
    if double_sided is None:  # 条件分支
        double_sided = False  # 赋值/初始化变量
    h_geom.update(struct.pack("<?", bool(double_sided)))  # 执行语句

    has_normals = False  # 赋值/初始化变量
    normals = mesh.GetNormalsAttr().Get() or []  # 赋值/初始化变量
    if normals:  # 条件分支
        has_normals = True  # 赋值/初始化变量
        normals_interp = None  # 赋值/初始化变量
        if hasattr(mesh, "GetNormalsInterpolation"):  # 条件分支
            try:  # 异常处理
                normals_interp = mesh.GetNormalsInterpolation()  # 赋值/初始化变量
            except Exception:  # 异常处理
                normals_interp = None  # 赋值/初始化变量
        _hash_update_token(h_geom, normals_interp)  # 执行语句
        h_geom.update(struct.pack("<I", len(normals)))  # 执行语句
        for n in normals:  # 循环
            _hash_update_floats(h_geom, (float(n[0]), float(n[1]), float(n[2])), eps=float_eps)  # 赋值/初始化变量
    else:  # 条件分支
        _hash_update_token(h_geom, "<no_normals>")  # 执行语句

    has_st = False  # 赋值/初始化变量
    st_pv = _get_st_primvar(prim)  # 赋值/初始化变量
    if st_pv is not None:  # 条件分支
        has_st = True  # 赋值/初始化变量
        _hash_update_token(h_geom, st_pv.GetInterpolation())  # 执行语句
        _hash_update_ints(h_geom, [int(st_pv.GetElementSize())])  # 执行语句
        st_vals = st_pv.Get() or []  # 赋值/初始化变量
        h_geom.update(struct.pack("<I", len(st_vals)))  # 执行语句
        for uv in st_vals:  # 循环
            _hash_update_floats(h_geom, (float(uv[0]), float(uv[1])), eps=float_eps)  # 赋值/初始化变量
        st_indices = []  # 赋值/初始化变量
        try:  # 异常处理
            if st_pv.IsIndexed():  # 条件分支
                st_indices = st_pv.GetIndices() or []  # 赋值/初始化变量
        except Exception:  # 异常处理
            st_indices = []  # 赋值/初始化变量
        _hash_update_ints(h_geom, st_indices)  # 执行语句
    else:  # 条件分支
        _hash_update_token(h_geom, "<no_st>")  # 执行语句

    geom_sig_hex = h_geom.hexdigest()  # 赋值/初始化变量

    # --- Transform signatures  # 执行语句
    # World transform at the mesh prim.  # 执行语句
    world_m = xform_cache.GetLocalToWorldTransform(prim)  # 赋值/初始化变量
    m16 = _matrix_to_row_major16(world_m)  # 赋值/初始化变量
    sx, sy, sz = _matrix_scale_xyz(world_m)  # 赋值/初始化变量

    h_scale = _sha256_init("mesh_scale_only_v1")  # 赋值/初始化变量
    _hash_update_str(h_scale, geom_sig_hex)  # 执行语句
    _hash_update_floats(h_scale, (sx, sy, sz), eps=float_eps)  # 赋值/初始化变量

    h_full = _sha256_init("mesh_full_matrix_v1")  # 赋值/初始化变量
    _hash_update_str(h_full, geom_sig_hex)  # 执行语句
    _hash_update_floats(h_full, m16, eps=float_eps)  # 赋值/初始化变量

    face_count = int(len(face_vertex_counts))  # 赋值/初始化变量

    return MeshSig(  # 返回结果
        prim_path=str(prim.GetPath()),  # 赋值/初始化变量
        geom_sig_hex=geom_sig_hex,  # 赋值/初始化变量
        scale_sig_hex=h_scale.hexdigest(),  # 赋值/初始化变量
        full_matrix_sig_hex=h_full.hexdigest(),  # 赋值/初始化变量
        vertex_count=int(len(points)),  # 赋值/初始化变量
        face_count=face_count,  # 赋值/初始化变量
        has_normals=has_normals,  # 赋值/初始化变量
        has_st=has_st,  # 赋值/初始化变量
    )  # 执行语句


def _aggregate_asset_sig(mesh_sigs_hex: List[str], *, tag: str) -> str:  # 定义函数
    h = _sha256_init(tag)  # 赋值/初始化变量
    # Sort to make it order-independent (multiset preserved via duplicates in list)  # 执行语句
    for sig in sorted(mesh_sigs_hex):  # 循环
        _hash_update_str(h, sig)  # 执行语句
    return h.hexdigest()  # 返回结果


def _parse_category_uid(assets_root: str, usd_path: str) -> Tuple[Optional[str], Optional[str]]:  # 定义函数
    # Expected: <assets_root>/<category>/<uid>/usd/<uid>.usd  # 执行语句
    rel = os.path.relpath(usd_path, assets_root)  # 赋值/初始化变量
    parts = rel.split(os.sep)  # 赋值/初始化变量
    if len(parts) >= 4 and parts[2] == "usd":  # 条件分支
        return parts[0], parts[1]  # 返回结果
    return None, None  # 返回结果


def _iter_asset_usd_files(assets_root: str) -> Iterable[str]:  # 定义函数
    # GRScenes assets are typically laid out as:  # 执行语句
    #   <assets_root>/<category>/<uid>/usd/<uid>.usd  # 执行语句
    # Scanning explicitly is much faster than a full os.walk on large datasets.  # 执行语句
    try:  # 异常处理
        with os.scandir(assets_root) as cat_it:  # 上下文管理（文件/资源）
            categories = [e for e in cat_it if e.is_dir()]  # 赋值/初始化变量
    except FileNotFoundError:  # 异常处理
        return  # 返回结果

    saw_any = False  # 赋值/初始化变量
    for cat in sorted(categories, key=lambda e: e.name):  # 循环
        try:  # 异常处理
            with os.scandir(cat.path) as uid_it:  # 上下文管理（文件/资源）
                uids = [e for e in uid_it if e.is_dir()]  # 赋值/初始化变量
        except (FileNotFoundError, PermissionError):  # 异常处理
            continue  # 执行语句

        for uid in sorted(uids, key=lambda e: e.name):  # 循环
            usd_dir = os.path.join(uid.path, "usd")  # 赋值/初始化变量
            if not os.path.isdir(usd_dir):  # 条件分支
                continue  # 执行语句
            saw_any = True  # 赋值/初始化变量

            preferred = os.path.join(usd_dir, f"{uid.name}.usd")  # 赋值/初始化变量
            if os.path.isfile(preferred):  # 条件分支
                yield preferred  # 执行语句
                continue  # 执行语句

            try:  # 异常处理
                with os.scandir(usd_dir) as usd_it:  # 上下文管理（文件/资源）
                    for f in usd_it:  # 循环
                        if f.is_file() and f.name.lower().endswith(".usd"):  # 条件分支
                            yield f.path  # 执行语句
            except (FileNotFoundError, PermissionError):  # 异常处理
                continue  # 执行语句

    # Fallback: if the dataset layout differs, use a full walk.  # 执行语句
    if not saw_any:  # 条件分支
        for dirpath, _, filenames in os.walk(assets_root):  # 循环
            if os.path.basename(dirpath) != "usd":  # 条件分支
                continue  # 执行语句
            for fn in filenames:  # 循环
                if fn.lower().endswith(".usd"):  # 条件分支
                    yield os.path.join(dirpath, fn)  # 执行语句


def _make_duplicates_map(records: List[AssetRecord], key: str) -> Dict[str, List[str]]:  # 定义函数
    out: Dict[str, List[str]] = {}  # 赋值/初始化变量
    for r in records:  # 循环
        sig = getattr(r, key)  # 赋值/初始化变量
        out.setdefault(sig, []).append(r.usd_path)  # 执行语句
    return {sig: paths for sig, paths in out.items() if len(paths) > 1}  # 返回结果


def _write_report(  # 定义函数
    out_path: str,  # 执行语句
    *,  # 执行语句
    mode: str,  # 执行语句
    assets_root: str,  # 执行语句
    dataset: str,  # 执行语句
    float_eps: float,  # 执行语句
    records: List[AssetRecord],  # 执行语句
    errors: List[Dict[str, str]],  # 执行语句
    started_at: float,  # 执行语句
) -> None:  # 执行语句
    duplicates_key = {  # 赋值/初始化变量
        "geom_only": "asset_geom_sig_hex",  # 执行语句
        "scale_only": "asset_scale_sig_hex",  # 执行语句
        "full_matrix": "asset_full_matrix_sig_hex",  # 执行语句
    }[mode]  # 执行语句

    dups = _make_duplicates_map(records, duplicates_key)  # 赋值/初始化变量

    payload = {  # 赋值/初始化变量
        "meta": {  # 执行语句
            "dataset": dataset,  # 执行语句
            "mode": mode,  # 执行语句
            "assets_root": os.path.abspath(assets_root),  # 执行语句
            "float_quantize_eps": float_eps,  # 执行语句
            "asset_usd_count": len(records),  # 执行语句
            "error_count": len(errors),  # 执行语句
            "duplicate_group_count": len(dups),  # 执行语句
            "generated_at_unix": int(time.time()),  # 执行语句
            "elapsed_sec": time.time() - started_at,  # 执行语句
        },  # 执行语句
        "duplicates": [  # 执行语句
            {"sig": sig, "count": len(paths), "usd_paths": paths}  # 执行语句
            for sig, paths in sorted(dups.items(), key=lambda kv: (-len(kv[1]), kv[0]))  # 循环
        ],  # 执行语句
        "assets": [  # 执行语句
            {  # 执行语句
                "usd_path": r.usd_path,  # 执行语句
                "category": r.category,  # 执行语句
                "uid": r.uid,  # 执行语句
                "mesh_count": r.mesh_count,  # 执行语句
                "asset_geom_sig": r.asset_geom_sig_hex,  # 执行语句
                "asset_scale_sig": r.asset_scale_sig_hex,  # 执行语句
                "asset_full_matrix_sig": r.asset_full_matrix_sig_hex,  # 执行语句
                "meshes": [  # 执行语句
                    {  # 执行语句
                        "prim_path": m.prim_path,  # 执行语句
                        "geom_sig": m.geom_sig_hex,  # 执行语句
                        "scale_sig": m.scale_sig_hex,  # 执行语句
                        "full_matrix_sig": m.full_matrix_sig_hex,  # 执行语句
                        "vertex_count": m.vertex_count,  # 执行语句
                        "face_count": m.face_count,  # 执行语句
                        "has_normals": m.has_normals,  # 执行语句
                        "has_st": m.has_st,  # 执行语句
                    }  # 执行语句
                    for m in r.meshes  # 循环
                ],  # 执行语句
            }  # 执行语句
            for r in records  # 循环
        ],  # 执行语句
        "errors": errors,  # 执行语句
    }  # 执行语句

    os.makedirs(os.path.dirname(out_path), exist_ok=True)  # 赋值/初始化变量
    with open(out_path, "w", encoding="utf-8") as f:  # 上下文管理（文件/资源）
        json.dump(payload, f, indent=2, ensure_ascii=False)  # 赋值/初始化变量


def main() -> int:  # 定义函数
    parser = argparse.ArgumentParser()  # 赋值/初始化变量
    parser.add_argument(  # 执行语句
        "--assets-root",  # 执行语句
        required=True,  # 赋值/初始化变量
        help="Assets root directory (e.g. GRScenes-test1/GRScenes_assets)",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--out-dir",  # 执行语句
        default="check_reports",  # 赋值/初始化变量
        help="Output directory for JSON reports",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--dataset",  # 执行语句
        default="test1",  # 赋值/初始化变量
        help="Dataset label to embed in reports/filenames",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--float-quantize-eps",  # 执行语句
        type=float,  # 赋值/初始化变量
        default=0.0,  # 赋值/初始化变量
        help="Quantize floats to this epsilon before hashing (0 disables)",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--limit",  # 执行语句
        type=int,  # 赋值/初始化变量
        default=0,  # 赋值/初始化变量
        help="Optional limit on number of USD files (0 = no limit)",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--progress-every",  # 执行语句
        type=int,  # 赋值/初始化变量
        default=100,  # 赋值/初始化变量
        help="Print/write progress every N files (0 disables)",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--progress-jsonl",  # 执行语句
        default=None,  # 赋值/初始化变量
        help="Optional JSONL progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.jsonl)",  # 赋值/初始化变量
    )  # 执行语句
    parser.add_argument(  # 执行语句
        "--progress-json",  # 执行语句
        default=None,  # 赋值/初始化变量
        help="Optional JSON progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.json)",  # 赋值/初始化变量
    )  # 执行语句
    args = parser.parse_args()  # 赋值/初始化变量

    assets_root = args.assets_root  # 赋值/初始化变量
    out_dir = args.out_dir  # 赋值/初始化变量
    dataset = args.dataset  # 赋值/初始化变量
    float_eps = float(args.float_quantize_eps)  # 赋值/初始化变量

    started_at = time.time()  # 赋值/初始化变量

    print("Starting asset mesh dedup report...", flush=True)  # 打印日志/进度

    progress_jsonl = args.progress_jsonl  # 赋值/初始化变量
    if not progress_jsonl:  # 条件分支
        progress_jsonl = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.jsonl")  # 赋值/初始化变量
    progress_json = args.progress_json  # 赋值/初始化变量
    if not progress_json:  # 条件分支
        progress_json = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.json")  # 赋值/初始化变量
    os.makedirs(os.path.dirname(progress_jsonl), exist_ok=True)  # 赋值/初始化变量

    records: List[AssetRecord] = []  # 赋值/初始化变量
    errors: List[Dict[str, str]] = []  # 赋值/初始化变量

    total = 0  # 赋值/初始化变量

    def _emit_progress(processed: int, *, phase: str, current_usd: Optional[str] = None) -> None:  # 定义函数
        if not args.progress_every or args.progress_every <= 0:  # 条件分支
            return  # 返回结果
        elapsed = max(1e-9, time.time() - started_at)  # 赋值/初始化变量
        rate = processed / elapsed  # 赋值/初始化变量
        if total <= 0:  # 条件分支
            eta_sec = None  # 赋值/初始化变量
        else:  # 条件分支
            remaining = max(0, total - processed)  # 赋值/初始化变量
            eta_sec = remaining / rate if rate > 0 else None  # 赋值/初始化变量
        payload = {  # 赋值/初始化变量
            "dataset": dataset,  # 执行语句
            "assets_root": os.path.abspath(assets_root),  # 执行语句
            "phase": phase,  # 执行语句
            "processed": processed,  # 执行语句
            "total": total,  # 执行语句
            "errors": len(errors),  # 执行语句
            "current_usd": current_usd,  # 执行语句
            "elapsed_sec": elapsed,  # 执行语句
            "rate_files_per_sec": rate,  # 执行语句
            "eta_sec": eta_sec,  # 执行语句
            "timestamp_unix": int(time.time()),  # 执行语句
        }  # 执行语句
        with open(progress_jsonl, "a", encoding="utf-8") as f:  # 上下文管理（文件/资源）
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")  # 赋值/初始化变量
        with open(progress_json, "w", encoding="utf-8") as f:  # 上下文管理（文件/资源）
            json.dump(payload, f, indent=2, ensure_ascii=False)  # 赋值/初始化变量
        eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"  # 赋值/初始化变量
        cur = "" if not current_usd else f" | cur={os.path.basename(current_usd)}"  # 赋值/初始化变量
        print(  # 打印日志/进度
            f"  {phase} {processed}/{total} | errors={len(errors)} | rate={rate:.2f}/s | eta={eta_msg}{cur}",  # 赋值/初始化变量
            flush=True,  # 赋值/初始化变量
        )  # 执行语句

    # Discover asset USDs  # 执行语句
    print(f"Listing asset USDs under: {os.path.abspath(assets_root)}", flush=True)  # 打印日志/进度
    _emit_progress(0, phase="discover")  # 赋值/初始化变量
    usd_files: List[str] = []  # 赋值/初始化变量
    discover_every = 0  # 赋值/初始化变量
    if args.progress_every and args.progress_every > 0:  # 条件分支
        # Don't spam during discovery; discovery can be very long on large datasets.  # 执行语句
        discover_every = max(args.progress_every * 20, 2000)  # 赋值/初始化变量
    for i, p in enumerate(_iter_asset_usd_files(assets_root), start=1):  # 循环
        usd_files.append(p)  # 执行语句
        if discover_every and i % discover_every == 0:  # 条件分支
            _emit_progress(i, phase="discover")  # 赋值/初始化变量
    usd_files.sort()  # 执行语句
    if args.limit and args.limit > 0:  # 条件分支
        usd_files = usd_files[: args.limit]  # 赋值/初始化变量

    total = len(usd_files)  # 赋值/初始化变量
    print(f"Scanning {total} asset USDs...", flush=True)  # 打印日志/进度
    # Emit an initial progress entry so users can 'watch' immediately.  # 执行语句
    _emit_progress(0, phase="scan")  # 赋值/初始化变量

    for idx, usd_path in enumerate(usd_files, start=1):  # 循环
        # Emit an early progress record when starting the first file so users  # 执行语句
        # can see which file we're on even if Stage.Open is slow.  # 执行语句
        if idx == 1:  # 条件分支
            _emit_progress(1, phase="scan", current_usd=usd_path)  # 赋值/初始化变量
        if args.progress_every and args.progress_every > 0 and idx % args.progress_every == 0:  # 条件分支
            _emit_progress(idx, phase="scan", current_usd=usd_path)  # 赋值/初始化变量

        try:  # 异常处理
            stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)  # 赋值/初始化变量
            if stage is None:  # 条件分支
                raise RuntimeError("Usd.Stage.Open returned None")  # 抛出异常/退出

            xform_cache = UsdGeom.XformCache()  # 赋值/初始化变量
            mesh_sigs: List[MeshSig] = []  # 赋值/初始化变量
            for prim in stage.Traverse():  # 循环
                if not prim.IsA(UsdGeom.Mesh):  # 条件分支
                    continue  # 执行语句
                mesh = UsdGeom.Mesh(prim)  # 赋值/初始化变量
                mesh_sigs.append(_compute_mesh_sigs(mesh, xform_cache, float_eps=float_eps))  # 赋值/初始化变量

            category, uid = _parse_category_uid(assets_root, usd_path)  # 赋值/初始化变量

            asset_geom_sig = _aggregate_asset_sig([m.geom_sig_hex for m in mesh_sigs], tag="asset_geom_only_v1")  # 赋值/初始化变量
            asset_scale_sig = _aggregate_asset_sig(  # 赋值/初始化变量
                [m.scale_sig_hex for m in mesh_sigs], tag="asset_scale_only_v1"  # 赋值/初始化变量
            )  # 执行语句
            asset_full_sig = _aggregate_asset_sig(  # 赋值/初始化变量
                [m.full_matrix_sig_hex for m in mesh_sigs], tag="asset_full_matrix_v1"  # 赋值/初始化变量
            )  # 执行语句

            records.append(  # 执行语句
                AssetRecord(  # 执行语句
                    usd_path=usd_path,  # 赋值/初始化变量
                    category=category,  # 赋值/初始化变量
                    uid=uid,  # 赋值/初始化变量
                    mesh_count=len(mesh_sigs),  # 赋值/初始化变量
                    asset_geom_sig_hex=asset_geom_sig,  # 赋值/初始化变量
                    asset_scale_sig_hex=asset_scale_sig,  # 赋值/初始化变量
                    asset_full_matrix_sig_hex=asset_full_sig,  # 赋值/初始化变量
                    meshes=mesh_sigs,  # 赋值/初始化变量
                )  # 执行语句
            )  # 执行语句
        except Exception as e:  # 异常处理
            errors.append({"usd_path": usd_path, "error": str(e)})  # 执行语句

    # Write three reports (same asset list, different duplicates perspective)  # 执行语句
    out_geom = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_geom_only.json")  # 赋值/初始化变量
    out_scale = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_scale_only.json")  # 赋值/初始化变量
    out_full = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_full_matrix.json")  # 赋值/初始化变量

    _write_report(  # 执行语句
        out_geom,  # 执行语句
        mode="geom_only",  # 赋值/初始化变量
        assets_root=assets_root,  # 赋值/初始化变量
        dataset=dataset,  # 赋值/初始化变量
        float_eps=float_eps,  # 赋值/初始化变量
        records=records,  # 赋值/初始化变量
        errors=errors,  # 赋值/初始化变量
        started_at=started_at,  # 赋值/初始化变量
    )  # 执行语句
    _write_report(  # 执行语句
        out_scale,  # 执行语句
        mode="scale_only",  # 赋值/初始化变量
        assets_root=assets_root,  # 赋值/初始化变量
        dataset=dataset,  # 赋值/初始化变量
        float_eps=float_eps,  # 赋值/初始化变量
        records=records,  # 赋值/初始化变量
        errors=errors,  # 赋值/初始化变量
        started_at=started_at,  # 赋值/初始化变量
    )  # 执行语句
    _write_report(  # 执行语句
        out_full,  # 执行语句
        mode="full_matrix",  # 赋值/初始化变量
        assets_root=assets_root,  # 赋值/初始化变量
        dataset=dataset,  # 赋值/初始化变量
        float_eps=float_eps,  # 赋值/初始化变量
        records=records,  # 赋值/初始化变量
        errors=errors,  # 赋值/初始化变量
        started_at=started_at,  # 赋值/初始化变量
    )  # 执行语句

    _emit_progress(total, phase="finalize")  # 赋值/初始化变量

    print("Wrote reports:", flush=True)  # 打印日志/进度
    print(f"  {out_geom}", flush=True)  # 打印日志/进度
    print(f"  {out_scale}", flush=True)  # 打印日志/进度
    print(f"  {out_full}", flush=True)  # 打印日志/进度
    if errors:  # 条件分支
        print(f"WARNING: {len(errors)} files had errors (see report errors[])", flush=True)  # 打印日志/进度

    return 0  # 返回结果


if __name__ == "__main__":  # 脚本入口判断
    raise SystemExit(main())  # 抛出异常/退出
