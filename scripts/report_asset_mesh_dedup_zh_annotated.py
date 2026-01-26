#+#+#+#+#+#+#+#+#+#+#+#+############################################################
# -*- coding: utf-8 -*-
# 说明：这是 scripts/report_asset_mesh_dedup.py 的“逐行中文注释版”（人工通读后逐行补注）。
# 目的：把“这行在做什么/为什么要这么做”写清楚，便于你快速理解脚本逻辑与设计取舍。
# 注意：这是阅读版，不建议直接运行；实际跑数据请用原脚本 scripts/report_asset_mesh_dedup.py。
# 对应导读文档：docs/operations/asset_mesh_dedup_code_guide.md
###############################################################################

#!/usr/bin/env python3  # shebang：让系统用 python3 解释器执行本文件
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

from __future__ import annotations  # 让类型注解以字符串形式延迟求值（减少循环依赖/导入开销）

import argparse  # 解析命令行参数（--assets-root/--dataset 等）
import hashlib  # sha256：生成稳定签名（hash）
import json  # 读写报告 JSON / 进度 JSONL
import math  # 向量长度 sqrt 等数学运算
import os  # 扫目录、拼路径、判断文件是否存在
import struct  # 把 int/float 稳定打包成字节（避免字符串拼接歧义）
import time  # 计时、生成报告时间戳、ETA
from dataclasses import dataclass  # 用 dataclass 写“结构化记录”，减少样板代码
from typing import Dict, Iterable, List, Optional, Sequence, Tuple  # 类型注解：只影响可读性/静态检查

from pxr import Gf, Usd, UsdGeom  # USD Python API：读 Stage/Prim/Mesh/Xform


@dataclass(frozen=True)  # frozen=True：让记录不可变，避免后续误改导致签名与内容不一致
class MeshSig:  # Mesh 级别的“指纹”：一个 UsdGeom.Mesh prim 对应一条
    prim_path: str  # Mesh prim 的路径（例如 /World/Model/mesh），用于定位问题
    geom_sig_hex: str  # 仅几何签名：只看 points/拓扑/normals/UV 等
    scale_sig_hex: str  # scale_only 签名：geom_sig + world scale (sx,sy,sz)
    full_matrix_sig_hex: str  # full_matrix 签名：geom_sig + world 4×4 矩阵（更严格）
    vertex_count: int  # 顶点数（用于快速 sanity check/统计）
    face_count: int  # 面数（通过 faceVertexCounts 统计）
    has_normals: bool  # 是否存在 normals（有些资产不带）
    has_st: bool  # 是否存在 UV primvar “st”（有些资产不带）


@dataclass(frozen=True)  # 资产记录：把一个 USD 文件中的所有 mesh 指纹聚合起来
class AssetRecord:  # Asset 级别“指纹”：一个资产 USD 文件对应一条
    usd_path: str  # 资产 USD 的文件路径（报告里的主索引）
    category: Optional[str]  # 类别名（从路径解析；解析失败则 None）
    uid: Optional[str]  # uid（从路径解析；解析失败则 None）
    mesh_count: int  # 该 USD 内 Mesh prim 的数量
    asset_geom_sig_hex: str  # asset 级 geom_only 签名（把所有 mesh 的 geom_sig 排序后聚合）
    asset_scale_sig_hex: str  # asset 级 scale_only 签名（把所有 mesh 的 scale_sig 排序后聚合）
    asset_full_matrix_sig_hex: str  # asset 级 full_matrix 签名（把所有 mesh 的 full_matrix_sig 排序后聚合）
    meshes: List[MeshSig]  # 明细：该资产里每个 mesh 的签名与统计


def _sha256_init(tag: str) -> "hashlib._Hash":  # 初始化一个 sha256，并把“用途 tag”写进去（防止跨用途碰撞）
    h = hashlib.sha256()  # 创建 sha256 哈希对象
    h.update(tag.encode("utf-8"))  # 先写入 tag：相当于把 hash 版本/用途绑定
    h.update(b"\0")  # 写入分隔符：避免 tag 与后续内容拼接产生歧义
    return h  # 返回可继续 update 的哈希对象


def _hash_update_str(h: "hashlib._Hash", s: str) -> None:  # 把字符串稳定地写入 hash（长度 + bytes）
    data = s.encode("utf-8")  # 统一用 UTF-8 编码成 bytes
    h.update(struct.pack("<I", len(data)))  # 先写入长度（uint32，小端）防止 ["ab","c"] 与 ["a","bc"] 歧义
    h.update(data)  # 再写入实际字符串字节


def _hash_update_token(h: "hashlib._Hash", token: Optional[str]) -> None:  # token 可能为 None，把它规范化后写入
    _hash_update_str(h, "" if token is None else str(token))  # None -> 空串，其他值 -> 字符串


def _hash_update_ints(h: "hashlib._Hash", values: Sequence[int]) -> None:  # 把整数数组稳定写入 hash（长度 + int32 序列）
    h.update(struct.pack("<I", len(values)))  # 先写长度，保证边界明确
    for v in values:  # 逐个写入
        h.update(struct.pack("<i", int(v)))  # 写入 int32（小端），形成稳定字节序


def _quantize(v: float, eps: float) -> float:  # 浮点量化：减少浮点噪声导致的 hash 不稳定
    if eps <= 0:  # eps<=0 表示禁用量化
        return float(v)  # 原样返回
    return round(float(v) / eps) * eps  # 以 eps 为网格四舍五入（例如 eps=1e-6）


def _hash_update_floats(h: "hashlib._Hash", values: Sequence[float], *, eps: float) -> None:  # 把浮点数组写入 hash（可选先量化）
    h.update(struct.pack("<I", len(values)))  # 先写长度
    for v in values:  # 逐个写入
        h.update(struct.pack("<d", _quantize(float(v), eps)))  # 以 float64 小端写入（比字符串更稳定）


def _matrix_to_row_major16(m: Gf.Matrix4d) -> List[float]:  # 把 4×4 矩阵展开成 16 个数（行主序）
    # 备注：Gf.Matrix4d 支持 m[row][col] 访问  # 说明索引方式
    return [float(m[r][c]) for r in range(4) for c in range(4)]  # 生成 [m00,m01,...,m33]


def _matrix_scale_xyz(m: Gf.Matrix4d) -> Tuple[float, float, float]:  # 从 world matrix 提取近似 scale（sx,sy,sz）
    # 思路：取 3×3 部分的三列向量，长度即对应轴向缩放（典型 TRS 变换成立）  # 解释为何这么算
    # 注意：如果存在 shear（剪切），会被“折叠”进长度里；这里不做严格分解  # 说明局限性
    c0 = (float(m[0][0]), float(m[1][0]), float(m[2][0]))  # x 轴 basis 列向量
    c1 = (float(m[0][1]), float(m[1][1]), float(m[2][1]))  # y 轴 basis 列向量
    c2 = (float(m[0][2]), float(m[1][2]), float(m[2][2]))  # z 轴 basis 列向量

    def _len3(v: Tuple[float, float, float]) -> float:  # 计算 3D 向量长度（L2 norm）
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])  # sqrt(x^2+y^2+z^2)

    return (_len3(c0), _len3(c1), _len3(c2))  # 返回 (sx, sy, sz)


def _get_st_primvar(mesh_prim: Usd.Prim) -> Optional[UsdGeom.Primvar]:  # 获取 mesh 的 UV primvar（约定名 st）
    pv_api = UsdGeom.PrimvarsAPI(mesh_prim)  # 通过 PrimvarsAPI 访问 primvar 数据
    pv = pv_api.GetPrimvar("st")  # 读取名为 st 的 primvar（业界常用 UV 名）
    if not pv or not pv.HasValue():  # 不存在或没值，则按“无 UV”处理
        return None  # 返回 None 表示没有可用 UV
    return pv  # 返回 primvar 对象（后续可读 interpolation/values/indices）


def _compute_mesh_sigs(  # 从一个 UsdGeom.Mesh 计算三种口径的 mesh 签名（geom/scale_only/full_matrix）
    mesh: UsdGeom.Mesh,  # 输入：UsdGeom.Mesh schema（封装了 points/拓扑/normals 等属性）
    xform_cache: UsdGeom.XformCache,  # 输入：XformCache（复用以加速求 world transform）
    *,  # 之后的参数只能用关键字传入，避免调用时位置参数搞混
    float_eps: float,  # 浮点量化 eps（用于减少浮点噪声导致的 hash 抖动）
) -> MeshSig:  # 输出：MeshSig（这一个 mesh prim 的指纹）
    prim = mesh.GetPrim()  # 拿到对应的 Usd.Prim（后续用于取路径/属性/transform）

    # --- Geometry signature (ignores transforms)  # 几何签名：只看几何内容，不看 transform
    h_geom = _sha256_init("mesh_geom_v1")  # 几何签名版本号：方便未来升级算法而不混淆旧结果

    points = mesh.GetPointsAttr().Get() or []  # 顶点坐标数组（核心几何）
    face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get() or []  # 每个面的顶点数（拓扑结构的一部分）
    face_vertex_indices = mesh.GetFaceVertexIndicesAttr().Get() or []  # 每个面的顶点索引（拓扑结构的一部分）

    _hash_update_ints(h_geom, face_vertex_counts)  # 先写 counts：拓扑必须一致
    _hash_update_ints(h_geom, face_vertex_indices)  # 再写 indices：拓扑必须一致

    h_geom.update(struct.pack("<I", len(points)))  # 写入顶点数量（防止边界歧义）
    for p in points:  # 遍历每个顶点
        # p 通常是 Gf.Vec3f / Gf.Vec3d（可下标访问）  # 说明 USD 返回的数据类型
        _hash_update_floats(h_geom, (float(p[0]), float(p[1]), float(p[2])), eps=float_eps)  # 写入 (x,y,z)

    subdivision_scheme = mesh.GetSubdivisionSchemeAttr().Get()  # 细分方案（如 catmullClark/none），会影响“几何解释”
    _hash_update_token(h_geom, subdivision_scheme)  # 把细分方案纳入签名，避免不同 scheme 被判成相同

    double_sided = prim.GetAttribute("doubleSided").Get()  # doubleSided 会影响渲染/法线意义，有时也影响资产语义
    if double_sided is None:  # 属性不存在时，按 USD 常见默认 false 处理
        double_sided = False  # 显式设默认，避免 None 参与 hash
    h_geom.update(struct.pack("<?", bool(double_sided)))  # 写入 bool（1 字节）

    has_normals = False  # 标记：是否有 normals
    normals = mesh.GetNormalsAttr().Get() or []  # normals 数组（可能为空）
    if normals:  # 有 normals 才纳入签名
        has_normals = True  # 记录存在 normals
        normals_interp = None  # normals 的插值方式（vertex/varying/faceVarying 等），不同插值语义不同
        if hasattr(mesh, "GetNormalsInterpolation"):  # 兼容：某些版本/绑定可能没有该方法
            try:  # 插值查询可能抛异常（坏 USD 或 schema 不完整）
                normals_interp = mesh.GetNormalsInterpolation()  # 读取 normals interpolation
            except Exception:  # 异常时退回 None（仍然可继续跑）
                normals_interp = None  # 失败则不写具体值
        _hash_update_token(h_geom, normals_interp)  # 把 interpolation 纳入签名
        h_geom.update(struct.pack("<I", len(normals)))  # 写入 normals 数量
        for n in normals:  # 逐个写入 normals
            _hash_update_floats(h_geom, (float(n[0]), float(n[1]), float(n[2])), eps=float_eps)  # 写入 (nx,ny,nz)
    else:  # 没有 normals
        _hash_update_token(h_geom, "<no_normals>")  # 写入占位，避免“缺失”与“空数组”歧义

    has_st = False  # 标记：是否有 UV(st)
    st_pv = _get_st_primvar(prim)  # 读取 st primvar（UV）
    if st_pv is not None:  # 有 UV 才纳入签名
        has_st = True  # 记录存在 UV
        _hash_update_token(h_geom, st_pv.GetInterpolation())  # 把 UV 插值方式纳入签名
        _hash_update_ints(h_geom, [int(st_pv.GetElementSize())])  # elementSize（比如 2）也纳入签名
        st_vals = st_pv.Get() or []  # UV 值数组
        h_geom.update(struct.pack("<I", len(st_vals)))  # 写入 UV 值数量
        for uv in st_vals:  # 逐个写入 (u,v)
            _hash_update_floats(h_geom, (float(uv[0]), float(uv[1])), eps=float_eps)  # 写入 UV 坐标
        st_indices = []  # indexed primvar 的 indices（如果不是 indexed，则保持空）
        try:  # IsIndexed()/GetIndices() 可能因 USD 数据不一致抛异常
            if st_pv.IsIndexed():  # 如果 UV 是 indexed primvar
                st_indices = st_pv.GetIndices() or []  # 读出 indices（拓扑相关）
        except Exception:  # 容错：异常则当作无 indices
            st_indices = []  # 置空以继续运行
        _hash_update_ints(h_geom, st_indices)  # 把 indices 纳入签名（indexed 与非 indexed 不能混）
    else:  # 没有 UV
        _hash_update_token(h_geom, "<no_st>")  # 写入占位，避免“缺失”与“空 primvar”歧义

    geom_sig_hex = h_geom.hexdigest()  # 得到几何签名（hex 字符串，便于 JSON 存储）

    # --- Transform signatures  # transform 相关签名：在 geom_sig 基础上叠加变换信息
    # 这里用“world transform at mesh prim”，能区分同几何但摆放不同的情况  # 解释为什么取 world
    world_m = xform_cache.GetLocalToWorldTransform(prim)  # 取 mesh prim 的 local->world 4×4 矩阵
    m16 = _matrix_to_row_major16(world_m)  # 展开成 16 个 float，方便写入 hash
    sx, sy, sz = _matrix_scale_xyz(world_m)  # 提取 (sx,sy,sz)：用于 scale_only 口径

    h_scale = _sha256_init("mesh_scale_only_v1")  # scale_only 签名版本号
    _hash_update_str(h_scale, geom_sig_hex)  # 先写入 geom_sig（把几何与 transform 绑定）
    _hash_update_floats(h_scale, (sx, sy, sz), eps=float_eps)  # 再写入 scale（忽略平移/旋转）

    h_full = _sha256_init("mesh_full_matrix_v1")  # full_matrix 签名版本号
    _hash_update_str(h_full, geom_sig_hex)  # 先写入 geom_sig
    _hash_update_floats(h_full, m16, eps=float_eps)  # 再写入完整矩阵（平移/旋转/缩放都敏感）

    face_count = int(len(face_vertex_counts))  # 面数：faceVertexCounts 的长度

    return MeshSig(  # 组装该 mesh prim 的最终记录
        prim_path=str(prim.GetPath()),  # prim 路径字符串化
        geom_sig_hex=geom_sig_hex,  # 几何签名
        scale_sig_hex=h_scale.hexdigest(),  # scale_only 签名（hex）
        full_matrix_sig_hex=h_full.hexdigest(),  # full_matrix 签名（hex）
        vertex_count=int(len(points)),  # 顶点数量
        face_count=face_count,  # 面数量
        has_normals=has_normals,  # 是否有 normals
        has_st=has_st,  # 是否有 UV(st)
    )  # 返回 MeshSig


def _aggregate_asset_sig(mesh_sigs_hex: List[str], *, tag: str) -> str:  # 把一个资产里“所有 mesh 的签名”聚合成 asset 级签名
    h = _sha256_init(tag)  # 创建用于 asset 聚合的 hash（tag 区分 geom/scale/full 三种口径）
    # 关键点：排序后写入，保证与 mesh 遍历顺序无关；重复项仍会被保留（multiset）  # 解释排序的意义
    for sig in sorted(mesh_sigs_hex):  # 按字典序遍历签名列表
        _hash_update_str(h, sig)  # 将每个 mesh 的签名写入 asset hash
    return h.hexdigest()  # 返回 asset 级签名（hex 字符串）


def _parse_category_uid(assets_root: str, usd_path: str) -> Tuple[Optional[str], Optional[str]]:  # 从文件路径解析 (category, uid)
    # 期望路径形态：<assets_root>/<category>/<uid>/usd/<uid>.usd（或同目录其他 .usd）  # 路径约定
    rel = os.path.relpath(usd_path, assets_root)  # 转成相对路径，便于按目录层级解析
    parts = rel.split(os.sep)  # 拆分为路径片段
    if len(parts) >= 4 and parts[2] == "usd":  # 满足 <category>/<uid>/usd/... 的结构
        return parts[0], parts[1]  # 返回 category 与 uid
    return None, None  # 解析失败：返回 None（报告里仍保留 usd_path）


def _iter_asset_usd_files(assets_root: str) -> Iterable[str]:  # 枚举 assets_root 下所有“资产主 USD”文件路径
    # GRScenes 典型布局：<assets_root>/<category>/<uid>/usd/<uid>.usd  # 明确目录规范
    # 这里优先用 scandir 分层扫描：比全量 os.walk 快很多（资产库巨大时差距明显）  # 解释性能考虑
    try:  # assets_root 可能不存在
        with os.scandir(assets_root) as cat_it:  # 扫描一级：category 目录
            categories = [e for e in cat_it if e.is_dir()]  # 只保留目录项
    except FileNotFoundError:  # 根目录不存在
        return  # 直接结束生成器

    saw_any = False  # 标记：是否发现过符合“category/uid/usd”布局的资产
    for cat in sorted(categories, key=lambda e: e.name):  # 按 category 名排序，保证遍历稳定
        try:  # category 目录可能被删/无权限
            with os.scandir(cat.path) as uid_it:  # 扫描第二级：uid 目录
                uids = [e for e in uid_it if e.is_dir()]  # 只保留目录项
        except (FileNotFoundError, PermissionError):  # 忽略异常目录
            continue  # 跳过该 category

        for uid in sorted(uids, key=lambda e: e.name):  # 按 uid 排序，保证遍历稳定
            usd_dir = os.path.join(uid.path, "usd")  # 约定：uid 下应有 usd 子目录
            if not os.path.isdir(usd_dir):  # 不符合预期布局就跳过
                continue  # 继续下一个 uid
            saw_any = True  # 至少发现一个符合预期布局的资产

            preferred = os.path.join(usd_dir, f"{uid.name}.usd")  # 优先选择与 uid 同名的主 USD
            if os.path.isfile(preferred):  # 主 USD 存在
                yield preferred  # 产出该 USD 路径
                continue  # 不再在目录里找其他 usd

            try:  # 主 USD 不存在时，从 usd_dir 里找任意 .usd 作为兜底
                with os.scandir(usd_dir) as usd_it:  # 扫描 usd 目录
                    for f in usd_it:  # 遍历目录项
                        if f.is_file() and f.name.lower().endswith(".usd"):  # 只取 .usd 文件（大小写不敏感）
                            yield f.path  # 产出找到的 usd 路径
            except (FileNotFoundError, PermissionError):  # 目录被删/无权限
                continue  # 跳过该 uid

    # 兜底：如果整体目录结构不是预期布局（例如多了一层/命名不同），退回全量 walk  # 解释 fallback 的触发条件
    if not saw_any:  # 没发现任何符合预期布局的项
        for dirpath, _, filenames in os.walk(assets_root):  # 全量遍历所有子目录
            if os.path.basename(dirpath) != "usd":  # 只关心名为 usd 的目录（减少噪声）
                continue  # 不是 usd 目录就跳过
            for fn in filenames:  # 遍历该 usd 目录下的文件
                if fn.lower().endswith(".usd"):  # 只取 .usd
                    yield os.path.join(dirpath, fn)  # 产出完整路径


def _make_duplicates_map(records: List[AssetRecord], key: str) -> Dict[str, List[str]]:  # 按指定签名字段分组，得到重复候选
    out: Dict[str, List[str]] = {}  # sig -> [usd_path,...]
    for r in records:  # 遍历每个资产记录
        sig = getattr(r, key)  # 动态取字段：geom_only/scale_only/full_matrix 三选一
        out.setdefault(sig, []).append(r.usd_path)  # 把该资产路径加入对应 sig 的列表
    return {sig: paths for sig, paths in out.items() if len(paths) > 1}  # 只保留 count>=2 的组（真正重复）


def _write_report(  # 将扫描结果写成最终 JSON 报告（包含 meta/duplicates/assets/errors）
    out_path: str,  # 输出文件路径
    *,  # 之后都用关键字参数（避免传参顺序出错）
    mode: str,  # 口径：geom_only / scale_only / full_matrix
    assets_root: str,  # 资产根目录（写入 meta 便于追溯）
    dataset: str,  # 数据集标签（写入 meta & 用于文件名）
    float_eps: float,  # 浮点量化 eps（写入 meta 便于复现）
    records: List[AssetRecord],  # 所有资产的详细记录
    errors: List[Dict[str, str]],  # 扫描期间的错误列表（每项一般包含 usd_path + message）
    started_at: float,  # 启动时间（用于计算耗时）
) -> None:  # 写文件，不返回
    duplicates_key = {  # 不同 mode 对应使用哪个 asset 级签名字段来分组
        "geom_only": "asset_geom_sig_hex",  # 用仅几何签名分组
        "scale_only": "asset_scale_sig_hex",  # 用几何+scale 签名分组
        "full_matrix": "asset_full_matrix_sig_hex",  # 用几何+完整矩阵签名分组
    }[mode]  # 根据 mode 选定字段名

    dups = _make_duplicates_map(records, duplicates_key)  # 生成重复组：sig -> [usd_paths...]

    payload = {  # 最终 JSON 顶层对象
        "meta": {  # 统计与可复现信息
            "dataset": dataset,  # 数据集标签
            "mode": mode,  # 本报告对应的口径
            "assets_root": os.path.abspath(assets_root),  # 绝对路径（避免相对路径歧义）
            "float_quantize_eps": float_eps,  # 浮点量化参数
            "asset_usd_count": len(records),  # 扫描到的资产 USD 数
            "error_count": len(errors),  # 错误数量
            "duplicate_group_count": len(dups),  # 重复组数量（sig 分组后 count>=2）
            "generated_at_unix": int(time.time()),  # 生成时间戳
            "elapsed_sec": time.time() - started_at,  # 总耗时（秒）
        },
        "duplicates": [  # 重复组列表（按组大小降序）
            {"sig": sig, "count": len(paths), "usd_paths": paths}  # 每组：签名 + 数量 + 明细路径
            for sig, paths in sorted(dups.items(), key=lambda kv: (-len(kv[1]), kv[0]))  # 先按 count 降序，再按 sig 稳定排序
        ],
        "assets": [  # 全量资产明细（很大：主要用于追溯与二次分析）
            {
                "usd_path": r.usd_path,  # 资产 USD 文件路径
                "category": r.category,  # 类别（可能为 None）
                "uid": r.uid,  # uid（可能为 None）
                "mesh_count": r.mesh_count,  # mesh 数量
                "asset_geom_sig": r.asset_geom_sig_hex,  # asset 级 geom_only 签名
                "asset_scale_sig": r.asset_scale_sig_hex,  # asset 级 scale_only 签名
                "asset_full_matrix_sig": r.asset_full_matrix_sig_hex,  # asset 级 full_matrix 签名
                "meshes": [  # 该资产内每个 mesh 的明细
                    {
                        "prim_path": m.prim_path,  # prim 路径
                        "geom_sig": m.geom_sig_hex,  # mesh 级 geom_only 签名
                        "scale_sig": m.scale_sig_hex,  # mesh 级 scale_only 签名
                        "full_matrix_sig": m.full_matrix_sig_hex,  # mesh 级 full_matrix 签名
                        "vertex_count": m.vertex_count,  # 顶点数
                        "face_count": m.face_count,  # 面数
                        "has_normals": m.has_normals,  # 是否有 normals
                        "has_st": m.has_st,  # 是否有 UV(st)
                    }
                    for m in r.meshes  # 遍历该资产内所有 mesh
                ],
            }
            for r in records  # 遍历所有资产
        ],
        "errors": errors,  # 错误明细（便于排查个别坏 USD）
    }

    os.makedirs(os.path.dirname(out_path), exist_ok=True)  # 确保输出目录存在
    with open(out_path, "w", encoding="utf-8") as f:  # 以 UTF-8 写 JSON
        json.dump(payload, f, indent=2, ensure_ascii=False)  # ensure_ascii=False：保留中文；indent=2：便于人工阅读


def main() -> int:  # 主入口：解析参数、扫描资产、写三份报告
    parser = argparse.ArgumentParser()  # 创建 CLI 参数解析器
    parser.add_argument(  # 添加 --assets-root 参数（必填）
        "--assets-root",  # 资产根目录（例如 GRScenes-test1/GRScenes_assets）
        required=True,  # 必须提供，否则直接报错
        help="Assets root directory (e.g. GRScenes-test1/GRScenes_assets)",  # help 文本：给用户看的说明
    )
    parser.add_argument(  # 添加 --out-dir 参数（输出目录）
        "--out-dir",  # 报告输出目录
        default="check_reports",  # 默认输出到 check_reports
        help="Output directory for JSON reports",  # help 文本
    )
    parser.add_argument(  # 添加 --dataset 参数（文件名前缀/报告 meta）
        "--dataset",  # 数据集标签
        default="test1",  # 默认标签 test1
        help="Dataset label to embed in reports/filenames",  # help 文本
    )
    parser.add_argument(  # 添加 --float-quantize-eps 参数（控制浮点量化）
        "--float-quantize-eps",  # 浮点量化 eps
        type=float,  # 解析为 float
        default=0.0,  # 0 表示禁用量化（完全按原始 float 写入 hash）
        help="Quantize floats to this epsilon before hashing (0 disables)",  # help 文本
    )
    parser.add_argument(  # 添加 --limit 参数（只扫前 N 个 usd，用于调试）
        "--limit",  # 限制扫描数量
        type=int,  # 解析为 int
        default=0,  # 0 表示不限制
        help="Optional limit on number of USD files (0 = no limit)",  # help 文本
    )
    parser.add_argument(  # 添加 --progress-every 参数（进度输出频率）
        "--progress-every",  # 每处理多少个文件输出一次进度
        type=int,  # 解析为 int
        default=100,  # 默认每 100 个文件更新一次
        help="Print/write progress every N files (0 disables)",  # help 文本
    )
    parser.add_argument(  # 添加 --progress-jsonl 参数（进度历史文件路径）
        "--progress-jsonl",  # JSONL：一行一个进度点，便于事后分析
        default=None,  # None 表示使用默认路径
        help="Optional JSONL progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.jsonl)",  # help 文本
    )
    parser.add_argument(  # 添加 --progress-json 参数（进度快照文件路径）
        "--progress-json",  # JSON：始终覆盖写，便于 watch 实时查看
        default=None,  # None 表示使用默认路径
        help="Optional JSON progress output path (defaults to <out-dir>/<dataset>_asset_mesh_dedup_progress.json)",  # help 文本
    )
    args = parser.parse_args()  # 解析命令行参数

    assets_root = args.assets_root  # 资产根目录
    out_dir = args.out_dir  # 输出目录
    dataset = args.dataset  # 数据集标签
    float_eps = float(args.float_quantize_eps)  # 量化 eps（强制 float，避免 None/字符串）

    started_at = time.time()  # 记录起始时间（用于耗时/ETA）

    print("Starting asset mesh dedup report...", flush=True)  # 打印启动信息（flush 便于 nohup 实时落盘）

    progress_jsonl = args.progress_jsonl  # 用户可自定义进度历史路径
    if not progress_jsonl:  # 没指定就用默认路径
        progress_jsonl = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.jsonl")  # <out-dir>/<dataset>_...jsonl
    progress_json = args.progress_json  # 用户可自定义进度快照路径
    if not progress_json:  # 没指定就用默认路径
        progress_json = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_progress.json")  # <out-dir>/<dataset>_...json
    os.makedirs(os.path.dirname(progress_jsonl), exist_ok=True)  # 确保进度文件目录存在

    records: List[AssetRecord] = []  # 扫描结果：每个资产一条 AssetRecord
    errors: List[Dict[str, str]] = []  # 错误列表：记录哪个 usd 出了什么错

    total = 0  # 总文件数：discover 完成后再填充

    def _emit_progress(processed: int, *, phase: str, current_usd: Optional[str] = None) -> None:  # 写进度文件 + 打印进度行
        if not args.progress_every or args.progress_every <= 0:  # progress-every<=0 表示完全禁用
            return  # 直接返回，不写文件
        elapsed = max(1e-9, time.time() - started_at)  # 已耗时（避免除零）
        rate = processed / elapsed  # 处理速率：files/sec
        if total <= 0:  # total 未知时（discover 阶段）
            eta_sec = None  # ETA 不可计算
        else:  # scan 阶段 total 已知
            remaining = max(0, total - processed)  # 剩余文件数
            eta_sec = remaining / rate if rate > 0 else None  # 预计剩余秒数
        payload = {  # 进度结构：会写入 json / jsonl
            "dataset": dataset,  # 数据集标签
            "assets_root": os.path.abspath(assets_root),  # 根目录绝对路径
            "phase": phase,  # 阶段：discover/scan/finalize
            "processed": processed,  # 已处理数量
            "total": total,  # 总数量（discover 阶段为 0）
            "errors": len(errors),  # 当前错误数
            "current_usd": current_usd,  # 正在处理的 usd（可为空）
            "elapsed_sec": elapsed,  # 已耗时
            "rate_files_per_sec": rate,  # 速率
            "eta_sec": eta_sec,  # ETA（可能为 None）
            "timestamp_unix": int(time.time()),  # 写入时刻
        }
        with open(progress_jsonl, "a", encoding="utf-8") as f:  # JSONL 追加：保留历史
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")  # 一行一个 JSON 对象
        with open(progress_json, "w", encoding="utf-8") as f:  # JSON 覆盖：便于 watch 读最新
            json.dump(payload, f, indent=2, ensure_ascii=False)  # 格式化输出，方便人读
        eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"  # 终端显示：把 ETA 转成分钟
        cur = "" if not current_usd else f" | cur={os.path.basename(current_usd)}"  # 终端显示：只展示文件名更短
        print(  # 打印单行进度，适合 tail -f
            f"  {phase} {processed}/{total} | errors={len(errors)} | rate={rate:.2f}/s | eta={eta_msg}{cur}",  # 一行包含阶段/计数/速率/ETA
            flush=True,  # 立刻刷新输出（nohup/log 场景很重要）
        )

    # Discover asset USDs  # 第一步：只枚举要扫描的 usd 列表（不打开 Stage）
    print(f"Listing asset USDs under: {os.path.abspath(assets_root)}", flush=True)  # 打印资产根目录（便于确认跑的是哪一套数据）
    _emit_progress(0, phase="discover")  # 立刻写一条 discover 进度，避免长时间无输出
    usd_files: List[str] = []  # 收集所有资产 usd 路径
    discover_every = 0  # discover 阶段的进度输出间隔（会比 scan 阶段更大）
    if args.progress_every and args.progress_every > 0:  # 只有启用进度时才计算 discover_every
        # discovery 可能非常长；这里把输出频率降下来，避免 progress.jsonl 爆炸  # 解释为何乘 20
        discover_every = max(args.progress_every * 20, 2000)  # 至少每 2000 个更新一次
    for i, p in enumerate(_iter_asset_usd_files(assets_root), start=1):  # 枚举所有资产 usd
        usd_files.append(p)  # 收集路径
        if discover_every and i % discover_every == 0:  # 到达输出间隔
            _emit_progress(i, phase="discover")  # 写入 discover 进度点
    usd_files.sort()  # 排序：保证运行稳定可复现（并且方便 diff）
    if args.limit and args.limit > 0:  # 若指定 limit
        usd_files = usd_files[: args.limit]  # 截断：只扫前 N 个

    total = len(usd_files)  # discover 完成后，总数确定
    print(f"Scanning {total} asset USDs...", flush=True)  # 打印将要扫描的总数量
    # 先写一条 scan 进度快照，让 watch 立刻看到 phase 从 discover 切到 scan  # 解释初始 emit 的意义
    _emit_progress(0, phase="scan")  # scan 阶段初始进度

    for idx, usd_path in enumerate(usd_files, start=1):  # 第二步：逐个打开 USD 并计算签名
        # 第一个文件可能卡在 Stage.Open；先写一条“当前文件”让用户知道正在干什么  # 解释 idx==1 的特殊处理
        if idx == 1:  # 第一个文件
            _emit_progress(1, phase="scan", current_usd=usd_path)  # 提前写 current_usd，增强可观测性
        if args.progress_every and args.progress_every > 0 and idx % args.progress_every == 0:  # 到达输出间隔
            _emit_progress(idx, phase="scan", current_usd=usd_path)  # 正常周期性更新进度

        try:  # 单个 USD 的失败不应该中断全局扫描
            stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)  # 打开 Stage（LoadNone：尽量不加载 payload，提高速度）
            if stage is None:  # 极端情况：Open 返回 None
                raise RuntimeError("Usd.Stage.Open returned None")  # 抛错并记录到 errors

            xform_cache = UsdGeom.XformCache()  # 用于计算 prim 的 world transform（每个 stage 一个即可）
            mesh_sigs: List[MeshSig] = []  # 收集该资产 USD 内所有 mesh 的签名
            for prim in stage.Traverse():  # 遍历整个 stage 的所有 prim
                if not prim.IsA(UsdGeom.Mesh):  # 只关心 mesh prim
                    continue  # 不是 mesh 就跳过
                mesh = UsdGeom.Mesh(prim)  # 把 prim 包装成 Mesh schema 以读取属性
                mesh_sigs.append(_compute_mesh_sigs(mesh, xform_cache, float_eps=float_eps))  # 计算该 mesh 的三种签名并收集

            category, uid = _parse_category_uid(assets_root, usd_path)  # 从路径解析类别与 uid（便于报告筛选）

            asset_geom_sig = _aggregate_asset_sig([m.geom_sig_hex for m in mesh_sigs], tag="asset_geom_only_v1")  # 聚合：geom_only
            asset_scale_sig = _aggregate_asset_sig(  # 聚合：scale_only
                [m.scale_sig_hex for m in mesh_sigs], tag="asset_scale_only_v1"  # 传入每个 mesh 的 scale_sig
            )
            asset_full_sig = _aggregate_asset_sig(  # 聚合：full_matrix
                [m.full_matrix_sig_hex for m in mesh_sigs], tag="asset_full_matrix_v1"  # 传入每个 mesh 的 full_matrix_sig
            )

            records.append(  # 把这个资产的结果写入 records
                AssetRecord(  # 构造 AssetRecord（不可变记录）
                    usd_path=usd_path,  # 资产 USD 路径
                    category=category,  # 类别
                    uid=uid,  # uid
                    mesh_count=len(mesh_sigs),  # mesh 数量
                    asset_geom_sig_hex=asset_geom_sig,  # asset 级 geom_only 签名
                    asset_scale_sig_hex=asset_scale_sig,  # asset 级 scale_only 签名
                    asset_full_matrix_sig_hex=asset_full_sig,  # asset 级 full_matrix 签名
                    meshes=mesh_sigs,  # mesh 明细
                )
            )
        except Exception as e:  # 捕获任意异常：坏 USD、IO、pxr 抛错等
            errors.append({"usd_path": usd_path, "error": str(e)})  # 记录错误到列表（报告 errors[]）

    # 第三步：写三份报告（同一批 assets，只是“重复分组口径”不同）  # 解释为什么是三份
    out_geom = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_geom_only.json")  # geom_only 报告路径
    out_scale = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_scale_only.json")  # scale_only 报告路径
    out_full = os.path.join(out_dir, f"{dataset}_asset_mesh_dedup_full_matrix.json")  # full_matrix 报告路径

    _write_report(  # 写 geom_only 报告
        out_geom,  # 输出文件
        mode="geom_only",  # 分组口径：只看几何
        assets_root=assets_root,  # 写入 meta
        dataset=dataset,  # 写入 meta
        float_eps=float_eps,  # 写入 meta
        records=records,  # 全量扫描记录
        errors=errors,  # 错误列表
        started_at=started_at,  # 用于计算 elapsed
    )
    _write_report(  # 写 scale_only 报告
        out_scale,  # 输出文件
        mode="scale_only",  # 分组口径：几何 + scale
        assets_root=assets_root,  # 写入 meta
        dataset=dataset,  # 写入 meta
        float_eps=float_eps,  # 写入 meta
        records=records,  # 全量扫描记录
        errors=errors,  # 错误列表
        started_at=started_at,  # 用于计算 elapsed
    )
    _write_report(  # 写 full_matrix 报告
        out_full,  # 输出文件
        mode="full_matrix",  # 分组口径：几何 + world matrix
        assets_root=assets_root,  # 写入 meta
        dataset=dataset,  # 写入 meta
        float_eps=float_eps,  # 写入 meta
        records=records,  # 全量扫描记录
        errors=errors,  # 错误列表
        started_at=started_at,  # 用于计算 elapsed
    )

    _emit_progress(total, phase="finalize")  # 最后写一条 finalize 进度，表示扫描与写文件都结束

    print("Wrote reports:", flush=True)  # 打印输出文件清单
    print(f"  {out_geom}", flush=True)  # geom_only 报告路径
    print(f"  {out_scale}", flush=True)  # scale_only 报告路径
    print(f"  {out_full}", flush=True)  # full_matrix 报告路径
    if errors:  # 如果有错误
        print(f"WARNING: {len(errors)} files had errors (see report errors[])", flush=True)  # 给出警告并提示去 errors[] 查细节

    return 0  # 正常退出码


if __name__ == "__main__":  # Python 约定：只在直接运行该文件时执行
    raise SystemExit(main())  # 把 main() 的返回值作为进程退出码（0=成功，非0=失败）
