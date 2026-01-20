#!/usr/bin/env python3  # 以 python3 运行此脚本

"""Generate a Blender-friendly layout.json from a GRScenes scene layout USD.

从 GRScenes 的场景布局 USD（layout.usd）生成一个 Blender 友好的 layout.json：
它只描述“摆放”（位姿）与“索引”（每个物体对应的 glb 路径），便于 Blender 侧批量导入 GLB 并还原布局。

This script reads a scene layout USD (typically: GRScenes100/<cat>/<scene_uid>_usd/layout.usd),
finds placed object assets under GRScenes_assets/<category>/<uid>/, and writes a JSON file
that indexes the corresponding per-asset GLBs:

    GRScenes_assets/<category>/<uid>/glb/<uid>.glb

Design goals / 设计目标：
- Only targets *object assets* (GRScenes_assets). Scene-only geometry is intentionally ignored.
    只处理物体资产（GRScenes_assets），不补齐场景自身几何/灯光等。
- No coordinate axis conversion (can be added later in the Blender importer if needed).
    不做坐标轴转换（需要时可在 Blender importer 侧统一加）。
- No PointInstancer expansion (per user requirement).
    不展开 PointInstancer（按当前需求不需要）。

Run in an environment with pxr (Isaac Sim python recommended):

    ./scripts/isaac_python.sh scripts/generate_layout_json_from_usd.py \
        --layout-usd sandbox/scene_subset_<SID>/GRScenes100/home/<SID>_usd/layout.usd \
        --subset-root sandbox/scene_subset_<SID>

"""  # 顶层文档字符串：说明用途与使用方式

from __future__ import annotations  # 延迟类型注解求值，避免前向引用问题

import argparse  # 命令行参数解析
import datetime as _dt  # 生成时间戳
import json  # JSON 读写
import os  # 路径/文件系统操作
import re  # 正则解析 uid/category
from dataclasses import dataclass  # 轻量数据结构
from typing import Dict, Iterable, List, Optional, Sequence, Tuple  # 类型注解

try:  # 尝试导入 USD(PXR) 运行时依赖
    from pxr import Gf, Sdf, Usd, UsdGeom  # USD 的核心模块：矩阵、资产路径、Stage、几何变换

    _PXR_IMPORT_ERROR: Optional[Exception] = None  # 记录导入错误（若成功则为 None）
except Exception as e:  # pragma: no cover  # 在无 pxr 环境下的兜底
    Gf = None  # type: ignore[assignment]  # 避免类型检查报错
    Sdf = None  # type: ignore[assignment]  # 避免类型检查报错
    Usd = None  # type: ignore[assignment]  # 避免类型检查报错
    UsdGeom = None  # type: ignore[assignment]  # 避免类型检查报错
    _PXR_IMPORT_ERROR = e  # 保存异常，便于后续报错信息更清晰


@dataclass(frozen=True)  # 不可变数据类，便于作为 dict/set key 使用
class AssetRef:  # 表示一个物体资产引用（category + uid）
    category: str  # 资产类别（例如 chair/desk 等）
    uid: str  # 资产 uid（通常是 hash 字符串）


_GRSCENES_ASSETS_RE = re.compile(r"(?:^|/)GRScenes_assets/([^/]+)/([^/]+)/")  # 从路径中提取 category/uid


def _norm_abs(p: str) -> str:  # 规范化为绝对路径（消除 .. 与多余分隔符）
    return os.path.abspath(os.path.normpath(p))  # 先 normpath 再 abspath


def _posix_relpath(path: str) -> str:  # 将路径分隔符转换为 POSIX 风格（JSON 内统一用 /）
    return path.replace(os.sep, "/")  # Windows/Unix 兼容


def _parse_asset_ref_from_path(path_str: str) -> Optional[AssetRef]:  # 从任意字符串路径中解析 AssetRef
    s = (path_str or "").replace("\\", "/")  # 统一用 / 做匹配
    m = _GRSCENES_ASSETS_RE.search(s)  # 查找 GRScenes_assets/<cat>/<uid>/
    if not m:  # 如果不匹配则返回 None
        return None  # 表示该路径不是物体资产引用
    return AssetRef(category=m.group(1), uid=m.group(2))  # 返回解析得到的 category 与 uid


def _resolve_maybe_relative(base_dir: str, path_str: str) -> str:  # 将相对路径按 base_dir 解析成绝对/规范路径
    if not path_str:  # 空字符串直接返回
        return ""  # 表示无有效路径
    if os.path.isabs(path_str):  # 已经是绝对路径
        return os.path.normpath(path_str)  # 只做规范化
    return os.path.normpath(os.path.join(base_dir, path_str))  # 相对路径则拼到 base_dir


def _find_subset_root_from_layout(layout_usd: str) -> Optional[str]:  # 从 layout.usd 路径推断 subset_root
    """Try to infer subset root by walking parents until we see GRScenes_assets."""  # 说明：向上找 GRScenes_assets

    p = _norm_abs(os.path.dirname(layout_usd))  # 从 layout.usd 所在目录开始向上走
    for _ in range(12):  # 最多向上走 12 层，避免无限循环
        if os.path.isdir(os.path.join(p, "GRScenes_assets")):  # 若该目录下存在 GRScenes_assets
            return p  # 则认为它是 subset_root
        parent = os.path.dirname(p)  # 否则继续向上
        if parent == p:  # 到达根目录则停止
            break  # 防止死循环
        p = parent  # 更新当前目录
    return None  # 未找到则返回 None


def _infer_scene_uid_and_category(layout_usd: str) -> Tuple[Optional[str], Optional[str]]:  # 从路径推断场景 uid 与类别
    """Infer (scene_uid, scene_category) from a path containing GRScenes100/<cat>/<sid>_usd/."""  # 仅从路径约定推断

    parts = _posix_relpath(_norm_abs(layout_usd)).split("/")  # 转为 posix 并按 / 切分
    try:  # 查找 GRScenes100 在路径中的位置
        i = parts.index("GRScenes100")  # GRScenes100/<cat>/<sid>_usd
    except ValueError:  # 不含 GRScenes100
        return None, None  # 则无法推断
    if i + 2 >= len(parts):  # 路径层级不够
        return None, None  # 无法推断
    cat = parts[i + 1]  # scene_category
    scene_dir = parts[i + 2]  # <scene_uid>_usd
    scene_uid = scene_dir  # 初始 scene_uid
    if scene_uid.endswith("_usd"):  # 兼容带 _usd 后缀
        scene_uid = scene_uid[: -len("_usd")]  # 去掉后缀
    return scene_uid, cat  # 返回 (scene_uid, scene_category)


def _iter_asset_paths_from_prim(prim) -> List[str]:  # 从 prim 上收集可能的资产路径（引用/payload/资产属性）
    """Collect asset paths (strings) that may reference a GRScenes asset USD for this prim."""  # 返回候选路径列表

    paths: List[str] = []  # 存放收集到的路径

    # 1) references：最常见的模型引用方式
    if prim.HasAuthoredReferences():  # prim 是否写过 references
        refs = prim.GetMetadata("references")  # 读取 references 元数据
        if refs:  # 若存在
            for ref in refs.GetAddedOrExplicitItems():  # 遍历显式/新增的引用
                ap = getattr(ref, "assetPath", "")  # 取 assetPath 字段
                if ap:  # 非空
                    paths.append(ap)  # 加入候选

    # 2) payloads：一些场景可能通过 payload 引入资产
    if prim.HasAuthoredPayloads():  # prim 是否写过 payloads
        payloads = prim.GetMetadata("payloads")  # 读取 payloads 元数据
        if payloads:  # 若存在
            for pl in payloads.GetAddedOrExplicitItems():  # 遍历显式/新增的 payload
                ap = getattr(pl, "assetPath", "")  # 取 assetPath 字段
                if ap:  # 非空
                    paths.append(ap)  # 加入候选

    # 3) asset-valued attributes (fallback)：兜底扫描所有资产路径属性
    for attr in prim.GetAttributes():  # 遍历 prim 的所有属性
        try:  # 属性读取可能失败
            val = attr.Get()  # 取属性值
        except Exception:  # 失败则跳过
            continue  # 继续下一个属性
        if isinstance(val, Sdf.AssetPath):  # 只关心 Sdf.AssetPath 类型
            ap = val.path  # 取实际路径字符串
            if ap and ap.lower().endswith((".usd", ".usda", ".usdc")):  # 只保留 USD 文件
                paths.append(ap)  # 加入候选

    return paths  # 返回候选路径列表


def _score_candidate_path(path_str: str, ref: AssetRef) -> int:  # 对候选路径打分，选择更像“规范资产 USD”的那条
    s = (path_str or "").replace("\\", "/")  # 标准化分隔符
    score = 0  # 初始得分
    if "/usd/" in s:  # 规范结构含 /usd/
        score += 10  # 加分
    if s.lower().endswith(f"/usd/{ref.uid.lower()}.usd"):  # 更严格匹配规范文件名
        score += 10  # 再加分
    if os.path.basename(s).lower() == f"{ref.uid.lower()}.usd":  # basename 恰好是 <uid>.usd
        score += 5  # 也加分
    return score  # 返回得分


def _choose_best_asset_ref(base_dir: str, raw_paths: Sequence[str]) -> Optional[Tuple[AssetRef, str]]:  # 从多条候选路径中选择最可信的资产引用
    """Choose best (AssetRef, resolved_usd_path) from a list of raw asset paths."""  # 返回 (AssetRef, 解析后的 USD 路径)

    best: Optional[Tuple[int, AssetRef, str]] = None  # (score, ref, resolved_path)

    for p in raw_paths:  # 遍历所有候选路径
        if not p:  # 空路径跳过
            continue  # 继续
        resolved = _resolve_maybe_relative(base_dir, p)  # 将相对路径解析为绝对/规范路径
        for candidate in (p, resolved):  # 同时尝试 raw 与 resolved 两种形态
            ref = _parse_asset_ref_from_path(candidate)  # 尝试解析 category/uid
            if ref is None:  # 不是 GRScenes_assets 路径
                continue  # 跳过
            sc = _score_candidate_path(candidate, ref)  # 对这条路径打分
            if best is None or sc > best[0]:  # 选择得分更高者
                best = (sc, ref, resolved)  # 注意：返回时携带 resolved 便于后续记录

    if best is None:  # 没有任何可用匹配
        return None  # 返回 None
    _, ref, resolved = best  # 解包最佳结果
    return ref, resolved  # 返回 (AssetRef, resolved_usd_path)


def _matrix4d_to_row_major_list(m) -> List[float]:  # 将 Gf.Matrix4d 展开为 row-major 的长度 16 数组
    # Gf.Matrix4d supports m[row][col]  # Gf.Matrix4d 支持通过 [row][col] 索引
    out: List[float] = []  # 输出数组
    for r in range(4):  # 行
        for c in range(4):  # 列
            out.append(float(m[r][c]))  # 按行优先展开
    return out  # 返回 16 个 float


def generate_layout_json(layout_usd: str, subset_root: str, skip_missing_glb: bool, strict: bool) -> Dict[str, object]:  # 核心：layout.usd → layout.json payload
    if _PXR_IMPORT_ERROR is not None:  # 若 pxr 导入失败
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")  # 明确报错，提示需要 Isaac/pxr 环境

    layout_usd = _norm_abs(layout_usd)  # 规范化输入路径
    subset_root = _norm_abs(subset_root)  # 规范化 subset_root

    stage = Usd.Stage.Open(layout_usd)  # 打开 USD Stage
    if not stage:  # 打开失败
        raise RuntimeError(f"Failed to open stage: {layout_usd}")  # 抛错

    scene_uid, scene_category = _infer_scene_uid_and_category(layout_usd)  # 从路径推断 scene 信息（可为空）

    meters_per_unit = None  # 记录 USD 单位（metersPerUnit）
    up_axis = None  # 记录 up axis（通常 Z）
    try:  # 读取 stage metadata：metersPerUnit
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))  # 转成 float
    except Exception:  # 读取失败则忽略
        pass  # 保持 None
    try:  # 读取 stage metadata：up axis
        up_axis = str(UsdGeom.GetStageUpAxis(stage))  # 转为字符串
    except Exception:  # 读取失败则忽略
        pass  # 保持 None

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())  # 用缓存高效计算 world transform

    instances: List[Dict[str, object]] = []  # 输出的实例列表

    base_dir = os.path.dirname(layout_usd)  # layout.usd 所在目录（相对引用的 base）

    for prim in stage.Traverse():  # 遍历全场景 prim
        if not prim.IsValid():  # 无效 prim 跳过
            continue  # 继续

        raw_paths = _iter_asset_paths_from_prim(prim)  # 从 prim 收集引用路径候选
        if not raw_paths:  # 没有任何路径则不是资产实例
            continue  # 继续

        chosen = _choose_best_asset_ref(base_dir, raw_paths)  # 从候选路径中挑最像“资产 USD”的那条
        if chosen is None:  # 没有解析出 category/uid
            continue  # 继续
        asset_ref, resolved_asset_usd = chosen  # 解包 (AssetRef, resolved_asset_usd)

        # Only keep meaningful placements (Xformable prims).  # 只保留可变换 prim（代表摆放）
        try:  # Xformable 检查可能抛异常
            xformable = UsdGeom.Xformable(prim)  # 尝试包装为 Xformable
            if not xformable:  # 包装失败则跳过
                continue  # 继续
        except Exception:  # 异常也跳过
            continue  # 继续

        try:  # 计算世界变换矩阵
            world: Gf.Matrix4d = xform_cache.GetLocalToWorldTransform(prim)  # local→world 4×4
        except Exception:  # 变换计算失败则跳过
            continue  # 继续

        glb_abs = os.path.join(  # 构造该资产对应的 glb 绝对路径
            subset_root, "GRScenes_assets", asset_ref.category, asset_ref.uid, "glb", f"{asset_ref.uid}.glb"  # 规范路径
        )
        glb_rel = _posix_relpath(os.path.relpath(glb_abs, subset_root))  # JSON 内写相对路径（且转 posix）
        glb_exists = os.path.isfile(glb_abs)  # 检查 glb 是否存在

        if strict and not glb_exists:  # strict 模式：缺 glb 就失败
            raise FileNotFoundError(f"Missing GLB for {asset_ref.category}/{asset_ref.uid}: {glb_abs}")  # 抛错
        if skip_missing_glb and not glb_exists:  # skip 模式：缺 glb 则跳过
            continue  # 继续

        inst: Dict[str, object] = {  # 组装单条实例记录
            "prim_path": str(prim.GetPath()),  # USD prim path（用于定位/调试）
            "name": prim.GetName(),  # prim 名称（可用作 Blender 空物体名）
            "category": asset_ref.category,  # 资产类别
            "uid": asset_ref.uid,  # 资产 uid
            "glb": glb_rel,  # 相对 subset_root 的 glb 路径
            "source_asset_usd": _posix_relpath(os.path.relpath(resolved_asset_usd, subset_root))  # 若在 subset 内则转相对
            if resolved_asset_usd.startswith(subset_root)  # 判断 resolved_asset_usd 是否在 subset_root 下
            else resolved_asset_usd,  # 否则保留原始路径（可能是绝对路径）
            "transform": {"matrix_row_major": _matrix4d_to_row_major_list(world)},  # world matrix（row-major）
            "availability": {"glb": glb_exists},  # 简单可用性标记
        }
        instances.append(inst)  # 加入实例列表

    instances.sort(key=lambda x: (x.get("prim_path") or ""))  # 为稳定输出按 prim_path 排序

    counts = {  # 统计信息（便于快速验收）
        "instances_total": len(instances),  # 实例总数
        "instances_with_glb": sum(1 for i in instances if (i.get("availability") or {}).get("glb")),  # 有 glb 的数量
        "instances_missing_glb": sum(1 for i in instances if not (i.get("availability") or {}).get("glb")),  # 缺 glb 的数量
    }

    payload: Dict[str, object] = {  # 顶层 JSON payload（schema v1）
        "schema_version": 1,  # schema 版本
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),  # 生成时间（UTC）
        "subset_root": subset_root,  # subset_root 绝对路径（便于追溯）
        "scene_uid": scene_uid,  # 场景 uid（可能为 None）
        "scene_category": scene_category,  # 场景类别（可能为 None）
        "source_layout_usd": _posix_relpath(os.path.relpath(layout_usd, subset_root)) if layout_usd.startswith(subset_root) else layout_usd,  # layout.usd 来源
        "asset_root": "GRScenes_assets",  # 资产根目录名（约定）
        "units": {"meters_per_unit": meters_per_unit},  # USD 单位元数据
        "up_axis": up_axis,  # up axis 元数据
        "counts": counts,  # 统计
        "instances": instances,  # 实例列表
    }

    return payload  # 返回 payload（由 main 写入 JSON）


def main(argv: Optional[Sequence[str]] = None) -> int:  # CLI 入口
    ap = argparse.ArgumentParser(description="Generate layout.json from a scene layout.usd (GRScenes_assets instances only).")  # 参数解析器
    ap.add_argument("--layout-usd", required=True, help="Path to layout.usd")  # 输入 layout.usd
    ap.add_argument(  # subset_root 参数
        "--subset-root",  # subset_root 路径
        default=None,  # 默认尝试自动推断
        help="Subset/package root that contains GRScenes_assets and GRScenes100. If omitted, inferred from --layout-usd.",  # 说明
    )
    ap.add_argument(  # 输出路径
        "--out",  # 输出 json 路径
        default=None,  # 默认 layout.usd 同级 layout.json
        help="Output JSON path. Default: alongside layout.usd as layout.json",  # 说明
    )
    ap.add_argument(  # 是否跳过缺失 glb
        "--skip-missing-glb",  # 缺 glb 的实例不写入
        action="store_true",  # 布尔开关
        help="Skip instances whose GLB does not exist (instead of emitting them with availability.glb=false).",  # 说明
    )
    ap.add_argument(  # strict 模式
        "--strict",  # 任一缺 glb 直接失败
        action="store_true",  # 布尔开关
        help="Fail if any instance's GLB does not exist.",  # 说明
    )

    args = ap.parse_args(argv)  # 解析参数

    layout_usd = args.layout_usd  # 取输入 layout.usd
    subset_root = args.subset_root  # 取 subset_root
    if subset_root is None:  # 若未提供 subset_root
        inferred = _find_subset_root_from_layout(layout_usd)  # 尝试从 layout.usd 路径向上推断
        if inferred is None:  # 推断失败
            raise SystemExit("Failed to infer --subset-root; please provide it explicitly")  # 提示用户显式提供
        subset_root = inferred  # 使用推断值

    out_path = args.out  # 取输出路径
    if out_path is None:  # 若未提供
        out_path = os.path.join(os.path.dirname(_norm_abs(layout_usd)), "layout.json")  # 默认 layout.usd 同级

    payload = generate_layout_json(  # 生成 payload
        layout_usd=layout_usd,  # 传入 layout.usd
        subset_root=subset_root,  # 传入 subset_root
        skip_missing_glb=bool(args.skip_missing_glb),  # 传入 skip 选项
        strict=bool(args.strict),  # 传入 strict 选项
    )

    os.makedirs(os.path.dirname(_norm_abs(out_path)), exist_ok=True)  # 确保输出目录存在
    with open(out_path, "w", encoding="utf-8") as f:  # 打开输出文件
        json.dump(payload, f, ensure_ascii=False, indent=2)  # 写入 JSON

    print(f"Wrote: {out_path}")  # 打印输出路径
    print(json.dumps(payload.get("counts", {}), ensure_ascii=False, indent=2))  # 打印统计信息
    return 0  # 正常退出


if __name__ == "__main__":  # 作为脚本执行时
    raise SystemExit(main())  # 执行 main 并将返回码作为退出码
