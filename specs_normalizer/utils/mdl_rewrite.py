"""
USD 材质路径改写工具 mdl_rewrite

整体介绍：
- 提供可复用的函数，用于将 USD 文件中对 MDL 材质的引用统一改写为“相对路径”或“绝对路径”。
- 支持同时处理两类常见引用：
  1) 通过 `Sdf.AssetPath` 指向 `.mdl` 文件（例如 `@./Materials/foo.mdl@` 或绝对路径 `/path/.../Materials/foo.mdl`）；
  2) 以模块名字符串形式出现的 `::.::Materials::<name>`（常见于某些生成流程的占位形式）。

使用建议：
- 在 Omniverse Isaac Sim 的 Python 环境下执行（本项目提供 `scripts/isaac_python.sh` 以自动配置 `pxr` 与依赖库环境）。
- 相对路径模式适合“仓库内自足”或“跨目录打开”；绝对路径模式适合“稳定依赖中央材质库”。
"""

import os
from typing import Optional, Dict

from pxr import Usd, Sdf


def _posix_join(base: str, rest: str) -> str:
    """
    将路径拼接并统一为 POSIX 斜杠形式（USD 推荐使用 `/`）。
    同时强制将路径中的 '/Textures/' 替换为 '/textures/' 以解决大小写敏感问题。
    """
    path = os.path.join(base, rest).replace("\\", "/")
    # 强制将 Textures 目录转换为小写 textures
    if "/Textures/" in path:
        path = path.replace("/Textures/", "/textures/")
    return path


def rewrite_usd_mdl_paths(
    src_usd: str,
    dst_usd: str,
    materials_dir: str,
    use_relative: bool = True,
    relative_base: Optional[str] = None,
    rewrite_module_refs: bool = True,
) -> Dict[str, str]:
    """
    改写 USD 文件中的 MDL 引用，并导出到目标路径。

    参数：
    - src_usd: 源 USD 文件的绝对路径。
    - dst_usd: 改写后导出的 USD 文件绝对路径。
    - materials_dir: 目标材质库目录的绝对路径（目录包含 `.mdl` 与可能的 `Textures/`）。
    - use_relative: True 表示改写为“相对 dst_usd 所在目录”的相对路径；False 表示改写为绝对路径。
    - relative_base: 相对路径的基准目录；不提供时默认使用 `os.path.dirname(dst_usd)`。
    - rewrite_module_refs: 是否处理字符串形式的 `::.::Materials::<name>` 为 `.mdl` 文件路径。

    返回：
    - dict，包含：
      - out: 导出的 USD 路径（dst_usd）
      - updated: 改写的属性个数（int 转为 str）
      - mode: "relative" 或 "absolute"
      - materials_base: 用于拼接的材质路径基准（相对前缀或绝对目录）
    """

    # 计算路径基准
    if use_relative:
        base_dir = relative_base or os.path.dirname(dst_usd)
        materials_base = os.path.relpath(materials_dir, base_dir)
    else:
        materials_base = materials_dir

    # 打开源 Stage 与 RootLayer
    stage = Usd.Stage.Open(src_usd)
    root = stage.GetRootLayer()

    updated = 0

    # 遍历所有 Prim 的所有属性，查找并改写材质引用
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            # 性能优化：先按类型过滤，避免对所有属性调用 attr.Get()
            t = attr.GetTypeName()

            # 1) 处理 AssetPath（文件路径型）
            if t == Sdf.ValueTypeNames.Asset:
                val = attr.Get()
                if not isinstance(val, Sdf.AssetPath):
                    continue
                path = val.path
                if not path:
                    continue
                # 统一识别出 'Materials/' 或 'Material/' 子串，并提取其后缀（rest）
                # 优先处理包含 'mdl/' 的长路径以避免重复拼接
                idx = -1
                prefix_len = 0
                
                # 策略1：如果是纹理路径（包含 Textures 或 textures），直接提取文件名并重组
                # 这能最稳健地修复路径重复问题（如 mdl/mdl/textures）和大小写问题
                tex_idx_lower = path.lower().rfind("/textures/")
                if tex_idx_lower >= 0:
                    filename = path[tex_idx_lower + len("/textures/") :]
                    # 直接构造目标路径：materials_base/textures/filename
                    # 注意：materials_base 通常指向 .../Material/mdl
                    newp = _posix_join(materials_base, "textures/" + filename)
                    if newp != path:
                        attr.Set(Sdf.AssetPath(newp))
                        updated += 1
                    continue # 处理完毕，跳过后续逻辑

                # 策略2：非纹理路径（如 .mdl 引用），按原有前缀截断逻辑
                if "Material/mdl/" in path:
                    idx = path.find("Material/mdl/")
                    prefix_len = len("Material/mdl/")
                elif "Materials/" in path:
                    idx = path.find("Materials/")
                    prefix_len = len("Materials/")
                elif "Material/" in path:
                    idx = path.find("Material/")
                    prefix_len = len("Material/")
                
                if idx >= 0:
                    rest = path[idx + prefix_len :]
                    
                    # [修复] 移除剩余路径中可能存在的 mdl/ 前缀，避免与 materials_base 中的 mdl 目录重复
                    if rest.startswith("mdl/"):
                        rest = rest[4:]
                    
                    newp = _posix_join(materials_base, rest)
                    if newp != path:
                        attr.Set(Sdf.AssetPath(newp))
                        updated += 1
                else:
                    # 对于不含 'Materials/' 的路径，保留原值（避免误改其它资产引用）
                    pass
            elif t == Sdf.ValueTypeNames.AssetArray:
                val = attr.Get()
                if not val:
                    continue
                changed = False
                new_vals = []
                for ap in val:
                    if not isinstance(ap, Sdf.AssetPath) or not ap.path:
                        new_vals.append(ap)
                        continue

                    path = ap.path

                    tex_idx_lower = path.lower().rfind("/textures/")
                    if tex_idx_lower >= 0:
                        filename = path[tex_idx_lower + len("/textures/") :]
                        newp = _posix_join(materials_base, "textures/" + filename)
                        if newp != path:
                            changed = True
                            new_vals.append(Sdf.AssetPath(newp))
                        else:
                            new_vals.append(ap)
                        continue

                    idx = -1
                    prefix_len = 0
                    if "Material/mdl/" in path:
                        idx = path.find("Material/mdl/")
                        prefix_len = len("Material/mdl/")
                    elif "Materials/" in path:
                        idx = path.find("Materials/")
                        prefix_len = len("Materials/")
                    elif "Material/" in path:
                        idx = path.find("Material/")
                        prefix_len = len("Material/")

                    if idx >= 0:
                        rest = path[idx + prefix_len :]
                        if rest.startswith("mdl/"):
                            rest = rest[4:]
                        newp = _posix_join(materials_base, rest)
                        if newp != path:
                            changed = True
                            new_vals.append(Sdf.AssetPath(newp))
                        else:
                            new_vals.append(ap)
                    else:
                        new_vals.append(ap)

                if changed:
                    attr.Set(new_vals)
                    updated += 1

            # 2) 处理模块名字符串占位（可选）
            elif rewrite_module_refs and t in (Sdf.ValueTypeNames.String, Sdf.ValueTypeNames.Token):
                val = attr.Get()
                if isinstance(val, str) and "::.::Materials::" in val:
                    name = val.split("::.::Materials::", 1)[1]
                    newp = _posix_join(materials_base, name + ".mdl")
                    # 对占位符，直接改写为 AssetPath 文件引用
                    attr.Set(Sdf.AssetPath(newp))
                    updated += 1
            else:
                continue

    # 导出到目标文件
    root.Export(dst_usd)

    return {
        "out": dst_usd,
        "updated": str(updated),
        "mode": "relative" if use_relative else "absolute",
        "materials_base": materials_base,
    }


def rewrite_usd_mdl_paths_inplace(
    usd_path: str,
    materials_dir: str,
    use_relative: bool = True,
    relative_base: Optional[str] = None,
    backup: bool = True,
    rewrite_module_refs: bool = True,
) -> Dict[str, str]:
    """
    原地改写：可选创建备份，然后将 USD 的材质引用改写为相对或绝对路径。

    参数：参考 `rewrite_usd_mdl_paths`，其中：
    - usd_path: 原地改写的 USD 文件。
    - backup: True 时在同目录生成 `.bak` 备份副本。
    """
    if backup:
        bak = usd_path + ".bak"
        try:
            import shutil
            shutil.copy2(usd_path, bak)
        except Exception:
            pass
    # 原地导出：dst=src
    return rewrite_usd_mdl_paths(
        src_usd=usd_path,
        dst_usd=usd_path,
        materials_dir=materials_dir,
        use_relative=use_relative,
        relative_base=relative_base or os.path.dirname(usd_path),
        rewrite_module_refs=rewrite_module_refs,
    )


def main():
    """
    简易 CLI：
    - 示例（相对改写）：
      ISAAC_SIM_ROOT=... ./scripts/isaac_python.sh -c "from specs_normalizer.utils.mdl_rewrite import main; main()" -- --src /abs/src.usd --dst /abs/out.usd --materials /abs/Materials --relative --relative-from /abs/out_dir
    - 示例（绝对改写）：
      ISAAC_SIM_ROOT=... ./scripts/isaac_python.sh -c "from specs_normalizer.utils.mdl_rewrite import main; main()" -- --src /abs/src.usd --dst /abs/out.usd --materials /abs/Materials
    """
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--materials", required=True)
    ap.add_argument("--relative", action="store_true")
    ap.add_argument("--relative-from", default=None)
    ap.add_argument("--rewrite-modules", action="store_true")
    args = ap.parse_args()

    res = rewrite_usd_mdl_paths(
        src_usd=args.src,
        dst_usd=args.dst,
        materials_dir=args.materials,
        use_relative=args.relative,
        relative_base=args.relative_from,
        rewrite_module_refs=args.rewrite_modules,
    )
    print("out:", res["out"], "updated:", res["updated"], "mode:", res["mode"], "materials_base:", res["materials_base"])


if __name__ == "__main__":
    main()

