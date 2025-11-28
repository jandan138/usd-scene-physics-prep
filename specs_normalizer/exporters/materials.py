"""
材质导出模块 materials

整体介绍：
- 从源目录 `Materials` 复制 `.mdl` 材质文件与 `Textures` 贴图目录到规范结构 `Material/mdl/` 与 `Material/mdl/textures/`。
- 保持文件名与相对层级不变，仅调整顶层位置。
"""

# 标准库用于路径与复制工具的导入通过 utils.fs 统一封装
import os
from ..utils.fs import ensure_dir, copy_file, copy_dir

# 将源目录中的材质复制到目标规范结构
def export_materials(src_root, dst_root):
    mats = os.path.join(src_root, "Materials")                 # 源材质目录
    dst_mdl = os.path.join(dst_root, "Material", "mdl")        # 目标 mdl 目录
    ensure_dir(dst_mdl)                                         # 确保目标目录存在
    for f in os.listdir(mats):                                   # 遍历源材质目录
        p = os.path.join(mats, f)                                # 拼接文件路径
        if os.path.isfile(p) and f.lower().endswith(".mdl"):    # 仅复制 .mdl 文件
            copy_file(p, os.path.join(dst_mdl, f))               # 复制到目标
    tex_src = os.path.join(mats, "Textures")                    # 源贴图目录
    if os.path.isdir(tex_src):
        tex_dst = os.path.join(dst_mdl, "Textures")
        ensure_dir(tex_dst)
        copy_dir(tex_src, tex_dst)
