"""
材质导出模块 materials

整体介绍：
- 从源目录 `Materials` 复制 `.mdl` 材质文件与 `Textures` 贴图目录到规范结构 `Material/mdl/` 与 `Material/mdl/textures/`。
- 保持文件名与相对层级不变，仅调整顶层位置。
"""

# 标准库用于路径与复制工具的导入通过 utils.fs 统一封装
import os
import re
from ..utils.fs import ensure_dir, copy_file, copy_dir

def _fix_mdl_text(s):
    p1 = re.compile(r"([\"'])(file:)?([^\"']*?Material/mdl/)Textures/([^\"']+\.(?:png|jpg|jpeg|exr|tif|tga|bmp|webp))(\1)")
    p2 = re.compile(r"([\"'])(file:)?([^\"']*?)Textures/([^\"']+\.(?:png|jpg|jpeg|exr|tif|tga|bmp|webp))(\1)")
    s2 = p1.sub(lambda m: m.group(1) + (m.group(2) or "") + m.group(3) + "textures/" + m.group(4) + m.group(5), s)
    s3 = p2.sub(lambda m: m.group(1) + (m.group(2) or "") + m.group(3) + "textures/" + m.group(4) + m.group(5), s2)
    return s3

def _auto_fix_mdl_case(mdl_root):
    """
    自动修复指定目录下 MDL 文件中的纹理路径大小写 (Textures -> textures)
    """
    count = 0
    for root, _, files in os.walk(mdl_root):
        for f in files:
            if f.lower().endswith(".mdl"):
                p = os.path.join(root, f)
                try:
                    with open(p, "r", encoding="utf-8", errors="ignore") as fh:
                        txt = fh.read()
                    new_txt = _fix_mdl_text(txt)
                    if new_txt != txt:
                        with open(p, "w", encoding="utf-8") as fh:
                            fh.write(new_txt)
                        count += 1
                except Exception:
                    pass
    if count > 0:
        print(f"Auto-fixed {count} MDL files (Textures -> textures).")

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
        tex_dst = os.path.join(dst_mdl, "textures")             # 目标贴图目录（小写）
        ensure_dir(tex_dst)
        copy_dir(tex_src, tex_dst)
    
    # 自动执行大小写修复
    _auto_fix_mdl_case(dst_mdl)
