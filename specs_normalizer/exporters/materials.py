import os
from ..utils.fs import ensure_dir, copy_file, copy_dir

def export_materials(src_root, dst_root):
    mats = os.path.join(src_root, "Materials")
    dst_mdl = os.path.join(dst_root, "Material", "mdl")
    ensure_dir(dst_mdl)
    for f in os.listdir(mats):
        p = os.path.join(mats, f)
        if os.path.isfile(p) and f.lower().endswith(".mdl"):
            copy_file(p, os.path.join(dst_mdl, f))
    tex_src = os.path.join(mats, "Textures")
    if os.path.isdir(tex_src):
        tex_dst = os.path.join(dst_mdl, "textures")
        ensure_dir(tex_dst)
        copy_dir(tex_src, tex_dst)

