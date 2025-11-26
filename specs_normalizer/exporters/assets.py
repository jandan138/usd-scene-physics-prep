import os
import json
from ..utils.fs import ensure_dir, copy_file

def iter_model_instances(models_root):
    scopes = ["layout", "object"]
    subcats = ["articulated", "others"]
    for s in scopes:
        for sub in subcats:
            base = os.path.join(models_root, s, sub)
            if not os.path.isdir(base):
                continue
            for category in os.listdir(base):
                cat_path = os.path.join(base, category)
                if not os.path.isdir(cat_path):
                    continue
                for uid in os.listdir(cat_path):
                    u_path = os.path.join(cat_path, uid)
                    if not os.path.isdir(u_path):
                        continue
                    inst = os.path.join(u_path, "instance.usd")
                    if os.path.exists(inst):
                        yield category, uid, inst

def export_assets(src_root, dst_root, asset_name, with_annotations):
    models_root = os.path.join(src_root, "models")
    dest_root = os.path.join(dst_root, asset_name)
    ensure_dir(dest_root)
    stats = {}
    seen = set()
    for category, uid, inst in iter_model_instances(models_root):
        cat_dir = os.path.join(dest_root, category)
        ensure_dir(cat_dir)
        out_path = os.path.join(cat_dir, uid + ".usd")
        key = (category, uid)
        if key in seen:
            continue
        seen.add(key)
        copy_file(inst, out_path)
        stats.setdefault(category, []).append(uid)
        if with_annotations:
            ann = {"uid": uid, "category": category}
            with open(os.path.join(cat_dir, uid + "_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(ann, f, ensure_ascii=False, indent=2)
    if with_annotations:
        for category, uids in stats.items():
            out = {"category": category, "count": len(uids), "uids": uids}
            with open(os.path.join(dest_root, category, "Asset_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)

