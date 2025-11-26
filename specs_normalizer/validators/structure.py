"""
源目录结构校验 structure

整体介绍：
- 校验 `Materials`、`models`、`scenes` 三大目录的存在与基本内容是否符合原始输出结构约定。
- 仅进行必要的存在性与简单内容检查，不深入解析 USD。
"""

# 使用 os 进行路径拼接与文件系统检查
import os

# 校验函数，返回 (ok, issues)
def validate_structure(src_root):
    issues = []
    ok = True
    summary = {"materials": None, "models": {}, "scenes": None}
    mats = os.path.join(src_root, "Materials")
    models = os.path.join(src_root, "models")
    scenes = os.path.join(src_root, "scenes")
    if not os.path.isdir(mats):
        ok = False
        issues.append("missing Materials")
        summary["materials"] = {"exists": False, "has_mdl": False, "mdl_count": 0, "has_textures": False, "textures_dir": None}
    else:
        mdl_count = sum(1 for f in os.listdir(mats) if os.path.isfile(os.path.join(mats, f)) and f.lower().endswith(".mdl"))
        has_mdl = mdl_count > 0
        tex_dir_name = None
        for d in os.listdir(mats):
            if os.path.isdir(os.path.join(mats, d)) and d.lower() == "textures":
                tex_dir_name = d
                break
        has_tex = tex_dir_name is not None
        summary["materials"] = {"exists": True, "has_mdl": has_mdl, "mdl_count": mdl_count, "has_textures": has_tex, "textures_dir": tex_dir_name}
        if not has_mdl:
            ok = False
            issues.append("Materials missing mdl")
        if not has_tex:
            ok = False
            issues.append("Materials missing Textures")
    if not os.path.isdir(models):
        ok = False
        issues.append("missing models")
    else:
        scopes = ["layout", "object"]
        subcats = ["articulated", "others"]
        found_any = False
        models_summary = {}
        for s in scopes:
            models_summary.setdefault(s, {})
            for c in subcats:
                p = os.path.join(models, s, c)
                cat_summary = {}
                if os.path.isdir(p):
                    found_any = True
                    for category in os.listdir(p):
                        cat_path = os.path.join(p, category)
                        if not os.path.isdir(cat_path):
                            continue
                        uids_total = 0
                        uids_with_usd = 0
                        uids_missing = []
                        for uid in os.listdir(cat_path):
                            uid_path = os.path.join(cat_path, uid)
                            if not os.path.isdir(uid_path):
                                continue
                            uids_total += 1
                            has_usd = any(os.path.isfile(os.path.join(uid_path, f)) and f.lower().endswith((".usd", ".usda", ".usdc")) for f in os.listdir(uid_path))
                            if has_usd:
                                uids_with_usd += 1
                            else:
                                uids_missing.append(uid)
                        if uids_total > 0:
                            cat_summary[category] = {"uids_total": uids_total, "uids_with_usd": uids_with_usd, "uids_missing_usd": uids_missing}
                            if uids_missing:
                                ok = False
                                issues.append(f"models {s}/{c}/{category} missing usd in some uids")
                models_summary[s][c] = cat_summary
        if not found_any:
            issues.append("models has no expected subfolders")
        summary["models"] = models_summary
    if not os.path.isdir(scenes):
        ok = False
        issues.append("missing scenes")
        summary["scenes"] = {"exists": False, "total_scene_dirs": 0, "with_any_usd": 0, "missing_any_usd": 0, "details": {"missing": [], "present": []}}
    else:
        subdirs = [d for d in os.listdir(scenes) if os.path.isdir(os.path.join(scenes, d))]
        if not subdirs:
            ok = False
            issues.append("scenes has no subdir")
            summary["scenes"] = {"exists": True, "total_scene_dirs": 0, "with_any_usd": 0, "missing_any_usd": 0, "details": {"missing": [], "present": []}}
        else:
            missing_any_usd = []
            present = []
            with_any = 0
            for d in subdirs:
                sp = os.path.join(scenes, d)
                has_usd = any(os.path.isfile(os.path.join(sp, f)) and f.lower().endswith((".usd", ".usda", ".usdc")) for f in os.listdir(sp))
                if has_usd:
                    with_any += 1
                    present.append(d)
                else:
                    missing_any_usd.append(d)
            if missing_any_usd:
                ok = False
                issues.append("some scene dirs missing usd")
            summary["scenes"] = {"exists": True, "total_scene_dirs": len(subdirs), "with_any_usd": with_any, "missing_any_usd": len(missing_any_usd), "details": {"missing": missing_any_usd, "present": present}}
    return ok, issues, summary
