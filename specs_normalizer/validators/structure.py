import os

def validate_structure(src_root):
    issues = []
    ok = True
    mats = os.path.join(src_root, "Materials")
    models = os.path.join(src_root, "models")
    scenes = os.path.join(src_root, "scenes")
    if not os.path.isdir(mats):
        ok = False
        issues.append("missing Materials")
    else:
        has_mdl = any(f.lower().endswith(".mdl") for f in os.listdir(mats) if os.path.isfile(os.path.join(mats, f)))
        has_tex = os.path.isdir(os.path.join(mats, "Textures"))
        if not has_mdl and not has_tex:
            issues.append("Materials has no mdl or Textures")
    if not os.path.isdir(models):
        ok = False
        issues.append("missing models")
    else:
        scopes = ["layout", "object"]
        subcats = ["articulated", "others"]
        found_any = False
        for s in scopes:
            for c in subcats:
                p = os.path.join(models, s, c)
                if os.path.isdir(p):
                    found_any = True
        if not found_any:
            issues.append("models has no expected subfolders")
    if not os.path.isdir(scenes):
        ok = False
        issues.append("missing scenes")
    else:
        has_layout = False
        for d in os.listdir(scenes):
            sp = os.path.join(scenes, d)
            if not os.path.isdir(sp):
                continue
            for nm in ["start_result_fix.usd", "start_result_new.usd"]:
                if os.path.exists(os.path.join(sp, nm)):
                    has_layout = True
                    break
        if not has_layout:
            issues.append("no start_result_*.usd in scenes")
    return ok, issues

