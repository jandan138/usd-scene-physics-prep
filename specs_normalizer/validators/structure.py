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
    issues = []  # 收集问题列表
    ok = True    # 总体是否通过
    mats = os.path.join(src_root, "Materials")  # 材质目录路径
    models = os.path.join(src_root, "models")   # 模型目录路径
    scenes = os.path.join(src_root, "scenes")   # 场景目录路径
    if not os.path.isdir(mats):
        ok = False  # 缺少材质目录
        issues.append("missing Materials")
    else:
        # 至少应存在 .mdl 或 Textures 子目录
        has_mdl = any(f.lower().endswith(".mdl") for f in os.listdir(mats) if os.path.isfile(os.path.join(mats, f)))
        has_tex = os.path.isdir(os.path.join(mats, "Textures"))
        if not has_mdl and not has_tex:
            issues.append("Materials has no mdl or Textures")
    if not os.path.isdir(models):
        ok = False  # 缺少模型目录
        issues.append("missing models")
    else:
        # 至少存在期望的 scope/子类别组合中的任意一个
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
        ok = False  # 缺少场景目录
        issues.append("missing scenes")
    else:
        has_layout = False  # 是否找到 start_result_* 布局文件
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
    return ok, issues  # 返回是否通过及问题列表
