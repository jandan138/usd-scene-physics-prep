"""
场景导出模块 scenes

整体介绍：
- 从源目录 `scenes/<sid>/start_result_fix.usd|start_result_new.usd` 选择布局文件，复制到规范结构 `Scene_name/Scene_category/{sid}/layout.usd`。
- 可选生成 `{sid}_annotation.json`，统计 `/Root/Meshes` 子层级下的对象数量（如可用）。
"""

# 导入标准库与工具函数
import os
import json
import re
from ..utils.fs import ensure_dir, copy_file
from ..utils.scene_rewrite import rewrite_scene_refs_inplace, _remap_model_path

# 选择一个布局文件（优先 raw，其次 fix，其次 new，再否则任取一个 USD）
def choose_layout(scene_dir):
    rawp = os.path.join(scene_dir, "start_result_raw.usd")
    fixp = os.path.join(scene_dir, "start_result_fix.usd")
    newp = os.path.join(scene_dir, "start_result_new.usd")
    if os.path.exists(rawp):
        return rawp
    if os.path.exists(fixp):
        return fixp
    if os.path.exists(newp):
        return newp
    for f in os.listdir(scene_dir):
        p = os.path.join(scene_dir, f)
        if os.path.isfile(p) and f.lower().endswith((".usd", ".usda", ".usdc")):
            return p
    return None

# 导出场景到规范结构
def export_scenes(src_root, dst_root, scene_name, scene_category, with_annotations, scene_ids=None):
    scenes_root = os.path.join(src_root, "scenes")                  # 源场景根目录
    dest_root = os.path.join(dst_root, scene_name, scene_category)   # 目标场景顶层目录
    ensure_dir(dest_root)                                            # 确保存在
    try:
        from pxr import Usd                                        # USD 可选导入用于统计
    except Exception:
        Usd = None                                                  # 未安装则置空
    for sid in os.listdir(scenes_root):
        if scene_ids and sid not in scene_ids:
            continue
        sp = os.path.join(scenes_root, sid)
        if not os.path.isdir(sp):
            continue
        layout = choose_layout(sp)                                  # 选择布局文件
        if not layout:
            continue
        out_dir = os.path.join(dest_root, sid)                      # 目标场景目录
        ensure_dir(out_dir)
        copy_file(layout, os.path.join(out_dir, "layout.usd"))
        for f in os.listdir(sp):
            srcf = os.path.join(sp, f)
            if not os.path.isfile(srcf):
                continue
            if not f.lower().endswith((".usd", ".usda", ".usdc")):
                continue
            if os.path.abspath(srcf) == os.path.abspath(layout):
                continue
            copy_file(srcf, os.path.join(out_dir, f))
        mats_abs = os.path.join(dst_root, "Material", "mdl")
        candidates = [
            os.path.join(dst_root, "models"),
            os.path.join(dst_root, "GRScenes_assets"),
            os.path.join(dst_root, "MesaTask_assets"),
        ]
        models_abs = None
        for c in candidates:
            if os.path.isdir(c):
                models_abs = c
                break
        if models_abs is None:
            models_abs = os.path.join(dst_root, "models")
        mats_rel = os.path.relpath(mats_abs, out_dir)
        models_rel = os.path.relpath(models_abs, out_dir)
        for uf in os.listdir(out_dir):
            if not uf.lower().endswith((".usda", ".usd", ".usdc")):
                continue
            usdf = os.path.join(out_dir, uf)
            try:
                rewrite_scene_refs_inplace(usdf, mats_abs, models_abs, relative_base=out_dir)
            except Exception:
                # 文本兜底重写
                try:
                    with open(usdf, "rb") as f:
                        head = f.read(64)
                    is_text = b"usda" in head or head.startswith(b"#")
                    
                    # 如果是二进制文件，尝试使用 pxr.Sdf 将其导出为文本
                    if not is_text:
                        print(f"Converting binary USD to text for rewriting: {usdf}")
                        try:
                            from pxr import Sdf
                            layer = Sdf.Layer.FindOrOpen(usdf)
                            if layer:
                                tmp_usda = usdf + ".tmp.usda"
                                layer.Export(tmp_usda)
                                # 读取转换后的文本
                                with open(tmp_usda, "r", encoding="utf-8", errors="ignore") as f:
                                    txt = f.read()
                            else:
                                print(f"Failed to open layer for conversion: {usdf}")
                                continue
                        except ImportError:
                            print("pxr module not found, cannot convert binary USD.")
                            continue
                        except Exception as e:
                            print(f"Error converting binary USD: {e}")
                            continue
                    else:
                        with open(usdf, "r", encoding="utf-8", errors="ignore") as f:
                            txt = f.read()
                            
                    def _sub(m):
                        p = m.group(1)
                        
                        # [Fix] Global fix for Textures -> textures case sensitivity
                        # This must run before other checks to ensure the path is normalized
                        if "/Textures/" in p:
                            p = p.replace("/Textures/", "/textures/")
                            
                        if "Materials/" in p:
                            idx = p.find("Materials/")
                            rest = p[idx + len("Materials/"):]
                            
                            # [Fix] 移除多余的 mdl/ 前缀
                            if rest.startswith("mdl/"):
                                rest = rest[4:]
                            
                            return "@" + mats_rel + "/" + rest + "@"
                        
                        # Handle cases where path is already rewritten to Material/ (singular) or similar
                        # but still needs relative path adjustment if it's pointing to the old structure
                        if "Material/mdl/" in p:
                             # This might happen if the file was partially processed or has different structure
                             # We assume it should point to mats_rel
                             idx = p.find("Material/mdl/")
                             rest = p[idx + len("Material/mdl/"):]
                             return "@" + mats_rel + "/" + rest + "@"

                        if "models/" in p:
                            remapped = _remap_model_path(p)
                            if remapped:
                                return "@" + models_rel + "/" + remapped + "@"
                            # Fallback logic if _remap returns None (shouldn't happen if "models/" in p)
                            idx = p.find("models/")
                            rest = p[idx + len("models/"):]
                            return "@" + models_rel + "/" + rest + "@"
                        return "@" + p + "@"
                    new_txt = re.sub(r"@([^@]+)@", _sub, txt)
                    
                    if new_txt != txt:
                        # 如果是原本的文本文件，直接覆盖；如果是二进制转换来的，先写回文本再转回二进制
                        if is_text:
                            with open(usdf, "w", encoding="utf-8") as f:
                                f.write(new_txt)
                        else:
                            # 写入修改后的临时文本
                            with open(tmp_usda, "w", encoding="utf-8") as f:
                                f.write(new_txt)
                            # 转回二进制覆盖原文件
                            # 使用 Sdf.Layer 重新导入并保存为 USDC
                            try:
                                layer = Sdf.Layer.FindOrOpen(tmp_usda)
                                if layer:
                                    layer.Export(usdf) # Sdf.Layer.Export 会根据扩展名自动决定格式，这里 usdf 是 .usd (通常默认 crate)
                                    # 但为了保险，我们可以显式指定 args? Sdf.Layer.Export 不支持 args
                                    # 通常 .usd 后缀在 Isaac Sim 环境下默认是二进制
                                    # 或者我们可以先删掉原文件
                                    pass
                                else:
                                    print(f"Failed to reload tmp text layer: {tmp_usda}")
                            except Exception as e:
                                print(f"Failed to convert back to binary: {e}")
                            
                            if os.path.exists(tmp_usda):
                                os.remove(tmp_usda)
                    elif not is_text:
                        # 没变化，清理临时文件
                        if os.path.exists(tmp_usda):
                            os.remove(tmp_usda)
                            
                except Exception as e:
                    print(f"Fallback rewrite failed for {usdf}: {e}")
                    pass
        if with_annotations:                                        # 可选生成注释
            ann = {"sid": sid}
            if Usd:                                                 # 若 USD 可用则统计 Meshes 子层级
                try:
                    stage = Usd.Stage.Open(layout)
                    root = stage.GetPrimAtPath("/Root/Meshes")
                    counts = {}
                    for child in root.GetChildren():
                        counts[child.GetName()] = len(child.GetChildren())
                    ann["counts"] = counts
                except Exception:
                    ann["counts"] = {}
            with open(os.path.join(out_dir, sid + "_annotation.json"), "w", encoding="utf-8") as f:
                json.dump(ann, f, ensure_ascii=False, indent=2)
