from typing import Optional
import os

def _posix(p: str) -> str:
    return p.replace(os.sep, "/")

def _remap_model_path(path_str: str) -> Optional[str]:
    """
    智能解析模型路径，将其从源结构映射到新规范结构。
    源结构示例：.../models/layout/articulated/<category>/<uid>/instance.usd
    新结构：<category>/<uid>/usd/<uid>.usd
    """
    p = _posix(path_str)
    if "models/" not in p:
        return None

    parts = p.split("/")
    
    # 策略1: 寻找 instance.usd
    # 结构: .../<category>/<uid>/instance.usd
    try:
        # 从后往前找，防止路径前面也有 instance.usd (虽然不太可能)
        # rindex 不支持 list，用遍历
        idx = -1
        for i in range(len(parts) - 1, -1, -1):
            if parts[i] == "instance.usd":
                idx = i
                break
        
        if idx >= 2:
            uid = parts[idx-1]
            category = parts[idx-2]
            return f"{category}/{uid}/usd/{uid}.usd"
    except Exception:
        pass

    # 策略2: 如果不是 instance.usd，但包含 models/，尝试提取最后几层
    # 假设源路径已经被清洗过，最后两层是 category/uid
    # 但如果文件名不是 instance.usd，我们可能不应该改名为 uid.usd?
    # 为了安全，如果匹配不到 instance.usd，我们尝试保留原有逻辑（截取 models/ 之后的部分），
    # 但考虑到 export_assets 做了扁平化，保留 layout/articulated 肯定是不对的。
    # 我们尝试提取倒数第二、三层作为 uid, category
    if len(parts) >= 3 and p.lower().endswith((".usd", ".usda", ".usdc")):
        # 情况 A：已经是新结构 .../<category>/<uid>/usd/<something>.usd
        if len(parts) >= 4 and parts[-2] == "usd":
            uid = parts[-3]
            category = parts[-4]
            return f"{category}/{uid}/usd/{uid}.usd"

        # 情况 B：源路径可能是 .../<category>/<uid>/<filename>.usd
        uid = parts[-2]
        category = parts[-3]
        return f"{category}/{uid}/usd/{uid}.usd"

    # 兜底：截取 models/ 之后的部分 (兼容旧逻辑，虽然可能是错的)
    idx = p.find("models/")
    return p[idx + len("models/"):]

def rewrite_scene_refs_inplace(scene_usd: str, materials_dir: str, models_dir: str, relative_base: Optional[str] = None) -> int:
    from pxr import Usd, Sdf
    stage = Usd.Stage.Open(scene_usd)
    root = stage.GetRootLayer()
    base_dir = relative_base or os.path.dirname(scene_usd)
    mats_rel = os.path.relpath(materials_dir, base_dir)
    models_rel = os.path.relpath(models_dir, base_dir)
    mats_rel = _posix(mats_rel)
    models_rel = _posix(models_rel)
    changed = 0
    
    def _process_path(p: str) -> Optional[str]:
        if not p:
            return None
        # 处理材质
        if "Materials/" in p:
            idx = p.find("Materials/")
            rest = p[idx + len("Materials/"):]
            
            # [Fix] 移除多余的 mdl/ 前缀
            if rest.startswith("mdl/"):
                rest = rest[4:]
            
            # [Fix] 强制 Textures -> textures 小写
            if "/Textures/" in rest:
                rest = rest.replace("/Textures/", "/textures/")
                
            return f"{mats_rel}/{rest}"
        elif p.endswith(".mdl"):
            # 兼容旧逻辑：如果只是 .mdl 结尾但没 Materials/，也指向材质库？
            # 建议：如果是同级 mdl，不应该指过去。但为了保险起见，如果它看起来像相对路径...
            # 这里保持原逻辑：basename + mats_rel
            rest = os.path.basename(p)
            return f"{mats_rel}/{rest}"
            
        # 处理模型
        if "models/" in p:
            remapped = _remap_model_path(p)
            if remapped:
                return f"{models_rel}/{remapped}"
        
        # 注意：不再处理不含 "models/" 但以 ".usd" 结尾的路径
        # 以保护同级引用不被错误重定向
        return None

    for prim in stage.Traverse():
        # 1) 资产属性
        for attr in prim.GetAttributes():
            val = attr.Get()
            if isinstance(val, Sdf.AssetPath):
                p = val.path or ""
                newp = _process_path(p)
                if newp and newp != p:
                    attr.Set(Sdf.AssetPath(newp))
                    changed += 1
                    
        # 2) References
        try:
            refs = prim.GetReferences()
            stack = prim.GetPrimStack()
            items = []
            for spec in stack:
                try:
                    for r in spec.referenceList.GetAppliedItems():
                        items.append(r)
                except Exception:
                    pass
            if items:
                refs.ClearReferences()
                for r in items:
                    ap = getattr(r, "assetPath", "") or ""
                    pp = getattr(r, "primPath", Sdf.Path.emptyPath)
                    lo = getattr(r, "layerOffset", None)
                    if ap:
                        newp = _process_path(ap) or ap
                        refs.AddReference(assetPath=newp, primPath=pp, layerOffset=lo)
                        if newp != ap:
                            changed += 1
                    else:
                        refs.AddInternalReference(pp)
        except Exception:
            pass
            
        # 3) Payloads
        try:
            payloads = prim.GetPayloads()
            stack = prim.GetPrimStack()
            items = []
            for spec in stack:
                try:
                    for pl in spec.payloadList.GetAppliedItems():
                        items.append(pl)
                except Exception:
                    pass
            if items:
                payloads.ClearPayloads()
                for pl in items:
                    ap = getattr(pl, "assetPath", "") or ""
                    pp = getattr(pl, "primPath", Sdf.Path.emptyPath)
                    lo = getattr(pl, "layerOffset", None)
                    if ap:
                        newp = _process_path(ap) or ap
                        payloads.AddPayload(assetPath=newp, primPath=pp, layerOffset=lo)
                        if newp != ap:
                            changed += 1
                    else:
                        payloads.AddInternalPayload(pp)
        except Exception:
            pass
            
    root.Save()
    return changed
