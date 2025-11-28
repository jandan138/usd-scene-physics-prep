from typing import Optional
import os

def _posix(p: str) -> str:
    return p.replace(os.sep, "/")

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
    for prim in stage.Traverse():
        # 1) 资产属性中的文件路径（包括 .usd 与 .mdl）
        for attr in prim.GetAttributes():
            val = attr.Get()
            if isinstance(val, Sdf.AssetPath):
                p = val.path or ""
                if not p:
                    continue
                if "Materials/" in p or p.endswith(".mdl"):
                    idx = p.find("Materials/") if "Materials/" in p else -1
                    rest = p[idx + len("Materials/"):] if idx >= 0 else os.path.basename(p)
                    newp = f"{mats_rel}/{rest}"
                    if newp != p:
                        attr.Set(Sdf.AssetPath(newp))
                        changed += 1
                elif "models/" in p or p.endswith(".usd"):
                    idx = p.find("models/") if "models/" in p else -1
                    rest = p[idx + len("models/"):] if idx >= 0 else os.path.basename(p)
                    newp = f"{models_rel}/{rest}"
                    if newp != p:
                        attr.Set(Sdf.AssetPath(newp))
                        changed += 1
        # 2) References 重写
        try:
            refs = prim.GetReferences()
            # 获取现有引用需要从 PrimStack 读取已应用项
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
                        np = ap
                        if "Materials/" in ap or ap.endswith(".mdl"):
                            idx = ap.find("Materials/") if "Materials/" in ap else -1
                            rest = ap[idx + len("Materials/"):] if idx >= 0 else os.path.basename(ap)
                            np = f"{mats_rel}/{rest}"
                        elif "models/" in ap or ap.endswith(".usd"):
                            idx = ap.find("models/") if "models/" in ap else -1
                            rest = ap[idx + len("models/"):] if idx >= 0 else os.path.basename(ap)
                            np = f"{models_rel}/{rest}"
                        refs.AddReference(assetPath=np, primPath=pp, layerOffset=lo)
                        changed += 1
                    else:
                        refs.AddInternalReference(pp)
        except Exception:
            pass
        # 3) Payloads 重写
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
                        np = ap
                        if "Materials/" in ap or ap.endswith(".mdl"):
                            idx = ap.find("Materials/") if "Materials/" in ap else -1
                            rest = ap[idx + len("Materials/"):] if idx >= 0 else os.path.basename(ap)
                            np = f"{mats_rel}/{rest}"
                        elif "models/" in ap or ap.endswith(".usd"):
                            idx = ap.find("models/") if "models/" in ap else -1
                            rest = ap[idx + len("models/"):] if idx >= 0 else os.path.basename(ap)
                            np = f"{models_rel}/{rest}"
                        payloads.AddPayload(assetPath=np, primPath=pp, layerOffset=lo)
                        changed += 1
                    else:
                        payloads.AddInternalPayload(pp)
        except Exception:
            pass
    root.Save()
    return changed

