#!/usr/bin/env python3
"""Make selected bottle body materials clear-glass-like inside a subset package.

This is intentionally a one-off utility for a known subset layout:
  <subset_root>/GRScenes_assets/bottle/<uid>/usd/<uid>.usd

Strategy:
- Only touch bottle body meshes under Group_bottle_body_00.
- Always give bottle bodies their own cloned material/shader, so cap/default
  parts keep their original bindings.
- For already-translucent bottle bodies, keep the current MDL source and only
  override clear-glass parameters on the clone.
- For opaque bottle bodies, retarget the cloned shader to a known translucent
  MDL already present in the subset, then author clear-glass parameters.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdShade  # type: ignore
except Exception as e:  # pragma: no cover
    print(f"ERROR: pxr/USD bindings unavailable: {e}", file=sys.stderr)
    sys.exit(2)


DEFAULT_UIDS: Tuple[str, ...] = (
    "0309dcb2f1cff82e56b5928b8258b489",
    "1a900e94264d9104f45200db6e0b5e5c",
    "2475b1d276562a0152bb481253b15d9a",
    "471d4d7acbc09edb085f4bfdac8bb2cc",
    "55c75d1633a577833010793d58dfee58",
    "173e246d5eb10f81265e2c174de2101d",
    "23f1d3ecc9f811bdcd7b4f9a169f9f9f",
    "45d5b1701028fc00adc75c5c76042cde",
    "501a5ac3cbdae92226c51c349e4e6a17",
    "7218c60ede56775d76d74bf9a709cd7e",
)

BODY_GROUP_TOKEN = "Group_bottle_body_00"
CLONE_SUFFIX = "__clear_glass_body"
GENERIC_TRANSLUCENT_MDL = "Num6526327829931a0001adab0d.mdl"
GENERIC_TRANSLUCENT_SUBID = "Num6526327829931a0001adab0d"

TARGET_OPACITY = 0.02
TARGET_REFRACTION_SWITCH = 1.0
TARGET_FRESNEL_B = 1.517
TARGET_BASECOLOR = (1.0, 1.0, 1.0, 1.0)
TARGET_METALLIC = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class BoundBodyMesh:
    mesh_path: str
    material_path: str
    shader_path: str
    shader_name: str
    source_asset: str
    sub_identifier: str


def _posix(path: str) -> str:
    return path.replace("\\", "/")


def _norm_abs(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def _iter_target_uids(args_uids: Sequence[str]) -> List[str]:
    if args_uids:
        return list(dict.fromkeys([u for u in args_uids if u]))
    return list(DEFAULT_UIDS)


def _asset_usd_path(subset_root: str, uid: str) -> str:
    return os.path.join(subset_root, "GRScenes_assets", "bottle", uid, "usd", f"{uid}.usd")


def _looks_root_path(stage: Usd.Stage) -> Sdf.Path:
    looks = stage.GetPrimAtPath("/Root/Looks")
    if not looks:
        raise RuntimeError("Missing /Root/Looks in asset stage")
    return looks.GetPath()


def _as_material(prim: Usd.Prim) -> Optional[UsdShade.Material]:
    mat = UsdShade.Material(prim)
    return mat if mat else None


def _material_from_prim(stage: Usd.Stage, prim: Usd.Prim) -> Optional[UsdShade.Material]:
    binding = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterial()
    return binding if binding else None


def _mdl_surface_shader(material: UsdShade.Material) -> Optional[UsdShade.Shader]:
    out = material.GetSurfaceOutput("mdl")
    if not out:
        return None
    src = out.GetConnectedSource()
    if not src:
        return None
    return UsdShade.Shader(src[0].GetPrim())


def _find_body_meshes(stage: Usd.Stage) -> List[BoundBodyMesh]:
    meshes: List[BoundBodyMesh] = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if BODY_GROUP_TOKEN not in path:
            continue
        material = _material_from_prim(stage, prim)
        if material is None:
            continue
        shader = _mdl_surface_shader(material)
        if shader is None:
            continue
        src_asset = shader.GetSourceAsset("mdl")
        src_path = src_asset.path if src_asset else ""
        subid = shader.GetSourceAssetSubIdentifier("mdl") or ""
        meshes.append(
            BoundBodyMesh(
                mesh_path=path,
                material_path=str(material.GetPath()),
                shader_path=str(shader.GetPath()),
                shader_name=shader.GetPrim().GetName(),
                source_asset=src_path,
                sub_identifier=subid,
            )
        )
    return meshes


def _copy_shader_inputs(src_shader: UsdShade.Shader, dst_shader: UsdShade.Shader) -> None:
    for inp in src_shader.GetInputs():
        dst_inp = dst_shader.CreateInput(inp.GetBaseName(), inp.GetTypeName())
        try:
            val = inp.Get()
        except Exception:
            continue
        if val is not None:
            dst_inp.Set(val)


def _ensure_unconnected_outputs(material: UsdShade.Material, src_material: UsdShade.Material) -> None:
    # Preserve the output surface structure expected by existing assets.
    if src_material.GetSurfaceOutput("mdl"):
        material.CreateSurfaceOutput("mdl")
    if src_material.GetSurfaceOutput():
        material.CreateSurfaceOutput()
    if src_material.GetVolumeOutput():
        material.CreateVolumeOutput()
    if src_material.GetDisplacementOutput():
        material.CreateDisplacementOutput()


def _clone_material_and_shader(
    stage: Usd.Stage,
    src_material: UsdShade.Material,
    src_shader: UsdShade.Shader,
    overwrite: bool,
) -> Tuple[UsdShade.Material, UsdShade.Shader, str]:
    src_mat_path = src_material.GetPath()
    src_shader_name = src_shader.GetPrim().GetName()

    if src_mat_path.name.endswith(CLONE_SUFFIX):
        return src_material, src_shader, "reuse_existing_clone"

    new_mat_path = src_mat_path.AppendChild(f"{src_mat_path.name}{CLONE_SUFFIX}")
    if stage.GetPrimAtPath(new_mat_path):
        if overwrite:
            stage.RemovePrim(new_mat_path)
        else:
            existing_mat = UsdShade.Material(stage.GetPrimAtPath(new_mat_path))
            existing_shader = _mdl_surface_shader(existing_mat)
            if existing_shader is None:
                raise RuntimeError(f"Existing cloned material missing mdl:surface shader: {new_mat_path}")
            return existing_mat, existing_shader, "reuse_existing_clone"

    new_shader_name = f"{src_shader_name}{CLONE_SUFFIX}"
    new_shader_path = new_mat_path.AppendChild(new_shader_name)

    new_mat = UsdShade.Material.Define(stage, new_mat_path)
    new_shader = UsdShade.Shader.Define(stage, new_shader_path)
    _ensure_unconnected_outputs(new_mat, src_material)
    _copy_shader_inputs(src_shader, new_shader)

    new_mat.CreateSurfaceOutput("mdl").ConnectToSource(new_shader.ConnectableAPI(), "out")
    return new_mat, new_shader, "created_clone"


def _ensure_shader_source(
    shader: UsdShade.Shader,
    *,
    usd_path: str,
    subset_root: str,
    force_translucent: bool,
    current_subid: str,
) -> Dict[str, str]:
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)

    if force_translucent:
        mdl_name = GENERIC_TRANSLUCENT_MDL
        subid = GENERIC_TRANSLUCENT_SUBID
    else:
        mdl_name = current_subid + ".mdl"
        subid = current_subid

    mdl_abs = os.path.join(subset_root, "Material", "mdl", mdl_name)
    mdl_rel = _posix(os.path.relpath(mdl_abs, os.path.dirname(usd_path)))
    shader.SetSourceAsset(Sdf.AssetPath(mdl_rel), "mdl")
    shader.SetSourceAssetSubIdentifier(subid, "mdl")
    return {"mdl_rel": mdl_rel, "sub_identifier": subid}


def _set_or_create_input(shader: UsdShade.Shader, name: str, type_name, value) -> None:
    inp = shader.GetInput(name)
    if not inp:
        inp = shader.CreateInput(name, type_name)
    inp.Set(value)


def _apply_clear_glass_overrides(shader: UsdShade.Shader) -> Dict[str, object]:
    _set_or_create_input(shader, "Opacity", Sdf.ValueTypeNames.Float, float(TARGET_OPACITY))
    _set_or_create_input(
        shader,
        "SwitchRefraction",
        Sdf.ValueTypeNames.Float,
        float(TARGET_REFRACTION_SWITCH),
    )
    _set_or_create_input(shader, "FresnelB", Sdf.ValueTypeNames.Float, float(TARGET_FRESNEL_B))
    _set_or_create_input(shader, "IsBaseColorTex", Sdf.ValueTypeNames.Float, 0.0)
    _set_or_create_input(shader, "BaseColor_Color", Sdf.ValueTypeNames.Float4, Gf.Vec4f(*TARGET_BASECOLOR))
    _set_or_create_input(shader, "IsMetallicTex", Sdf.ValueTypeNames.Float, 0.0)
    _set_or_create_input(shader, "Metallic_Color", Sdf.ValueTypeNames.Float4, Gf.Vec4f(*TARGET_METALLIC))
    return {
        "inputs:Opacity": TARGET_OPACITY,
        "inputs:SwitchRefraction": TARGET_REFRACTION_SWITCH,
        "inputs:FresnelB": TARGET_FRESNEL_B,
        "inputs:IsBaseColorTex": 0.0,
        "inputs:BaseColor_Color": list(TARGET_BASECOLOR),
        "inputs:IsMetallicTex": 0.0,
        "inputs:Metallic_Color": list(TARGET_METALLIC),
    }


def _rebind_mesh(stage: Usd.Stage, mesh_path: str, material: UsdShade.Material) -> None:
    prim = stage.GetPrimAtPath(mesh_path)
    if not prim:
        raise RuntimeError(f"Mesh not found for rebind: {mesh_path}")
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def _process_asset(
    subset_root: str,
    uid: str,
    *,
    dry_run: bool,
    overwrite: bool,
) -> Dict[str, object]:
    usd_path = _asset_usd_path(subset_root, uid)
    report: Dict[str, object] = {"uid": uid, "usd_path": usd_path, "updated": False, "meshes": [], "errors": []}

    if not os.path.isfile(usd_path):
        report["errors"] = [f"USD not found: {usd_path}"]
        return report

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        report["errors"] = [f"Failed to open USD: {usd_path}"]
        return report

    body_meshes = _find_body_meshes(stage)
    if not body_meshes:
        report["errors"] = [f"No bound body meshes found under {BODY_GROUP_TOKEN}"]
        return report

    by_material: Dict[str, List[BoundBodyMesh]] = {}
    for mesh in body_meshes:
        by_material.setdefault(mesh.material_path, []).append(mesh)

    for material_path, meshes in sorted(by_material.items()):
        src_material = UsdShade.Material(stage.GetPrimAtPath(material_path))
        src_shader = _mdl_surface_shader(src_material)
        if src_shader is None:
            report["errors"].append(f"Missing mdl:surface shader for material {material_path}")
            continue

        clone_mat, clone_shader, clone_action = _clone_material_and_shader(stage, src_material, src_shader, overwrite)
        current_subid = src_shader.GetSourceAssetSubIdentifier("mdl") or ""
        force_translucent = "Opacity" not in [inp.GetBaseName() for inp in src_shader.GetInputs()]
        source_info = _ensure_shader_source(
            clone_shader,
            usd_path=usd_path,
            subset_root=subset_root,
            force_translucent=force_translucent,
            current_subid=current_subid if current_subid else GENERIC_TRANSLUCENT_SUBID,
        )
        overrides = _apply_clear_glass_overrides(clone_shader)

        for mesh in meshes:
            if not dry_run:
                _rebind_mesh(stage, mesh.mesh_path, clone_mat)
            report["meshes"].append(
                {
                    "mesh_path": mesh.mesh_path,
                    "old_material": mesh.material_path,
                    "old_shader": mesh.shader_path,
                    "clone_material": str(clone_mat.GetPath()),
                    "clone_shader": str(clone_shader.GetPath()),
                    "clone_action": clone_action,
                    "force_translucent": force_translucent,
                    "new_mdl": source_info,
                    "overrides": overrides,
                }
            )

    if not report["errors"]:
        report["updated"] = True
        if not dry_run:
            stage.GetRootLayer().Save()
    return report


def _write_report(path: str, data: Dict[str, object]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--subset-root",
        default="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/subsets/grscenes_uid_subset_10",
        help="Subset root containing GRScenes_assets/ and Material/mdl/",
    )
    ap.add_argument("--uid", action="append", default=[], help="Bottle UID to process (repeatable)")
    ap.add_argument("--dry-run", action="store_true", help="Inspect and report without writing USD files")
    ap.add_argument(
        "--report-out",
        default="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/bottle_clear_glass_subset10.json",
        help="Path to JSON report output",
    )
    ap.add_argument(
        "--no-overwrite-clones",
        action="store_true",
        help="Reuse existing cloned materials if present instead of recreating them",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    subset_root = _norm_abs(args.subset_root)
    if not os.path.isdir(subset_root):
        print(f"ERROR: subset root not found: {subset_root}", file=sys.stderr)
        return 2

    uids = _iter_target_uids(args.uid)
    overwrite = not bool(args.no_overwrite_clones)
    asset_reports = []
    updated_count = 0
    error_count = 0

    for uid in uids:
        report = _process_asset(subset_root, uid, dry_run=bool(args.dry_run), overwrite=overwrite)
        asset_reports.append(report)
        if report.get("updated"):
            updated_count += 1
        if report.get("errors"):
            error_count += 1

    summary: Dict[str, object] = {
        "subset_root": subset_root,
        "dry_run": bool(args.dry_run),
        "uids": uids,
        "updated_assets": updated_count,
        "error_assets": error_count,
        "assets": asset_reports,
    }
    _write_report(_norm_abs(args.report_out), summary)

    print(json.dumps({"updated_assets": updated_count, "error_assets": error_count, "report": _norm_abs(args.report_out)}, ensure_ascii=False, indent=2))
    return 1 if error_count else 0


if __name__ == "__main__":
    raise SystemExit(main())
