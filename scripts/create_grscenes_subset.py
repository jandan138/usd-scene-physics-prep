import os
import json
import shutil
import random
import argparse
import re
from typing import List, Dict, Set
from pxr import Usd, Sdf

def scan_assets(assets_root: str) -> Dict[str, List[str]]:
    """
    Scan assets and filter those with full descriptions.
    Returns a dict mapping category -> list of asset uids.
    """
    valid_assets = {}
    
    if not os.path.isdir(assets_root):
        print(f"Error: Assets root not found: {assets_root}")
        return valid_assets

    for category in os.listdir(assets_root):
        cat_dir = os.path.join(assets_root, category)
        if not os.path.isdir(cat_dir):
            continue

        # Support both layouts:
        # - Old: <assets_root>/<category>/<uid>_annotation.json
        # - New: <assets_root>/<category>/<uid>/<uid>_annotation.json
        for root, _, files in os.walk(cat_dir):
            for fn in files:
                if not fn.endswith("_annotation.json"):
                    continue
                json_path = os.path.join(root, fn)

                # Derive uid from filename (preferred) or parent folder name.
                uid = fn[: -len("_annotation.json")]
                if not uid:
                    uid = os.path.basename(root)

                try:
                    with open(json_path, "r") as jf:
                        data = json.load(jf)
                    if "description" in data and str(data["description"]).strip():
                        valid_assets.setdefault(category, []).append(uid)
                except Exception as e:
                    print(f"Error reading {json_path}: {e}")
                    
    return valid_assets

def select_assets(valid_assets: Dict[str, List[str]], target_count=40, min_categories=4) -> List[tuple]:
    """
    Select assets ensuring minimum category coverage.
    Returns list of (category, uid) tuples.
    """
    categories = list(valid_assets.keys())
    if len(categories) < min_categories:
        print(f"Warning: Only found {len(categories)} categories with valid assets (required: {min_categories})")
    
    selected = []
    
    # 1. Ensure at least one from each available category (up to min_categories)
    random.shuffle(categories)
    
    pool = []
    for cat in categories:
        for uid in valid_assets[cat]:
            pool.append((cat, uid))
            
    if len(pool) < target_count:
        print(f"Warning: Only found {len(pool)} total valid assets (target: {target_count})")
        return pool
        
    used_categories = set()
    forced_picks = []
    
    # Try to pick one from each category until we have enough categories or run out
    for cat in categories:
        if len(used_categories) >= min_categories:
            break
        if valid_assets[cat]:
            uid = random.choice(valid_assets[cat])
            forced_picks.append((cat, uid))
            used_categories.add(cat)
            
    # Fill the rest
    remaining_count = target_count - len(forced_picks)
    remaining_pool = [x for x in pool if x not in forced_picks]
    
    if len(remaining_pool) < remaining_count:
        selected = forced_picks + remaining_pool
    else:
        selected = forced_picks + random.sample(remaining_pool, remaining_count)
        
    return selected

def find_mdl_dependencies(usd_path: str) -> Set[str]:
    """
    Open USD and find all referenced MDL files.
    Returns a set of relative paths (filenames).
    """
    refs = set()
    try:
        stage = Usd.Stage.Open(usd_path)
        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                val = attr.Get()
                if isinstance(val, Sdf.AssetPath):
                    path = val.path
                    if "mdl" in path.lower():
                        filename = os.path.basename(path)
                        refs.add(filename)
    except Exception as e:
        print(f"Error parsing USD {usd_path}: {e}")
        
    return refs

def scan_mdl_internal_dependencies(mdl_path: str, src_material_root: str, processed_mdls: Set[str]) -> Set[str]:
    """
    Recursively scan an MDL file for imports and 'using' statements to find dependencies.
    """
    deps = set()
    if mdl_path in processed_mdls:
        return deps
    
    processed_mdls.add(mdl_path)
    full_path = os.path.join(src_material_root, mdl_path)
    
    if not os.path.exists(full_path):
        print(f"Warning: MDL file not found during dependency scan: {full_path}")
        return deps

    try:
        with open(full_path, 'r', errors='ignore') as f:
            content = f.read()
            
            # Regex for "using .::ModuleName import *;"
            # Matches: using .::OmniUe4Function import *;
            using_matches = re.findall(r'using\s+\.::(\w+)\s+import', content)
            for mod in using_matches:
                deps.add(f"{mod}.mdl")
                
            # Regex for "import ::ModuleName::*;" or "import ::ModuleName;"
            # Matches: import ::OmniUe4Base::*;
            import_matches = re.findall(r'import\s+::(\w+)(?:::\*)?;', content)
            for mod in import_matches:
                # Filter out standard libs if needed, but checking file existence handles it
                if mod not in ["math", "state", "tex", "anno", "df"]:
                    deps.add(f"{mod}.mdl")
                    
    except Exception as e:
        print(f"Error scanning MDL dependencies for {mdl_path}: {e}")

    # Recursively scan found dependencies
    sub_deps = set()
    for dep in deps:
        sub_deps.update(scan_mdl_internal_dependencies(dep, src_material_root, processed_mdls))
    
    deps.update(sub_deps)
    return deps

def copy_assets_and_materials(selected: List[tuple], src_root: str, dst_root: str):
    src_assets = os.path.join(src_root, "GRScenes_assets")
    src_material = os.path.join(src_root, "Material", "mdl")
    
    dst_assets = os.path.join(dst_root, "GRScenes_assets")
    dst_material = os.path.join(dst_root, "Material", "mdl")
    dst_scenes = os.path.join(dst_root, "GRScenes100")
    
    os.makedirs(dst_assets, exist_ok=True)
    os.makedirs(dst_material, exist_ok=True)
    os.makedirs(dst_scenes, exist_ok=True)
    
    src_textures = os.path.join(src_material, "textures")
    dst_textures = os.path.join(dst_material, "textures")

    initial_mdls = set()
    direct_textures = {} # Map filename -> absolute source path
    
    print(f"Copying {len(selected)} assets...")
    
    for cat, uid in selected:
        # Copy USD (new layout)
        src_usd_candidates = [
            os.path.join(src_assets, cat, uid, "usd", f"{uid}.usd"),
            os.path.join(src_assets, cat, f"{uid}.usd"),  # backward-compatible
        ]
        src_usd = next((p for p in src_usd_candidates if os.path.exists(p)), src_usd_candidates[0])

        dst_asset_dir = os.path.join(dst_assets, cat, uid)
        dst_usd_dir = os.path.join(dst_asset_dir, "usd")
        dst_usd = os.path.join(dst_usd_dir, f"{uid}.usd")

        os.makedirs(dst_usd_dir, exist_ok=True)
        if os.path.exists(src_usd):
            shutil.copy2(src_usd, dst_usd)
            
            # Find dependencies and REWRITE paths using Sdf.Layer
            try:
                # Use FindOrOpen to open the layer directly
                layer = Sdf.Layer.FindOrOpen(dst_usd)
                if not layer:
                     print(f"Error: Failed to open layer {dst_usd}")
                     continue

                modified = False
                src_usd_dir = os.path.dirname(src_usd)
                dst_usd_dir_abs = os.path.dirname(dst_usd)
                rel_mdl = os.path.relpath(dst_material, dst_usd_dir_abs).replace(os.sep, "/")
                
                # Recursive function to traverse PrimSpecs
                def traverse_prim_spec(prim_spec):
                    nonlocal modified
                    
                    # Traverse attributes
                    for attr in prim_spec.attributes:
                        # Check default value
                        if attr.HasDefaultValue():
                            val = attr.default
                            if isinstance(val, Sdf.AssetPath):
                                path = val.path
                                filename = os.path.basename(path)
                                ext = os.path.splitext(filename)[1].lower()
                                
                                new_path = None
                                
                                # Handle MDL
                                if "mdl" in path.lower() and path.lower().endswith('.mdl'):
                                    new_path = f"{rel_mdl}/{filename}"
                                    initial_mdls.add(filename)
                                
                                # Handle Textures
                                elif ext in ['.png', '.jpg', '.jpeg', '.tga', '.exr', '.bmp', '.hdr']:
                                    print(f"DEBUG: Found texture in layer: {path} at {prim_spec.path}.{attr.name}")
                                    
                                    # Resolve absolute path
                                    abs_src_path = None
                                    if os.path.isabs(path):
                                        if os.path.exists(path):
                                            abs_src_path = path
                                    else:
                                        try:
                                            potential = os.path.normpath(os.path.join(src_usd_dir, path))
                                            if os.path.exists(potential):
                                                abs_src_path = potential
                                        except:
                                            pass
                                            
                                    if not abs_src_path:
                                        # Fallbacks
                                        potential = os.path.join(src_textures, filename)
                                        if os.path.exists(potential):
                                            abs_src_path = potential
                                        else:
                                            potential = os.path.join(src_material, filename)
                                            if os.path.exists(potential):
                                                abs_src_path = potential
                                                
                                    if abs_src_path:
                                        direct_textures[filename] = abs_src_path
                                        print(f"DEBUG: Resolved texture {filename} -> {abs_src_path}")
                                    else:
                                        print(f"DEBUG: Could not resolve texture {filename}")
                                        
                                    new_path = f"{rel_mdl}/textures/{filename}"

                                # Apply rewrite
                                if new_path and path != new_path:
                                    attr.default = Sdf.AssetPath(new_path)
                                    modified = True

                    # Traverse children
                    for child in prim_spec.nameChildren:
                        traverse_prim_spec(child)
                    
                    # Also traverse variants! Sdf layer traversal sees them as variantSets
                    for variant_set_name in prim_spec.variantSets:
                        variant_set = prim_spec.variantSets[variant_set_name]
                        for variant in variant_set.variantList:
                             traverse_prim_spec(variant.primSpec)

                # Start traversal from pseudo-root
                traverse_prim_spec(layer.pseudoRoot)
                
                if modified:
                    layer.Save()
                
            except Exception as e:
                print(f"Error processing USD layer {dst_usd}: {e}")
                import traceback
                traceback.print_exc()

        else:
            print(f"Error: USD not found {src_usd}")

        # Copy Annotation (new layout: under <uid>/)
        src_anno_candidates = [
            os.path.join(src_assets, cat, uid, f"{uid}_annotation.json"),
            os.path.join(src_assets, cat, f"{uid}_annotation.json"),  # backward-compatible
        ]
        src_anno = next((p for p in src_anno_candidates if os.path.exists(p)), src_anno_candidates[0])
        dst_anno = os.path.join(dst_asset_dir, f"{uid}_annotation.json")
        if os.path.exists(src_anno):
            shutil.copy2(src_anno, dst_anno)
            
    print(f"Found {len(initial_mdls)} directly referenced MDLs.")
    print(f"Found {len(direct_textures)} directly referenced textures in USD.")

    # Recursive dependency scan for MDLs
    print("Scanning for internal MDL dependencies...")
    all_mdls = set(initial_mdls)
    processed_mdls = set()
    
    queue = list(initial_mdls)
    while queue:
        current_mdl = queue.pop(0)
        deps = scan_mdl_internal_dependencies(current_mdl, src_material, processed_mdls)
        for d in deps:
            if d not in all_mdls:
                all_mdls.add(d)
                queue.append(d)
                
    print(f"Total MDLs including dependencies: {len(all_mdls)}")

    # Copy MDLs
    for mdl in all_mdls:
        src_mdl_path = os.path.join(src_material, mdl)
        if os.path.exists(src_mdl_path):
            shutil.copy2(src_mdl_path, os.path.join(dst_material, mdl))
        else:
            if mdl.endswith('.mdl'):
                print(f"Warning: Referenced MDL not found: {mdl}")
            
    # Copy Textures (MDL referenced)
    # We combine MDL-referenced textures and USD-direct textures
    required_textures = set(direct_textures.keys())
    
    # Scan MDLs for texture references
    for mdl in all_mdls:
        src_mdl_path = os.path.join(src_material, mdl)
        if os.path.exists(src_mdl_path):
            try:
                with open(src_mdl_path, 'r', errors='ignore') as f:
                    content = f.read()
                    matches = re.findall(r'\"(?:\./)?textures/([^"]+)\"', content, re.IGNORECASE)
                    required_textures.update(matches)
            except Exception as e:
                print(f"Error reading MDL {mdl}: {e}")

    print(f"Total referenced textures: {len(required_textures)}")
    
    if required_textures:
        os.makedirs(dst_textures, exist_ok=True)
        for tex in required_textures:
            # Check if we already resolved it as a direct texture
            if tex in direct_textures:
                src_path = direct_textures[tex]
                dst_path = os.path.join(dst_textures, tex)
                try:
                    shutil.copy2(src_path, dst_path)
                except Exception as e:
                    print(f"Error copying {src_path}: {e}")
                continue

            # Otherwise, use standard lookup for MDL-referenced textures
            src_tex_path = os.path.join(src_textures, tex)
            found = False
            
            if os.path.exists(src_tex_path):
                found = True
            else:
                # Try lowercase in textures/
                src_tex_path_lower = os.path.join(src_textures, tex.lower())
                if os.path.exists(src_tex_path_lower):
                    src_tex_path = src_tex_path_lower
                    found = True
                else:
                    # Fallback: Check if it's in Material/mdl root
                    src_tex_root = os.path.join(src_material, tex)
                    if os.path.exists(src_tex_root):
                        src_tex_path = src_tex_root
                        found = True
            
            if found:
                 dst_tex_path = os.path.join(dst_textures, tex)
                 os.makedirs(os.path.dirname(dst_tex_path), exist_ok=True)
                 shutil.copy2(src_tex_path, dst_tex_path)
            else:
                # Only warn if we really can't find it
                print(f"Warning: Texture not found: {tex}")
    
    print("Done copying.")

def main():
    parser = argparse.ArgumentParser(description="Create a subset of GRScenes assets.")
    parser.add_argument("--src", required=True, help="Source GRScenes root directory")
    parser.add_argument("--dst", required=True, help="Destination directory for the subset")
    parser.add_argument("--count", type=int, default=40, help="Number of assets to select")
    
    args = parser.parse_args()
    
    src_assets = os.path.join(args.src, "GRScenes_assets")
    
    print("Scanning assets...")
    valid_assets = scan_assets(src_assets)
    
    total_valid = sum(len(v) for v in valid_assets.values())
    print(f"Found {total_valid} assets with full descriptions across {len(valid_assets)} categories.")
    
    print("Selecting subset...")
    selected = select_assets(valid_assets, target_count=args.count)
    
    print(f"Selected {len(selected)} assets.")
    for cat, uid in selected:
        print(f" - {cat}/{uid}")
        
    print("Copying to destination...")
    copy_assets_and_materials(selected, args.src, args.dst)
    
if __name__ == "__main__":
    main()
