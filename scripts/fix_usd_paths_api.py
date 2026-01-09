
import os
import sys
from pxr import Usd, Sdf

def fix_stage(stage_path):
    #print(f"Processing: {stage_path}")
    try:
        stage = Usd.Stage.Open(stage_path)
        if not stage:
            print(f"Error: Could not open stage: {stage_path}")
            return False
    except Exception as e:
        print(f"Error opening {stage_path}: {e}")
        return False

    has_changes = False

    # Traverse all prims
    for prim in stage.Traverse():
        # 1. Check Attributes (for Textures mostly)
        for attr in prim.GetAttributes():
            val = attr.Get()
            
            # Check Sdf.AssetPath
            if isinstance(val, Sdf.AssetPath):
                path_str = val.path
                original_path = path_str
                
                # Logic 1: Textures -> textures
                if "/Textures/" in path_str:
                    path_str = path_str.replace("/Textures/", "/textures/")
                
                # Logic 2: Material/mdl deduplication
                if "Material/mdl/mdl/" in path_str:
                    path_str = path_str.replace("Material/mdl/mdl/", "Material/mdl/")
                
                # Apply change if needed
                if path_str != original_path:
                    #print(f"  Fixing AssetPath: {original_path} -> {path_str}")
                    attr.Set(Sdf.AssetPath(path_str))
                    has_changes = True
            
            # Check String that looks like path (Fallback for some shaders inputs:file)
            elif isinstance(val, str) and ("/Textures/" in val or "Material/mdl/mdl/" in val):
                new_val = val
                if "/Textures/" in val:
                    new_val = new_val.replace("/Textures/", "/textures/")
                if "Material/mdl/mdl/" in val:
                    new_val = new_val.replace("Material/mdl/mdl/", "Material/mdl/")
                
                if new_val != val:
                    #print(f"  Fixing String Path: {val} -> {new_val}")
                    attr.Set(new_val)
                    has_changes = True

    if has_changes:
        stage.GetRootLayer().Save()
        print(f"Fixed: {stage_path}")
        return True
    else:
        return False

def process_directory(root_dir):
    count = 0
    fixed = 0
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".usd") or file.endswith(".usda") or file.endswith(".usdc"):
                path = os.path.join(root, file)
                count += 1
                if fix_stage(path):
                    fixed += 1
    
    print(f"Scanned {count} files. Fixed {fixed} files.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python fix_usd_paths_api.py <directory>")
        sys.exit(1)
    
    process_directory(sys.argv[1])
