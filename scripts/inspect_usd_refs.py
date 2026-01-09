
import os
import sys
from pxr import Usd, Sdf

def inspect_usd(path):
    print(f"Inspecting: {path}")
    if not os.path.exists(path):
        print(f"Error: File not found: {path}")
        return

    stage = Usd.Stage.Open(path)
    if not stage:
        print(f"Error: Could not open stage: {path}")
        return

    print(f"Stage opened successfully.")
    base_dir = os.path.dirname(path)
    
    print("\n--- References Inspection ---")
    for prim in stage.Traverse():
        # Check References
        if prim.HasAuthoredReferences():
            refs = prim.GetMetadata("references")
            if refs:
                # refs is a pxr.Sdf.ReferenceListOp
                # explicit items
                for ref in refs.GetAddedOrExplicitItems():
                    asset_path = ref.assetPath
                    if not asset_path:
                        continue
                    
                    # Resolve path
                    abs_path = asset_path
                    if not os.path.isabs(asset_path):
                        abs_path = os.path.normpath(os.path.join(base_dir, asset_path))
                    
                    exists = os.path.exists(abs_path)
                    status = "OK" if exists else "MISSING"
                    
                    print(f"Prim: {prim.GetPath()}")
                    print(f"  Ref: {asset_path}")
                    print(f"  Abs: {abs_path}")
                    print(f"  Status: {status}")
                    
        # Check Attributes (e.g. for Textures)
        for attr in prim.GetAttributes():
            val = attr.Get()
            # Check Sdf.AssetPath
            if isinstance(val, Sdf.AssetPath):
                path_str = val.path
                if path_str:
                    abs_path = path_str
                    if not os.path.isabs(path_str):
                        abs_path = os.path.normpath(os.path.join(base_dir, path_str))
                    exists = os.path.exists(abs_path)
                    status = "OK" if exists else "MISSING"
                    print(f"Prim: {prim.GetPath()}.{attr.GetName()}")
                    print(f"  AssetPath: {path_str}")
                    print(f"  Abs: {abs_path}")
                    print(f"  Status: {status}")
            
            # Check String that looks like path (Fallback for some shaders)
            elif isinstance(val, str) and ("/" in val or "\\" in val):
                if val.endswith((".png", ".jpg", ".usd", ".mdl")):
                     # Check if it looks like a relative path
                     abs_path = val
                     if not os.path.isabs(val):
                        abs_path = os.path.normpath(os.path.join(base_dir, val))
                     exists = os.path.exists(abs_path)
                     status = "OK" if exists else "MISSING"
                     if not exists: # Only print suspicious strings if missing, to reduce noise
                        print(f"Prim: {prim.GetPath()}.{attr.GetName()} (String)")
                        print(f"  Val: {val}")
                        print(f"  Abs: {abs_path}")
                        print(f"  Status: {status}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python inspect_usd_refs.py <path_to_usd>")
        sys.exit(1)
    
    inspect_usd(sys.argv[1])
