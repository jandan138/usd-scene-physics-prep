import os
import argparse
import sys
from pxr import Usd, Sdf

def inspect_usd(usd_path: str):
    print(f"Inspecting: {usd_path}")
    
    if not os.path.exists(usd_path):
        print(f"Error: File not found: {usd_path}")
        return

    try:
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            print("Error: Failed to open stage")
            return

        usd_dir = os.path.dirname(os.path.abspath(usd_path))
        print(f"Base Directory: {usd_dir}")
        print("-" * 60)
        print(f"{'Property':<40} | {'Path':<40} | {'Status'}")
        print("-" * 60)

        found_issues = False
        
        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                val = attr.Get()
                if isinstance(val, Sdf.AssetPath):
                    path = val.path
                    if not path:
                        continue
                        
                    # Resolve absolute path
                    if os.path.isabs(path):
                        abs_path = path
                    else:
                        abs_path = os.path.normpath(os.path.join(usd_dir, path))
                    
                    exists = os.path.exists(abs_path)
                    status = "[OK]" if exists else "[MISSING]"
                    
                    # Shorten property name for display
                    prop_name = f"{prim.GetName()}:{attr.GetName()}"
                    if len(prop_name) > 38:
                        prop_name = prop_name[:35] + "..."
                        
                    print(f"{prop_name:<40} | {path:<40} | {status}")
                    
                    if not exists:
                        found_issues = True
                        print(f"  -> Resolved to: {abs_path}")
                        # Check if it might be in textures subdir
                        filename = os.path.basename(path)
                        # Assuming standard structure ../../Material/mdl/textures/
                        # If path is ../../Material/mdl/foo.png
                        potential_tex = os.path.normpath(os.path.join(usd_dir, "../../Material/mdl/textures", filename))
                        if os.path.exists(potential_tex):
                             print(f"  -> HINT: Found at {potential_tex}")
                        else:
                             # Try root mdl
                             potential_mdl = os.path.normpath(os.path.join(usd_dir, "../../Material/mdl", filename))
                             if os.path.exists(potential_mdl):
                                 print(f"  -> HINT: Found at {potential_mdl}")

        print("-" * 60)
        if found_issues:
            print("Issues found! Some assets are missing.")
        else:
            print("All assets resolved successfully.")

    except Exception as e:
        print(f"Exception: {e}")

def main():
    parser = argparse.ArgumentParser(description="Inspect a single USD file for asset references.")
    parser.add_argument("usd_path", help="Path to the USD file")
    args = parser.parse_args()
    
    inspect_usd(args.usd_path)

if __name__ == "__main__":
    main()
