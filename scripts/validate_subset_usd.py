import os
import argparse
import sys
import time
from pxr import Usd, Sdf

def validate_usd(usd_path: str):
    """
    Load USD and check for MDL related errors in the log.
    Note: Since we are running this script inside Isaac Sim python environment, 
    we rely on standard output/error capture if possible, or just the fact that 
    Usd.Stage.Open might trigger errors. 
    However, MDL errors often happen asynchronously or during hydra engine loading which 
    doesn't happen with just Usd.Stage.Open.
    
    A better validation for MDL syntax is trying to resolve the shader.
    But without a full rendering context, it's hard to trigger "MDLC:COMPILER" errors 
    unless we use omni.isaac.core or similar.
    
    Given the constraint, we will try to open the stage and traverse it. 
    If there are missing files, Usd might complain.
    For MDL syntax errors, we might not catch them easily in pure python without 
    running a simulation step or rendering.
    
    However, the user asked for a "headless mode check". 
    If we run this with ./python.sh, it loads the USD core.
    If we want to trigger MDL compilation, we might need to verify if the referenced 
    MDL files exist and are readable.
    """
    print(f"Validating {usd_path}...")
    
    if not os.path.exists(usd_path):
        print(f"Error: File not found: {usd_path}")
        return False

    try:
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            print(f"Error: Failed to open stage {usd_path}")
            return False
            
        # Basic traversal to ensure references are resolvable (to some extent)
        for prim in stage.Traverse():
            pass
            
        print("USD Stage opened successfully.")
        return True
    except Exception as e:
        print(f"Exception opening stage: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Validate USD files in a subset.")
    parser.add_argument("--subset", required=True, help="Root directory of the subset")
    args = parser.parse_args()
    
    assets_dir = os.path.join(args.subset, "GRScenes_assets")
    if not os.path.isdir(assets_dir):
        print(f"Error: GRScenes_assets not found in {args.subset}")
        return
        
    # Find all USDs
    usd_files = []
    for root, dirs, files in os.walk(assets_dir):
        for file in files:
            if file.endswith(".usd"):
                usd_files.append(os.path.join(root, file))
                
    print(f"Found {len(usd_files)} USD files.")
    
    # Validate a few samples or all
    # Since we can't easily capture internal C++ logs from python script output 
    # (unless we redirect fd 2), we will focus on file existence of referenced MDLs.
    
    mdl_root = os.path.join(args.subset, "Material", "mdl")
    missing_mdls = set()
    
    for usd_path in usd_files:
        try:
            stage = Usd.Stage.Open(usd_path)
            for prim in stage.Traverse():
                for attr in prim.GetAttributes():
                    # Check AssetPath attributes
                    val = attr.Get()
                    if isinstance(val, Sdf.AssetPath):
                        path = val.path
                        if "mdl" in path.lower():
                            # Check if the path is resolved correctly relative to the USD file
                            # The path should be something like "../../Material/mdl/Foo.mdl"
                            # We can try to resolve it relative to the USD file
                            usd_dir = os.path.dirname(usd_path)
                            resolved_path = os.path.normpath(os.path.join(usd_dir, path))
                            
                            if not os.path.exists(resolved_path):
                                filename = os.path.basename(path)
                                print(f"Broken Link: {path} -> {resolved_path} (referenced in {os.path.basename(usd_path)})")
                                
                                # Check if it exists in the root anyway
                                if os.path.exists(os.path.join(mdl_root, filename)):
                                     print(f"  -> File exists in MDL root but link is broken.")
                                else:
                                     missing_mdls.add(filename)
                                     print(f"  -> File is MISSING entirely.")
                            
        except Exception as e:
            print(f"Failed to check {usd_path}: {e}")

    if missing_mdls:
        print(f"ERROR: Found {len(missing_mdls)} missing MDL files referenced in USDs.")
        for m in missing_mdls:
            print(f" - {m}")
    else:
        print("All referenced MDL files exist and paths seem valid (or at least resolvable).")

    # Check for OmniUe4Function and OmniUe4Base specifically as they are known dependencies
    print("Checking for known dependencies...")
    common_deps = ["OmniUe4Function.mdl", "OmniUe4Base.mdl"]
    for dep in common_deps:
        if os.path.exists(os.path.join(mdl_root, dep)):
            print(f" - {dep}: OK")
        else:
            print(f" - {dep}: MISSING (This might cause compilation errors!)")

if __name__ == "__main__":
    main()
