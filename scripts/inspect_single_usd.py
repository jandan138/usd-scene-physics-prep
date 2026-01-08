from pxr import Usd, Sdf

def inspect_usd(path):
    print(f"Inspecting: {path}")
    try:
        stage = Usd.Stage.Open(path)
        if not stage:
            print("  FAILED to open stage.")
            return

        print("  Stage opened successfully.")
        
        # Traverse all prims
        count = 0
        mesh_count = 0
        for prim in stage.Traverse():
            count += 1
            if prim.GetTypeName() == "Mesh":
                mesh_count += 1
            
            # Check references and payloads
            if prim.HasAuthoredReferences():
                refs = prim.GetMetadata("references")
                if refs:
                    print(f"    Prim {prim.GetPath()} has references: {refs}")
            
            if prim.HasAuthoredPayloads():
                payloads = prim.GetMetadata("payloads")
                if payloads:
                    print(f"    Prim {prim.GetPath()} has payloads: {payloads}")

        print(f"  Total Prims: {count}")
        print(f"  Mesh Prims: {mesh_count}")
        
        # Check specific structure
        root = stage.GetPrimAtPath("/Root")
        if root.IsValid():
            inst = stage.GetPrimAtPath("/Root/Instance")
            if inst.IsValid():
                children = inst.GetChildren()
                print(f"  /Root/Instance children count: {len(children)}")
                for child in children:
                    print(f"    - {child.GetName()} ({child.GetTypeName()})")
            else:
                print("  /Root/Instance NOT found.")
        else:
            print("  /Root NOT found.")

    except Exception as e:
        print(f"  Error: {e}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: ./isaac_python.sh scripts/inspect_single_usd.py <usd_path>")
        sys.exit(1)
    
    inspect_usd(sys.argv[1])
