#!/usr/bin/env python3
"""Open a USD scene in Isaac Sim headless mode and capture all errors/warnings.

Usage (must run via Isaac Sim python wrapper):
  ./scripts/isaac_python.sh scripts/debug_scene_headless.py --scene /path/to/scene.usd

Captures structured error output from omni.hydra, omni.usd, omni.kit, PhysX,
material/shader subsystems, and any other Carbonite log channels.
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import sys
import time

# ---------------------------------------------------------------------------
# Parse arguments BEFORE Isaac Sim init (SimulationApp may modify sys.argv)
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="Open a USD scene in Isaac Sim headless mode and capture all errors."
)
parser.add_argument("--scene", required=True, help="Path to the USD scene file")
parser.add_argument(
    "--timeout",
    type=int,
    default=30,
    help="Seconds to wait after open_stage for assets to resolve (default: 30)",
)
parser.add_argument(
    "--json",
    dest="json_output",
    default=None,
    help="Optional path to write JSON error report",
)
parser.add_argument(
    "--verbose",
    action="store_true",
    help="Print every captured log message (not just the summary)",
)
args = parser.parse_args()

scene_path = os.path.abspath(args.scene)
if not os.path.isfile(scene_path):
    print(f"ERROR: scene file not found: {scene_path}", file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Initialize Isaac Sim headless
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp

CONFIG = {"headless": True, "sync_loads": True, "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(launch_config=CONFIG)

import carb  # noqa: E402  (available after SimulationApp init)
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics  # noqa: E402

# ---------------------------------------------------------------------------
# Set up Carbonite log capture
# ---------------------------------------------------------------------------
# severity levels: 0=verbose, 1=info, 2=warn, 3=error, 4=fatal
_SEVERITY_NAMES = {0: "VERBOSE", 1: "INFO", 2: "WARNING", 3: "ERROR", 4: "FATAL"}

# Collected messages: list of (severity_str, source, message)
captured_messages: list[tuple[str, str, str]] = []


def _log_handler(source: str, level: int, filename: str, function: str, line: int, message: str):
    """Carbonite log callback — captures WARNING and above."""
    if level >= 2:  # WARNING, ERROR, FATAL
        sev = _SEVERITY_NAMES.get(level, f"LEVEL{level}")
        captured_messages.append((sev, source or "unknown", message))
        if args.verbose:
            print(f"  [{sev}] {source}: {message}")


# Register the handler via carb logging
log_iface = carb.logging.acquire_logging()
if hasattr(log_iface, "add_logger"):
    log_iface.add_logger(_log_handler)
    _handler_registered = True
else:
    # Fallback: some Isaac Sim versions expose set_log_level but not add_logger.
    # We'll still get stderr output; just won't have structured capture.
    _handler_registered = False
    print("NOTE: carb.logging.add_logger not available; relying on stderr capture only.")

# ---------------------------------------------------------------------------
# Open stage
# ---------------------------------------------------------------------------
print(f"\n{'='*70}")
print(f"Scene : {scene_path}")
print(f"Timeout: {args.timeout}s")
print(f"{'='*70}\n")

context = omni.usd.get_context()
t0 = time.time()
result = context.open_stage(scene_path)
t_open = time.time() - t0
print(f"open_stage returned: {result}  ({t_open:.1f}s)")

# Wait for assets / references to resolve
print(f"Waiting {args.timeout}s for stage to fully load...")
time.sleep(args.timeout)

# ---------------------------------------------------------------------------
# Analyse loaded stage
# ---------------------------------------------------------------------------
stage = context.get_stage()
report: dict = {
    "scene": scene_path,
    "open_result": result,
    "open_time_s": round(t_open, 2),
}

if stage is None:
    print("\nFATAL: stage is None after open_stage!")
    report["stage_valid"] = False
else:
    report["stage_valid"] = True

    # Traverse all prims
    all_prims = list(stage.Traverse())
    report["total_prims"] = len(all_prims)

    # Classify prims
    mesh_count = 0
    shader_count = 0
    material_count = 0
    xform_count = 0
    physics_rigid_count = 0
    physics_collider_count = 0
    unloaded_refs = []
    inactive_prims = 0

    for prim in all_prims:
        type_name = prim.GetTypeName()
        if type_name == "Mesh":
            mesh_count += 1
        elif prim.IsA(UsdShade.Shader):
            shader_count += 1
        elif prim.IsA(UsdShade.Material):
            material_count += 1
        if prim.IsA(UsdGeom.Xform):
            xform_count += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            physics_rigid_count += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            physics_collider_count += 1

    # Check for prims that are not active (could be loaded but inactive)
    for prim in stage.TraverseAll():
        if not prim.IsActive():
            inactive_prims += 1

    # Check composition errors via layer stack
    root_layer = stage.GetRootLayer()
    sublayer_paths = root_layer.subLayerPaths
    external_refs = []
    for prim in all_prims:
        prim_stack = prim.GetPrimStack()
        for spec in prim_stack:
            layer = spec.layer
            if layer and layer.identifier != root_layer.identifier:
                ref_id = layer.identifier
                if ref_id not in external_refs:
                    external_refs.append(ref_id)

    report["prim_counts"] = {
        "total": len(all_prims),
        "meshes": mesh_count,
        "shaders": shader_count,
        "materials": material_count,
        "xforms": xform_count,
        "physics_rigid_bodies": physics_rigid_count,
        "physics_colliders": physics_collider_count,
        "inactive_prims": inactive_prims,
    }
    report["external_layer_count"] = len(external_refs)

    # Print summary
    print(f"\n--- Stage Summary ---")
    for k, v in report["prim_counts"].items():
        print(f"  {k:30s}: {v}")
    print(f"  {'external layers':30s}: {len(external_refs)}")

# ---------------------------------------------------------------------------
# Summarize captured errors/warnings
# ---------------------------------------------------------------------------
print(f"\n--- Captured Log Messages (WARNING+) ---")
print(f"  Total captured: {len(captured_messages)}")

# Group by (severity, source)
counter: dict[tuple[str, str], list[str]] = collections.defaultdict(list)
for sev, src, msg in captured_messages:
    counter[(sev, src)].append(msg)

# Build summary rows sorted by count descending
summary_rows = []
for (sev, src), msgs in sorted(counter.items(), key=lambda x: -len(x[1])):
    summary_rows.append({
        "severity": sev,
        "source": src,
        "count": len(msgs),
        "sample": msgs[0][:300],
    })

report["error_summary"] = summary_rows
report["total_warnings_and_errors"] = len(captured_messages)

if summary_rows:
    print(f"\n  {'Severity':<10} {'Source':<40} {'Count':>6}  Sample")
    print(f"  {'-'*10} {'-'*40} {'-'*6}  {'-'*50}")
    for row in summary_rows:
        sample_oneline = row["sample"].replace("\n", " ")[:80]
        print(f"  {row['severity']:<10} {row['source']:<40} {row['count']:>6}  {sample_oneline}")
else:
    print("  (none captured)")

# ---------------------------------------------------------------------------
# Write JSON report if requested
# ---------------------------------------------------------------------------
if args.json_output:
    out_path = os.path.abspath(args.json_output)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"\nJSON report written to: {out_path}")

# ---------------------------------------------------------------------------
# Final status
# ---------------------------------------------------------------------------
error_count = sum(1 for sev, _, _ in captured_messages if sev in ("ERROR", "FATAL"))
warn_count = sum(1 for sev, _, _ in captured_messages if sev == "WARNING")

print(f"\n{'='*70}")
print(f"RESULT: {error_count} errors, {warn_count} warnings")
print(f"{'='*70}")

# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------
simulation_app.close()

sys.exit(1 if error_count > 0 else 0)
