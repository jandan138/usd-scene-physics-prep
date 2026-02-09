#!/usr/bin/env python3

import argparse
from typing import Iterable, Optional

from pxr import Sdf, Usd


def _safe_repr(v) -> str:
    try:
        return repr(v)
    except Exception:
        return f"<{type(v).__name__}>"


def _print_header(title: str) -> None:
    print("\n" + title)
    print("-" * len(title))


def _iter_limited(items: Iterable, limit: int):
    for i, item in enumerate(items):
        if i >= limit:
            print(f"  ... ({i} shown, more omitted)")
            return
        yield item


def _print_ref_listop(listop: Optional[object], base_dir: str) -> None:
    if not listop:
        print("  (none)")
        return

    # We only care about explicit/added references.
    try:
        refs = list(listop.GetAddedOrExplicitItems())
    except Exception:
        print(f"  {listop}")
        return

    if not refs:
        print("  (empty)")
        return

    for ref in refs:
        asset_path = getattr(ref, "assetPath", "")
        prim_path = getattr(ref, "primPath", "")
        layer_offset = getattr(ref, "layerOffset", None)
        custom_data = getattr(ref, "customData", None)

        print(f"  assetPath: {asset_path}")
        if prim_path:
            print(f"  primPath:  {prim_path}")
        if layer_offset:
            print(f"  offset:    {layer_offset}")
        if custom_data:
            print(f"  customData:{custom_data}")

        if asset_path and not str(asset_path).startswith("/"):
            print(f"  resolved:  {base_dir}/{asset_path}")


def inspect(layout_path: str, prim_path: str) -> int:
    stage = Usd.Stage.Open(layout_path)
    if not stage:
        print(f"ERROR: Failed to open stage: {layout_path}")
        return 2

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"ERROR: Prim not found: {prim_path}")
        return 3

    _print_header("Prim Summary")
    print(f"layout: {layout_path}")
    print(f"prim:   {prim.GetPath()}")
    print(f"type:   {prim.GetTypeName() or '(empty)'}")
    print(f"active: {prim.IsActive()}")
    print(f"valid:  {prim.IsValid()}")

    try:
        print(f"instanceable: {prim.IsInstanceable()}")
    except Exception:
        pass

    _print_header("Ancestor Chain")
    cur = prim
    chain = []
    while cur and cur.IsValid() and cur.GetPath() != Sdf.Path.absoluteRootPath:
        chain.append(cur)
        cur = cur.GetParent()
    for p in reversed(chain):
        print(f"- {p.GetPath()} ({p.GetTypeName() or '(empty)'})")

    _print_header("Metadata")
    for key in ["kind", "purpose", "assetInfo", "customData", "comment", "apiSchemas"]:
        try:
            if prim.HasAuthoredMetadata(key):
                print(f"{key}: {_safe_repr(prim.GetMetadata(key))}")
        except Exception:
            continue

    _print_header("Composition Arcs")
    root_layer = stage.GetRootLayer()
    root_layer_path = str(root_layer.realPath or "")
    root_dir = root_layer_path.rsplit("/", 1)[0] if "/" in root_layer_path else root_layer_path

    print("references:")
    try:
        refs = prim.GetMetadata("references")
        _print_ref_listop(refs, root_dir)

        # Also print a resolved path for convenience (correctly anchored to the root layer).
        if refs:
            try:
                for ref in refs.GetAddedOrExplicitItems():
                    ap = getattr(ref, "assetPath", "")
                    if ap and not str(ap).startswith("/"):
                        resolved = Sdf.ComputeAssetPathRelativeToLayer(root_layer, ap)
                        print(f"  resolved(via root layer): {resolved}")
            except Exception:
                pass
    except Exception as e:
        print(f"  <error reading references: {e}>")

    print("payloads:")
    try:
        payloads = prim.GetMetadata("payloads")
        if not payloads:
            print("  (none)")
        else:
            try:
                items = list(payloads.GetAddedOrExplicitItems())
            except Exception:
                items = []
            if not items:
                print(f"  {payloads}")
            else:
                for p in items:
                    print(f"  assetPath: {getattr(p, 'assetPath', '')}")
                    print(f"  primPath:  {getattr(p, 'primPath', '')}")
    except Exception as e:
        print(f"  <error reading payloads: {e}>")

    _print_header("Relationships (top-level)")
    rels = list(prim.GetRelationships())
    if not rels:
        print("(none)")
    else:
        for rel in _iter_limited(rels, 40):
            try:
                targets = rel.GetTargets()
            except Exception:
                targets = []
            tshow = ", ".join(str(t) for t in targets[:5])
            suffix = "" if len(targets) <= 5 else f" (+{len(targets)-5} more)"
            print(f"{rel.GetName()}: {tshow}{suffix}")

    _print_header("Attributes (selected)")
    interesting_prefixes = (
        "skel:",
        "physx",
        "physics:",
        "rigidBody",
        "collision",
        "joint",
        "xformOp:",
    )
    attrs = list(prim.GetAttributes())
    interesting = []
    for a in attrs:
        name = a.GetName()
        if name.startswith(interesting_prefixes) or any(p in name.lower() for p in ["skel", "physx", "physics", "joint", "rigid", "collision"]):
            interesting.append(a)

    if not interesting:
        print("(no obvious animation/physics attrs found on this prim)")
    else:
        for a in _iter_limited(interesting, 80):
            try:
                val = a.Get()
            except Exception as e:
                val = f"<error: {e}>"
            line = f"{a.GetName()} ({a.GetTypeName()}): {_safe_repr(val)}"
            try:
                ts = a.GetTimeSamples() if a.ValueMightBeTimeVarying() else []
                if ts:
                    line += f"  [timeSamples={len(ts)} first={ts[:5]}]"
            except Exception:
                pass
            print(line)

    _print_header("Xform Samples")
    xform_attrs = [a for a in attrs if a.GetName().startswith("xformOp:")]
    if not xform_attrs:
        print("(no xformOp:* attrs)")
    else:
        for a in _iter_limited(xform_attrs, 20):
            try:
                ts = a.GetTimeSamples()
                if not ts:
                    print(f"{a.GetName()}: default only")
                    continue
                print(f"{a.GetName()}: timeSamples={len(ts)} first={ts[:5]}")
                for t in ts[:3]:
                    try:
                        v = a.Get(t)
                    except Exception as e:
                        v = f"<error: {e}>"
                    print(f"  t={t}: {_safe_repr(v)}")
            except Exception as e:
                print(f"{a.GetName()}: <error: {e}>")

    _print_header("Who Targets This Prim")
    target_path = prim.GetPath()
    hits = []
    for p in stage.Traverse():
        for rel in p.GetRelationships():
            try:
                if target_path in rel.GetTargets():
                    hits.append((str(p.GetPath()), f"rel:{rel.GetName()}"))
            except Exception:
                continue
        for a in p.GetAttributes():
            try:
                tn = str(a.GetTypeName())
                if tn not in ("path", "path[]"):
                    continue
                v = a.Get()
                if v == target_path or (isinstance(v, (list, tuple)) and target_path in v):
                    hits.append((str(p.GetPath()), f"attr:{a.GetName()}"))
            except Exception:
                continue

    if not hits:
        print("(no relationships or path-valued attributes point to this prim)")
    else:
        for hp, how in hits[:80]:
            print(f"- {hp} via {how}")
        if len(hits) > 80:
            print(f"... ({len(hits)} total hits, truncated)")

    _print_header("Child Preview")
    children = prim.GetChildren()
    print(f"children: {len(children)}")
    for c in _iter_limited(children, 30):
        print(f"- {c.GetPath()} ({c.GetTypeName() or '(empty)'})")

    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("layout", help="Path to layout.usd")
    ap.add_argument("prim", help="Prim path inside layout")
    args = ap.parse_args()
    return inspect(args.layout, args.prim)


if __name__ == "__main__":
    raise SystemExit(main())
