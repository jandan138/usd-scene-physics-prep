"""Shared USD transform helpers extracted for reuse across scripts.

Provides utilities for reading local prim transforms, computing accumulated
chain transforms between ancestor/descendant prims, and finding mesh prims
in a USD hierarchy.  These were originally private helpers in
``normalize_asset_transforms.py`` and are exposed here as public API so that
other scripts (e.g. dedup compensation, verification) can reuse them without
duplicating logic.
"""

from __future__ import annotations

from typing import List, Optional

try:
    from pxr import Gf, Usd, UsdGeom

    _PXR_IMPORT_ERROR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Gf = None  # type: ignore
    Usd = None  # type: ignore
    UsdGeom = None  # type: ignore
    _PXR_IMPORT_ERROR = e

__all__ = [
    "get_local_matrix",
    "get_chain_transform",
    "find_all_meshes",
]


def _ensure_pxr() -> None:
    """Raise at call time if pxr is unavailable."""
    if _PXR_IMPORT_ERROR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")


def get_local_matrix(prim) -> "Gf.Matrix4d":
    """Return the local transform matrix of a prim.

    Wraps ``UsdGeom.Xformable.GetLocalTransformation`` and handles the
    tuple-vs-matrix return value difference across pxr versions.
    """
    _ensure_pxr()
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def get_chain_transform(ancestor, descendant) -> "Gf.Matrix4d":
    """Compute accumulated local transform from ancestor's child down to descendant (inclusive).

    This gives the transform that maps points in descendant's local space
    into ancestor's local space (NOT including ancestor's own transform).

    Row-vector convention: p_ancestor = p_descendant * M_chain, where
    M_chain = M_descendant_local * M_parent_local * ... * M_ancestor_child_local.
    """
    _ensure_pxr()
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()  # top-down order

    result = Gf.Matrix4d(1.0)
    for p in chain:
        result = get_local_matrix(p) * result
    return result


def find_all_meshes(root_prim) -> List:
    """Find all UsdGeom.Mesh prims under *root_prim* (inclusive).

    Only prims with non-empty ``points`` attributes are included.
    """
    _ensure_pxr()
    meshes = []
    for prim in Usd.PrimRange(root_prim):
        mesh = UsdGeom.Mesh(prim)
        if mesh:
            pts = mesh.GetPointsAttr().Get()
            if pts and len(pts) > 0:
                meshes.append(prim)
    return meshes
