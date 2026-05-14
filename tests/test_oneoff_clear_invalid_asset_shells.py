import os
import sys
from pathlib import Path

from pxr import Sdf, Usd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import oneoff_clear_invalid_asset_shells as mod


def _make_layout(path: Path, target_ref: str, keep_ref: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    layer = Sdf.Layer.CreateNew(str(path))
    stage = Usd.Stage.Open(layer.identifier)

    target = stage.DefinePrim("/Root/Meshes/Furnitures/other/target", "Xform")
    target.GetReferences().AddReference(target_ref)

    keep = stage.DefinePrim("/Root/Meshes/Furnitures/other/keep", "Xform")
    keep.GetReferences().AddReference(keep_ref)

    stage.GetRootLayer().Save()


def _authored_ref_paths(layout_path: Path, prim_path: str) -> list[str]:
    stage = Usd.Stage.Open(str(layout_path))
    prim = stage.GetPrimAtPath(prim_path)
    refs = prim.GetMetadata("references")
    if not refs:
        return []
    return [r.assetPath for r in refs.GetAddedOrExplicitItems()]


def test_apply_removes_only_target_invalid_asset_references(tmp_path):
    dataset = tmp_path / "dataset"
    target_ref = "../../../GRScenes_assets/other/d41d8cd98f00b204e9800998ecf8427e/usd/d41d8cd98f00b204e9800998ecf8427e.usd"
    keep_ref = "../../../GRScenes_assets/other/keep/usd/keep.usd"
    layout = dataset / "GRScenes100" / "commercial" / "SCENE_usd" / "layout.usd"
    _make_layout(layout, target_ref, keep_ref)

    (dataset / "GRScenes_assets" / "other" / "keep" / "usd").mkdir(parents=True)
    (dataset / "GRScenes_assets" / "other" / "keep" / "usd" / "keep.usd").write_text(
        "#usda 1.0\n",
        encoding="utf-8",
    )

    report = mod.process_dataset(
        dataset_root=dataset,
        invalid_assets=["other/d41d8cd98f00b204e9800998ecf8427e"],
        apply=True,
        quarantine_root=None,
        backup_root=tmp_path / "backup",
        report_path=None,
    )

    assert report["references_removed_total"] == 1
    assert report["touched_stages"] == 1
    assert _authored_ref_paths(
        layout, "/Root/Meshes/Furnitures/other/target"
    ) == []
    assert _authored_ref_paths(layout, "/Root/Meshes/Furnitures/other/keep") == [
        keep_ref
    ]


def test_apply_quarantines_annotation_only_asset_shell(tmp_path):
    dataset = tmp_path / "dataset"
    asset_dir = (
        dataset
        / "GRScenes_assets"
        / "person"
        / "351316cbb083f9f4df0cccd60cbfa848"
    )
    asset_dir.mkdir(parents=True)
    (asset_dir / "351316cbb083f9f4df0cccd60cbfa848_annotation.json").write_text(
        "{}\n",
        encoding="utf-8",
    )
    quarantine = tmp_path / "quarantine"

    report = mod.process_dataset(
        dataset_root=dataset,
        invalid_assets=["person/351316cbb083f9f4df0cccd60cbfa848"],
        apply=True,
        quarantine_root=quarantine,
        backup_root=tmp_path / "backup",
        report_path=None,
    )

    quarantined = (
        quarantine
        / "GRScenes_assets"
        / "person"
        / "351316cbb083f9f4df0cccd60cbfa848"
    )
    assert report["asset_shells_quarantined_total"] == 1
    assert not asset_dir.exists()
    assert quarantined.is_dir()
    assert (
        quarantined / "351316cbb083f9f4df0cccd60cbfa848_annotation.json"
    ).is_file()
