#!/usr/bin/env python3
"""TDD test: Verify parallel workspace asset categories are normalized to underscore style.

This test should FAIL before running merge_asset_categories_test1.py
and PASS after the normalization is complete.
"""

import os
import pytest

# The 16 category pairs that should be normalized
CATEGORY_MERGES = {
    "bathtub": "bath_tub",
    "bookshelf": "book_shelf",
    "chestofdrawers": "chest_of_drawers",
    "coffeemaker": "coffee_maker",
    "dishwasher": "dish_washer",
    "electriccooker": "electric_cooker",
    "nightstand": "night_stand",
    "shoppingtrolley": "shopping_trolley",
    "shoecabinet": "shoe_cabinet",
    "sideboardcabinet": "sideboard_cabinet",
    "sofachair": "sofa_chair",
    "teatable": "tea_table",
    "trashcan": "trash_can",
    "tvstand": "tv_stand",
    "washingmachine": "washing_machine",
    "Musical_instrument": "musical_instrument",
}

PARALLEL_DATASET_ROOT = "/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset"


def test_no_old_category_folders_remain():
    """Verify no old (non-underscore) category folders exist in GRScenes_assets."""
    assets_root = os.path.join(PARALLEL_DATASET_ROOT, "GRScenes_assets")
    
    for old_cat in CATEGORY_MERGES.keys():
        old_path = os.path.join(assets_root, old_cat)
        assert not os.path.exists(old_path), f"Old category folder still exists: {old_path}"


def test_canonical_category_folders_exist():
    """Verify all canonical (underscore) category folders exist in GRScenes_assets."""
    assets_root = os.path.join(PARALLEL_DATASET_ROOT, "GRScenes_assets")
    
    for new_cat in CATEGORY_MERGES.values():
        new_path = os.path.join(assets_root, new_cat)
        assert os.path.exists(new_path), f"Canonical category folder missing: {new_path}"
        assert os.path.isdir(new_path), f"Canonical path exists but is not a directory: {new_path}"


def test_layout_usd_references_use_canonical_paths():
    """Spot-check: sample layout.usd files to ensure they use canonical category paths."""
    import glob
    
    layout_files = glob.glob(
        os.path.join(PARALLEL_DATASET_ROOT, "GRScenes100", "*", "layout.usd")
    )
    
    # Sample at most 5 scenes
    sample_layouts = layout_files[:5]
    
    for layout_path in sample_layouts:
        with open(layout_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Check old paths are absent
        for old_cat in CATEGORY_MERGES.keys():
            needle = f"GRScenes_assets/{old_cat}/"
            assert needle not in content, (
                f"layout.usd {layout_path} still references old category: {old_cat}"
            )
        
        # Check canonical paths are present (at least one per file)
        found_canonical = False
        for new_cat in CATEGORY_MERGES.values():
            if f"GRScenes_assets/{new_cat}/" in content:
                found_canonical = True
                break
        
        # Only assert if the file has any asset references at all
        if "GRScenes_assets/" in content:
            assert found_canonical, (
                f"layout.usd {layout_path} has asset references but no canonical categories"
            )
