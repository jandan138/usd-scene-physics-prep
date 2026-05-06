#!/bin/bash
# Cleanup layout intermediate files - keep only layout.usd
# Usage: ./cleanup_layout_artifacts.sh [--dry-run] <dataset-root>

set -euo pipefail

DRY_RUN=false
if [ "${1:-}" = "--dry-run" ]; then
    DRY_RUN=true
    shift
fi

DATASET_ROOT="${1:-/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset}"
SCENES_DIR="$DATASET_ROOT/GRScenes100"

echo "Cleaning up intermediate layout files..."
echo "Dataset root: $DATASET_ROOT"
echo "Dry run: $DRY_RUN"
echo ""

# Count files to delete
TOTAL_FILES=$(find "$SCENES_DIR" -name "layout.*.usd" -type f | wc -l)
TOTAL_SIZE=$(find "$SCENES_DIR" -name "layout.*.usd" -type f -exec stat --format='%s' {} + | awk '{sum+=$1} END {printf "%.1fMB", sum/1024/1024}')

echo "Files to delete: $TOTAL_FILES"
echo "Space to free: $TOTAL_SIZE"
echo ""

if [ "$DRY_RUN" = true ]; then
    echo "DRY RUN - Files that would be deleted:"
    find "$SCENES_DIR" -name "layout.*.usd" -type f | head -20
    if [ "$TOTAL_FILES" -gt 20 ]; then
        echo "... and $((TOTAL_FILES - 20)) more files"
    fi
    echo ""
    echo "To actually delete, run without --dry-run"
    exit 0
fi

# Safety check: ensure layout.usd exists in each scene dir before deleting
echo "Safety check: verifying layout.usd exists in all scene directories..."
MISSING_LAYOUT=0
while IFS= read -r scene_dir; do
    if [ ! -f "$scene_dir/layout.usd" ]; then
        echo "WARNING: $scene_dir/layout.usd missing! Skipping this directory."
        MISSING_LAYOUT=$((MISSING_LAYOUT + 1))
    fi
done < <(find "$SCENES_DIR" -type d -name "*_usd")

if [ "$MISSING_LAYOUT" -gt 0 ]; then
    echo "ERROR: $MISSING_LAYOUT scene directories missing layout.usd"
    echo "Aborting to prevent data loss."
    exit 1
fi

echo "All scene directories have layout.usd. Proceeding with cleanup..."
echo ""

# Delete intermediate files
DELETED=0
while IFS= read -r file; do
    rm -f "$file"
    DELETED=$((DELETED + 1))
    if [ $((DELETED % 500)) -eq 0 ]; then
        echo "Progress: $DELETED / $TOTAL_FILES files deleted..."
    fi
done < <(find "$SCENES_DIR" -name "layout.*.usd" -type f)

echo ""
echo "Cleanup complete!"
echo "Deleted: $DELETED files"
echo "Space freed: $TOTAL_SIZE"
