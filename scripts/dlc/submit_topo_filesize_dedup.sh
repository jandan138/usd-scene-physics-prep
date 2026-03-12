#!/bin/bash
set -euo pipefail
# Submit per-category topo_filesize dedup jobs to DLC.
#
# This submits one DLC job per asset category (79 total).
# Each job runs dedup mode with --mode topo_filesize --filesize-tolerance 0.02.
#
# Prerequisites:
#   - ./dlc binary exists and is executable
#   - scripts/dlc/dedup_by_category.py supports --mode topo_filesize
#
# Usage:
#   bash scripts/dlc/submit_topo_filesize_dedup.sh [--dry-run]

CODE_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
ASSETS_ROOT=$CODE_ROOT/GRScenes-test1-normalized/GRScenes_assets
OUT_DIR=$CODE_ROOT/check_reports/topo_filesize_full

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "=== DRY RUN MODE — will only print commands, not submit ==="
fi

mkdir -p "$OUT_DIR"

SUBMITTED=0

for i in $(seq 0 78); do
    CMD_ARGS="dedup --chunk-id $i --chunk-total 79 --assets-root $ASSETS_ROOT --out-dir $OUT_DIR --merge-tolerance 0.005 --float-quantize-eps 1e-2 --mode topo_filesize --filesize-tolerance 0.02"

    if [ "$DRY_RUN" = true ]; then
        echo "[DRY] bash scripts/dlc/launch_job.sh topo_fs_dedup_${i} $i 79 \"\" \"$CMD_ARGS\""
    else
        echo "Submitting: topo_fs_dedup_${i}"
        bash "$CODE_ROOT/scripts/dlc/launch_job.sh" "topo_fs_dedup_${i}" "$i" 79 "" "$CMD_ARGS"
    fi
    SUBMITTED=$((SUBMITTED + 1))
done

echo ""
echo "Done. Submitted $SUBMITTED topo_filesize dedup jobs."
echo ""
echo "After all jobs complete, run 3-way union merge:"
echo "  python scripts/union_dedup_reports.py --batch \\"
echo "    --geom-dir check_reports/normalized_v2_dedup/ \\"
echo "    --shape-dir check_reports/shape_invariant_full/ \\"
echo "    --topo-dir $OUT_DIR/ \\"
echo "    --output-dir check_reports/union_merged_full/ \\"
echo "    --summary check_reports/union_merged_full/summary.json"
