#!/bin/bash
set -euo pipefail
# Submit per-chunk shape_invariant dedup jobs on the PRE-DEDUP dataset (v8).
#
# v8 = regenerated on prededup to get complete duplicate groups (old reports
# ran on post-v3-dedup data and missed pairs involving deleted assets).
#
# Usage:
#   bash scripts/dlc/submit_v8_prededup_shape_invariant_dedup.sh [--dry-run]

CODE_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
ASSETS_ROOT=$CODE_ROOT/GRScenes-test0-rebuilt-normalize-prededup/GRScenes_assets
OUT_DIR=$CODE_ROOT/check_reports/test0_rebuilt_dedup/v8_prededup/shape_invariant
CHUNK_TOTAL=10

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "=== DRY RUN MODE — will only print commands, not submit ==="
fi

mkdir -p "$OUT_DIR"

SUBMITTED=0

for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
    CMD_ARGS="dedup --chunk-id $i --chunk-total $CHUNK_TOTAL --assets-root $ASSETS_ROOT --out-dir $OUT_DIR --merge-tolerance 0.005 --float-quantize-eps 1e-2 --mode shape_invariant --hausdorff-threshold 0.05"

    if [ "$DRY_RUN" = true ]; then
        echo "[DRY] bash scripts/dlc/launch_job.sh v8p_shape_dedup_${i} $i $CHUNK_TOTAL \"\" \"$CMD_ARGS\""
    else
        echo "Submitting: v8p_shape_dedup_${i}"
        bash "$CODE_ROOT/scripts/dlc/launch_job.sh" "v8p_shape_dedup_${i}" "$i" "$CHUNK_TOTAL" "" "$CMD_ARGS"
    fi
    SUBMITTED=$((SUBMITTED + 1))
done

echo ""
echo "Done. Submitted $SUBMITTED shape_invariant dedup jobs (${CHUNK_TOTAL} chunks)."
echo "Output dir: $OUT_DIR"
