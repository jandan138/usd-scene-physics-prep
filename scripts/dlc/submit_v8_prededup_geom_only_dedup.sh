#!/bin/bash
set -euo pipefail
# Submit per-chunk geom_only dedup jobs on the PRE-DEDUP dataset (v8).
#
# v8 = regenerated on prededup to get complete duplicate groups (old reports
# ran on post-v3-dedup data and found 0 geom_only groups).
#
# 114 categories total. 10 chunks → ~11 cats per worker.
#
# Usage:
#   bash scripts/dlc/submit_v8_prededup_geom_only_dedup.sh [--dry-run]

CODE_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
ASSETS_ROOT=$CODE_ROOT/GRScenes-test0-rebuilt-normalize-prededup/GRScenes_assets
OUT_DIR=$CODE_ROOT/check_reports/test0_rebuilt_dedup/v8_prededup/geom_only
CHUNK_TOTAL=10

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
    echo "=== DRY RUN MODE — will only print commands, not submit ==="
fi

mkdir -p "$OUT_DIR"

SUBMITTED=0

for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
    CMD_ARGS="dedup --chunk-id $i --chunk-total $CHUNK_TOTAL --assets-root $ASSETS_ROOT --out-dir $OUT_DIR --merge-tolerance 0.005 --float-quantize-eps 1e-2 --mode all"

    if [ "$DRY_RUN" = true ]; then
        echo "[DRY] bash scripts/dlc/launch_job.sh v8p_geom_dedup_${i} $i $CHUNK_TOTAL \"\" \"$CMD_ARGS\""
    else
        echo "Submitting: v8p_geom_dedup_${i}"
        bash "$CODE_ROOT/scripts/dlc/launch_job.sh" "v8p_geom_dedup_${i}" "$i" "$CHUNK_TOTAL" "" "$CMD_ARGS"
    fi
    SUBMITTED=$((SUBMITTED + 1))
done

echo ""
echo "Done. Submitted $SUBMITTED geom_only dedup jobs (${CHUNK_TOTAL} chunks)."
echo "Output dir: $OUT_DIR"
