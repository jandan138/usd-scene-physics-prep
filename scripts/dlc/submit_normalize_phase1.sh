#!/bin/bash
set -euo pipefail
# Submit per-category Phase 1 (asset normalization) jobs to DLC.
#
# This submits one DLC job per asset category (79 total).
# Each job runs normalize_assets mode with --phase 1 --category <cat>.
#
# Prerequisites:
#   - ./dlc binary exists and is executable
#   - scripts/normalize_asset_transforms.py supports --phase flag
#
# Usage:
#   bash scripts/dlc/submit_normalize_phase1.sh [--dry-run] [--assets-root PATH] [--scenes-root PATH] [--output-root PATH] [--report-dir PATH] [--run-label LABEL] [--data-sources IDS]

CODE_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
ASSETS_ROOT=$CODE_ROOT/GRScenes-test1/GRScenes_assets
SCENES_ROOT=$CODE_ROOT/GRScenes-test1/GRScenes100
OUTPUT_ROOT=$CODE_ROOT/GRScenes-test1-normalized
REPORT_DIR=$CODE_ROOT/check_reports/normalize_v2
RUN_LABEL=""
DATA_SOURCES=""

DRY_RUN=false

usage() {
    cat <<EOF
Usage:
  bash scripts/dlc/submit_normalize_phase1.sh [options]

Options:
  --assets-root PATH   Override Phase 1 assets root. Default: $ASSETS_ROOT
  --scenes-root PATH   Override scenes root. Default: $SCENES_ROOT
  --output-root PATH   Override normalized output root. Default: $OUTPUT_ROOT
  --report-dir PATH    Override phase1 report root. Each category writes to
                       <report-dir>/<category>/. Default: $REPORT_DIR
  --run-label LABEL    Optional label inserted into DLC job names.
  --data-sources IDS   Optional comma-separated DLC data source ids passed to launch_job.sh.
  --dry-run            Print launch commands without submitting jobs.
  --help               Show this message.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --assets-root)
            ASSETS_ROOT="${2:?Missing value for --assets-root}"
            shift 2
            ;;
        --scenes-root)
            SCENES_ROOT="${2:?Missing value for --scenes-root}"
            shift 2
            ;;
        --output-root)
            OUTPUT_ROOT="${2:?Missing value for --output-root}"
            shift 2
            ;;
        --report-dir)
            REPORT_DIR="${2:?Missing value for --report-dir}"
            shift 2
            ;;
        --run-label)
            RUN_LABEL="${2:?Missing value for --run-label}"
            shift 2
            ;;
        --data-sources)
            DATA_SOURCES="${2:?Missing value for --data-sources}"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [ "$DRY_RUN" = true ]; then
    echo "=== DRY RUN MODE — will only print commands, not submit ==="
fi

mkdir -p "$REPORT_DIR"

SUBMITTED=0
SKIPPED=0

for cat_path in "$ASSETS_ROOT"/*/; do
    cat=$(basename "$cat_path")

    # Skip non-directory entries (e.g. Asset_annotation.json)
    if [ ! -d "$cat_path" ]; then
        continue
    fi

    CAT_REPORT_DIR="$REPORT_DIR/$cat"
    mkdir -p "$CAT_REPORT_DIR"

    CMD_ARGS="normalize_assets --assets-root $ASSETS_ROOT --scenes-root $SCENES_ROOT --output-root $OUTPUT_ROOT --category $cat --phase 1 --symlink-copy --report-dir $CAT_REPORT_DIR"
    JOB_NAME="norm_p1_${cat}"
    if [ -n "$RUN_LABEL" ]; then
        JOB_NAME="norm_p1_${RUN_LABEL}_${cat}"
    fi

    if [ "$DRY_RUN" = true ]; then
        echo "[DRY] bash scripts/dlc/launch_job.sh $JOB_NAME 0 1 \"$DATA_SOURCES\" \"$CMD_ARGS\""
    else
        echo "Submitting: $JOB_NAME"
        bash "$CODE_ROOT/scripts/dlc/launch_job.sh" "$JOB_NAME" 0 1 "$DATA_SOURCES" "$CMD_ARGS"
    fi
    SUBMITTED=$((SUBMITTED + 1))
done

echo ""
echo "Done. Submitted $SUBMITTED Phase 1 jobs."
