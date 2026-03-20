#!/bin/bash
set -euo pipefail
# Submit Phase 2 (scene compensation) job to DLC.
#
# Phase 2 reads the asset reports from Phase 1 and applies inverse-transform
# compensation to all scene layout files. This is a single job since it
# processes all scenes sequentially and needs the full asset_centers map.
#
# Prerequisites:
#   - All Phase 1 jobs must have completed successfully
#   - Phase 1 reports must exist in $REPORT_DIR
#   - scripts/normalize_asset_transforms.py supports --phase flag
#
# Usage:
#   bash scripts/dlc/submit_normalize_phase2.sh [--dry-run] [--assets-root PATH] [--scenes-root PATH] [--output-root PATH] [--report-dir PATH] [--centers-dir PATH] [--run-label LABEL] [--data-sources IDS]

CODE_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
ASSETS_ROOT=$CODE_ROOT/GRScenes-test1/GRScenes_assets
SCENES_ROOT=$CODE_ROOT/GRScenes-test1/GRScenes100
OUTPUT_ROOT=$CODE_ROOT/GRScenes-test1-normalized
REPORT_DIR=$CODE_ROOT/check_reports/normalize_v2
CENTERS_DIR=""
RUN_LABEL=""
DATA_SOURCES=""

DRY_RUN=false

usage() {
    cat <<EOF
Usage:
  bash scripts/dlc/submit_normalize_phase2.sh [options]

Options:
  --assets-root PATH   Override Phase 2 assets root. Default: $ASSETS_ROOT
  --scenes-root PATH   Override scenes root. Default: $SCENES_ROOT
  --output-root PATH   Override normalized output root. Default: $OUTPUT_ROOT
  --report-dir PATH    Override phase2 report directory. Default: $REPORT_DIR
  --centers-dir PATH   Directory containing merged centers_*.json files.
  --run-label LABEL    Optional label inserted into the DLC job name.
  --data-sources IDS   Optional comma-separated DLC data source ids passed to launch_job.sh.
  --dry-run            Print the launch command without submitting the job.
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
        --centers-dir)
            CENTERS_DIR="${2:?Missing value for --centers-dir}"
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

if [ -z "$CENTERS_DIR" ]; then
    CENTERS_DIR=$(python3 - <<'PY' "$REPORT_DIR"
import os
import sys

report_dir = os.path.abspath(sys.argv[1])
print(os.path.normpath(os.path.join(report_dir, "..", "phase1", "centers_merged")))
PY
)
fi

if [ ! -d "$CENTERS_DIR" ]; then
    echo "ERROR: centers-dir not found: $CENTERS_DIR" >&2
    exit 1
fi

JOB_NAME="norm_p2_scenes"
if [ -n "$RUN_LABEL" ]; then
    JOB_NAME="norm_p2_${RUN_LABEL}_scenes"
fi

CMD_ARGS="normalize_assets --assets-root $ASSETS_ROOT --scenes-root $SCENES_ROOT --output-root $OUTPUT_ROOT --phase 2 --centers-dir $CENTERS_DIR --symlink-copy --report-dir $REPORT_DIR"

if [ "$DRY_RUN" = true ]; then
    echo "[DRY] bash scripts/dlc/launch_job.sh $JOB_NAME 0 1 \"$DATA_SOURCES\" \"$CMD_ARGS\""
else
    echo "Submitting Phase 2 (scene compensation)..."
    bash "$CODE_ROOT/scripts/dlc/launch_job.sh" "$JOB_NAME" 0 1 "$DATA_SOURCES" "$CMD_ARGS"
fi

echo "Done."
