#!/bin/bash
# =============================================================================
# Compose demo video from SUMO screenshots
#
# With step-length ≈ 1/60s, every frame is a real simulation screenshot
# at native 60fps. No frame interpolation needed — straight encode.
#
# Usage:
#   bash make_video.sh              # default: screenshots/
#   bash make_video.sh my_dir       # custom screenshot dir
# =============================================================================

set -e

SCREENSHOT_DIR="${1:-screenshots}"
OUTPUT="demo_output.mp4"

TOTAL=$(ls "$SCREENSHOT_DIR"/frame_*.png 2>/dev/null | wc -l)

if [ "$TOTAL" -eq 0 ]; then
    echo "No frames found in $SCREENSHOT_DIR"
    exit 1
fi

DURATION=$(echo "scale=1; $TOTAL / 60" | bc)
echo "Encoding $TOTAL frames @ 60fps = ${DURATION}s ..."

ffmpeg -y \
  -framerate 60 \
  -i "$SCREENSHOT_DIR/frame_%06d.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 \
  "$OUTPUT"

echo ""
echo "Done: $OUTPUT"
echo "  Frames: $TOTAL"
echo "  Duration: $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUTPUT" 2>/dev/null)s"
