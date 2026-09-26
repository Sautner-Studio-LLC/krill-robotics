#!/usr/bin/env bash
# Grab a frame of the bench from one of kraken's BRIOs.
#
# Two traps this exists to avoid, both of which fail SILENTLY into a plausible-looking image:
#   1. A BRIO exposes four /dev/video* nodes and only the FIRST is the camera. The third
#      (video2 / video6) is a 340x340 greyscale IR sensor for face auth -- capture from it
#      and you get a tiny monochrome frame a model will happily describe as a dark room.
#      So: assert the device advertises mjpeg before trusting anything it returns.
#   2. The FIRST frame is underexposed -- auto-exposure needs ~12 frames to settle.
#      Measured 2026-09-24: frame 0 was 67 KB, frame 12 was 368 KB of the same scene.
#      Discarding warmup frames is not an optimisation, it is the difference between a
#      usable image and a dark one that looks like a lighting problem.
#
# Usage: bench-cam.sh [-d /dev/videoN] [-r WxH] [-c W:H:X:Y] [-o out.jpg]
set -euo pipefail

DEV=${BENCH_CAM:-/dev/video4}      # video0 = room camera, video4 = bench camera
RES=3840x2160
CROP=""
OUT="bench-$(date -u +%Y%m%dT%H%M%SZ).jpg"
WARMUP=12

while getopts "d:r:c:o:w:" o; do case $o in
  d) DEV=$OPTARG;; r) RES=$OPTARG;; c) CROP=$OPTARG;; o) OUT=$OPTARG;; w) WARMUP=$OPTARG;;
esac; done

if ! ffmpeg -hide_banner -f v4l2 -list_formats all -i "$DEV" 2>&1 | grep -q "mjpeg"; then
  echo "$DEV does not advertise mjpeg -- refusing. It is probably the IR sensor." >&2
  exit 1
fi

VF="select=gte(n\\,$WARMUP)"
[[ -n $CROP ]] && VF="$VF,crop=$CROP"

ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size "$RES" \
       -i "$DEV" -vf "$VF" -frames:v 1 -y "$OUT"

SZ=$(stat -c%s "$OUT")
(( SZ < 40000 )) && echo "WARNING: ${SZ}B is small for $RES -- frame may still be underexposed." >&2
echo "$OUT  (${SZ} bytes, $RES from $DEV)"
