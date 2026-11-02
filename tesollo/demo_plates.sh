#!/usr/bin/env bash
# 보스 11/02: two plates in a row by the GT replay (the 11/02 10:08 Q3 pin 6 success moved along the row, 20 mm pins):
# the 1st (red) into pin P1, the 2nd (white) into pin P2, one tesollo/demo_safe.sh run each, Enter in between.
#   bash /home/pan/pan/6D_pose/tesollo/demo_plates.sh                 # pins 4, 10, the basket emptied first
#   bash /home/pan/pan/6D_pose/tesollo/demo_plates.sh --pins 3 9      # other pins (P2 > P1 + 2: the 2nd on the robot side)
#   bash /home/pan/pan/6D_pose/tesollo/demo_plates.sh --only 2        # the 2nd plate alone (today's memory kept)
#   bash /home/pan/pan/6D_pose/tesollo/demo_plates.sh --dry           # print the commands only
# Anything else is passed on to dg5fs_sdk.py (e.g. --leg_enter).  Residuals: yaw -6 deg both, x -10 mm red / +25 mm white.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
P1=4; P2=10; ONLY=""; DRY=0; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --pins) P1="$2"; P2="$3"; shift 3 ;;
    --only) ONLY="$2"; shift 2 ;;
    --dry) DRY=1; shift ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
for p in "$P1" "$P2"; do
  [[ "$p" =~ ^[0-9]+$ ]] && [ "$p" -ge 1 ] && [ "$p" -le 12 ] || { echo "pins are 1-12 (got $p)"; exit 1; }
done
[ "$P2" -ge $((P1 + 3)) ] || { echo "the 2nd pin must be >= the 1st + 3 (robot side, clear of the 1st plate): $P1 $P2"; exit 1; }
COMMON=(--taught_pivot slot --inhand_check --gt_replay --gt_pitch_mm 20 --plates_by_order --plate_pins "$P1" "$P2"
        --gt_dyaw_deg -6 --gt_dx_by_order -10 25)
run() {   # $1 = plate number, rest = extra args
  local n="$1"; shift
  local cmd=(bash "$REPO/tesollo/demo_safe.sh" "${COMMON[@]}" "$@" "${EXTRA[@]}")
  echo; echo "=== plate $n ($([ "$n" = 1 ] && echo "red -> pin $P1" || echo "white -> pin $P2")) ==="; echo "${cmd[*]}"
  [ "$DRY" = 1 ] && return 0
  read -r -p "put the plate on the grasp spot, Enter = go, q = stop: " ans
  [ "$ans" = q ] && exit 0
  DEMO_SAFE_YES=1 "${cmd[@]}" || { echo "!! plate $n run ended with an error"; return 1; }
}
if [ "$DRY" = 0 ] && git rev-parse -q --verify refs/tags/demo-safe-20261029 >/dev/null \
   && ! git diff --quiet demo-safe-20261029 -- tesollo/dg5fs_sdk.py tesollo/taught_plan.py; then
  echo "!! demo files changed since demo-safe-20261029 (GT replay etc., 11/02)"
  read -r -p "run anyway? yes / Enter = stop: " ans
  [ "$ans" = yes ] || exit 1
fi
if [ "$ONLY" != 2 ]; then
  run 1 --new_demo || { read -r -p "go on to plate 2 anyway? yes / Enter = stop: " ans; [ "$ans" = yes ] || exit 1; }
fi
[ "$ONLY" = 1 ] || run 2
echo; echo "done: today's memory $(cat /tmp/tesollo_demo_state.json 2>/dev/null)"
