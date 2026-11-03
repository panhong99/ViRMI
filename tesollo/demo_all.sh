#!/usr/bin/env bash
# 보스 11/03: bowls first, then the two plates, one command.  Each object = one tesollo/demo_safe.sh run, Enter in between.
#   bowl(s): the default demo_safe.sh run (the camera sees a bowl -> an empty quadrant, Q1 / Q2 first)
#   plates : the same args as tesollo/demo_plates.sh (GT replay, red -> pin P1, white -> pin P2 of Q3)
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh                  # 1 bowl, then plates on pins 4, 10
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh --bowls 2        # 2 bowls first
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh --pins 3 9       # other pins
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh --quadrant Q4    # plates into Q4 (the Q3 GT moved across; Q3 must be empty)
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh --no_flow        # without --flow (the stop-and-go legs of demo_plates.sh)
# --flow by default (11/03 보스): one kept mover, timed even-knot splines at --vel 60 (insert / descend as before), the hand
# connected during the perception, GT planned before the lift.
#   bash /home/pan/pan/6D_pose/tesollo/demo_all.sh --dry            # print the commands only
# Anything else is passed on to dg5fs_sdk.py.  The basket memory is cleared once at the start (the 1st bowl's --new_demo).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
NB=1; P1=4; P2=10; DRY=0; QD=Q3; FLOW=(--flow); EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --bowls) NB="$2"; shift 2 ;;
    --pins) P1="$2"; P2="$3"; shift 3 ;;
    --dry) DRY=1; shift ;;
    --quadrant) QD="$2"; shift 2 ;;
    --no_flow) FLOW=(); shift ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
[[ "$QD" =~ ^Q[34]$ ]] || { echo "--quadrant is Q3 or Q4 (got $QD)"; exit 1; }
[[ "$NB" =~ ^[0-3]$ ]] || { echo "--bowls is 0-3 (got $NB)"; exit 1; }
for p in "$P1" "$P2"; do
  [[ "$p" =~ ^[0-9]+$ ]] && [ "$p" -ge 1 ] && [ "$p" -le 12 ] || { echo "pins are 1-12 (got $p)"; exit 1; }
done
[ "$P2" -ge $((P1 + 3)) ] || { echo "the 2nd pin must be >= the 1st + 3 (robot side, clear of the 1st plate): $P1 $P2"; exit 1; }
PLATE=(--taught_pivot slot --inhand_check --gt_replay --gt_pitch_mm 20 --plates_by_order --plate_pins "$P1" "$P2"
       --gt_dyaw_deg -6 --gt_dx_by_order -10 25 --plate_quadrant "$QD")   # = tesollo/demo_plates.sh COMMON + quadrant
run() {   # $1 = label, rest = args
  local what="$1"; shift
  local cmd=(bash "$REPO/tesollo/demo_safe.sh" "${FLOW[@]}" "$@" "${EXTRA[@]}")
  echo; echo "=== $what ==="; echo "${cmd[*]}"
  [ "$DRY" = 1 ] && return 0
  read -r -p "put it on the grasp spot, Enter = go, q = stop: " ans
  [ "$ans" = q ] && exit 0
  DEMO_SAFE_YES=1 "${cmd[@]}" || { echo "!! $what ended with an error"; return 1; }
}
next() { read -r -p "go on anyway? yes / Enter = stop: " ans; [ "$ans" = yes ] || exit 1; }
if [ "$DRY" = 0 ] && git rev-parse -q --verify refs/tags/demo-safe-20261029 >/dev/null \
   && ! git diff --quiet demo-safe-20261029 -- tesollo/dg5fs_sdk.py tesollo/taught_plan.py; then
  echo "!! demo files changed since demo-safe-20261029 (GT replay etc., 11/02)"
  read -r -p "run anyway? yes / Enter = stop: " ans
  [ "$ans" = yes ] || exit 1
fi
NEW=(--new_demo)
for i in $(seq 1 "$NB"); do
  run "bowl $i (an empty quadrant)" "${NEW[@]}" || next
  NEW=()
done
run "plate 1 (red -> $QD pin $P1)" "${PLATE[@]}" "${NEW[@]}" || next
run "plate 2 (white -> $QD pin $P2)" "${PLATE[@]}"
echo; echo "done: today's memory $(cat /tmp/tesollo_demo_state.json 2>/dev/null)"
