#!/usr/bin/env bash
# 보스 10/29: the safe demo - every setting fixed, the basket not measured (it stays put: tesollo/demo_safe/basket_fixed.json).
#   bash ~/pan/6D_pose/tesollo/demo_safe.sh              # as it is (remembers what is in the basket today)
#   bash ~/pan/6D_pose/tesollo/demo_safe.sh --new_demo   # the basket emptied: forget today's bowls / plates
# Before: robot connected (bash ~/pan/6D_pose/calibration/run.sh connect, control on the PC), hand on, DGManager closed.
# --stepwise: the post-grasp moves as tested on 10/29 (the newer one-spline path is the default of dg5fs_sdk.py itself).
# The code is the one tagged demo-safe-20261029; if a demo file changed since, it says so and asks before going on.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
TAG=demo-safe-20261029
FILES=(tesollo/dg5fs_sdk.py tesollo/taught_plan.py tesollo/basket_plan.py tesollo/arm_plan.py tesollo/arm_poses.yaml
       tesollo/poses.yaml tesollo/perceive.py tesollo/empty_quadrant.py tesollo/fixed_frames.py tesollo/taught_inserts.json
       tesollo/demo_safe/basket_fixed.json calibration/demo_watch.py calibration/ros2_nodes.py calibration/plate_plans.py
       calibration/basket_vision.py)
if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null && ! git diff --quiet "$TAG" -- "${FILES[@]}"; then
  echo "!! demo files changed since $TAG:"; git diff --stat "$TAG" -- "${FILES[@]}"
  read -r -p "run anyway? yes / Enter = stop: " ans
  [ "$ans" = yes ] || exit 1
fi
exec ~/miniconda3/bin/python3 "$REPO/tesollo/dg5fs_sdk.py" \
  --object auto \
  --basket_json "$REPO/tesollo/demo_safe/basket_fixed.json" \
  --taught 0 --taught_wrist_less 25 \
  --vel 60 --insert_vel 40 --lift 0.30 \
  --grasp_ms 3000 --motion_ms 3000 \
  --stepwise \
  "$@"
