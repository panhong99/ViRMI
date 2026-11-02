#!/usr/bin/env bash
# 10/15 night result: sim pin-target insertion, all 4 quadrants (seated & exact 0.99 / 0.98 on two never-tuned layout sets)
#   plans:  cd calibration && python plate_plans.py random_plans --sim --far_flip --n N --x 0.757 0.857 --y -0.242 -0.142 \
#             --yaw_range 10 --plate_x 0.808 0.928 --plate_y 0.259 0.359 --basket_z 0.145 --phi_step 10 --seed S   (-> replay.jsonl)
#   replay: bash rl/basket/replay_final.sh <replay.jsonl> <out.json>
#   Q1 flipped wrist, no top re-grip, lean 0.5, aim +2.5 cm | Q2 flipped wrist, lean 0, +3.8 cm | Q3 lean -0.5, -4.64 cm | Q4 lean 0.5, -3.5 cm
#   (lean = factor of the tine lean at the release; aim = basket x shift of the closed-loop target; sim only, not the real-robot trim)
cd "$(dirname "$0")/../.."
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV /home/pan/IsaacLab/isaaclab.sh -p rl/basket/expert.py replay --headless --closed_loop --cl_top --cl_top_q Q1:0 \
  --cl_iter 2 --cl_dx_q Q1:0.025,Q2:0.038,Q3:-0.0464,Q4:-0.035 --cl_lean_q Q1:0.5,Q2:0,Q3:-0.5,Q4:0.5 --per_run 2 --plans "$1" --out "$2" "${@:3}"
