# HANDOFF to Codex (from Claude) — 2026-10-06

Project: `/home/pan/pan/6D_pose` (human-in-the-loop robotics). Role: **Builder**. Human writes Korean/English; answer in the language of the last message; `collab/` notes are Korean.
**Latest (2026-10-06 19:30, Claude): `rl/` was reorganised — file names in §2-§5 below are historical; use this block.**

### 2026-10-07 update (Claude) — read first
- Human: **robot speed slowest is the TOP priority (above success).** v10 fast policy (98.3%, ~0.9 m/s, bobbed the plate 39 cm) rejected.
- v11 slow (DECISIONS 2026-10-07): speed caps inside the env (TCP 5 cm/s, rotation 10 deg/s, gripper 20 mm/s; action scale 0.0025, windup 1 cm), lift band 10 cm + `over_lift`, hold needs plate speed < 3 cm/s (from position change), `hold_still` / `held_speed` / `held_accel` penalties, episodes 20 s / 30 s. Experiment `m1013_2f85_rim_slow`, fine-tuning from v10 model_4996 since 09:49.
- `eval.py demo` = home -> MoveJ (15 s) -> ready -> policy, GUI. `eval.py gate` now also reports TCP speed and plate bobbing; `--speed_cap` overrides the cap.
- **IL impact**: `IL/data` was collected with the old action scale/dynamics; re-collect before BC on v11.
- W&B: no default entity on the account -> `train.py` falls back to offline; sync after the Human sets an entity.
- Figures: no grid, no top/right spines, no overall title (axis labels ok).
- Real gripper: commanded only from the PC (USB-RS485 /dev/ttyUSB0, `calibration/gripper.py`), slowest speed hard-coded, **force 64/255 = Human standard (no slip)**. Agents read status only; the Human runs motion commands.
- Final goal: stand the plate in the dish tray (pin pitch 20 mm, ~2-week demo). Place pose will be taught by pendant; see DECISIONS 2026-10-07 tray entries.

### Current RL code (`rl/`, run everything with `~/IsaacLab/isaaclab.sh -p`)
| file | role |
|---|---|
| `config.yaml` | every number: paths, robot actuators, cell geometry (box 114.6 mm, rim target, ready axes), task thresholds, action (scale, hold windup, orientation lock), shield margins, PhysX, reward terms per stage `[weight, params]`, PPO, curriculum |
| `plate_env.py` | Isaac Lab task = v10 behaviour without version flags: `make_env_cfg(stage, num_envs, viewer)`, `_state()` (cached task quantities), `SafeDiffIKAction` (held TCP target + orientation lock + shield), observations (43-dim, same order as v4..v10 checkpoints), `REWARD_FUNCS`, terminations, events (every episode starts at the ready joints) |
| `rl_utils.py` | `load_config`, `SafetyShield`, checkpoint/run helpers, `run_isaac` (subprocess + log watchdog), `ppo_runner_cfg`, `load_policy` |
| `train.py` | `curriculum --stage 2 --init <model.pt>` (chunks + deterministic gate; stops after the stage, Human GUI check before the next) / `chunk` (one Isaac PPO run, rsl_rl `OnPolicyRunner` directly) |
| `eval.py` | `gate` (EVAL_JSON) / `watch` (GUI, follows newest ckpt) / `trace` (per-second state + reward terms, `--bias_z`) / `scripted` (no-policy grasp + zero-action drift check) |
| `env_setup.py` | `urdf` / `plate` / `sdf` / `ready` asset builders |

Checkpoints: `~/IsaacLab/logs/rsl_rl/m1013_2f85_rim_slow/<date>_stage<k>/` (v11 slow; `m1013_2f85_rim/` = rejected v10 fast), curriculum status `rl/logs/m1013_2f85_rim_slow/`. Old experiments (`m1013_2f85_rim_v4..v10`) untouched; latest policy `m1013_2f85_rim_v10/2026-10-06_18-56-06_stage2/model_3000.pt` (loads unchanged in the new code). Old files: `rl/archive/2026-10-06_pre_refactor/` (incl. the earlier backups).
Verification of the refactor: regenerated URDF and cell SDF identical to the previous assets; `eval.py scripted` 6/8 scripted 5 s holds, zero-action hold stays put; `eval.py gate/trace` on v10 `model_3000` same behaviour as before; `train.py chunk` resumes and saves; curriculum driver end-to-end OK.
**calibration/ was reorganised the same way (19:55):** `config.yaml`, `calib_core.py`, `robot_model.py` (replaces kinematics/workcell/trajectory — `rl/` now imports `robot_model.frame_layout`), `sensors.py`, `calibrate.py wrist|fixed|gt|selftest`, `camera_tools.py preview|marker|probe`, `pipeline.py`, `ros2_nodes.py flange|publisher|mover` (system Python), `sim_scene.py`, `tests.py`, `run.sh connect|stream|pipeline|sim|tests`; old files in `calibration/archive/2026-10-06_pre_refactor/`. Re-solving saved samples reproduces the stored results exactly.
Code-layout rule (Human): few role files, functions/modes instead of new scripts, YAML for many options, no 1000+ line files.

Older handoffs still valid for the real-robot side: `HANDOFF_CODEX_2026-10-01.md`, `HANDOFF_REAL_HANDEYE_2026-10-01.md`, `NEXT_2026-10-02.md`.

## 0. First actions
1. Read `CLAUDE.md`, this file, `DECISIONS.md` (entries dated 2026-10-02/03, esp. "RL v3"), `LOG.md` (2026-10-02..06), `HUMAN_NOTES.md`.
2. `ps -eo pid,etime,args | awk '/train.py|curriculum/ && !/awk/'` — a training run may be active. Do not kill it without asking.
3. Append results (metrics + log paths + risks) to `DECISIONS.md`/`LOG.md` before ending a session. Core logic changes (reward, success criteria, split) go to `DECISIONS.md` and need Human review.

## 1. HARD RULES (Human)
- **Never send real robot / ROS2 motion commands.** Real-robot speed must always be the slowest (memory `feedback-robot-speed-slowest`). RL is simulation only. Since 2026-10-07 the RL env itself caps TCP 5 cm/s, rotation 10 deg/s, gripper 20 mm/s (speed first, above success).
- Never overwrite real hand-eye results in `calibration/data/real_handeye/*`.
- Real robot starts at the Human home joints **-5.98, -14.75, 117.74, -3.92, 7.46, -85.07 deg**; home -> ready is a slow MoveJ. RL episodes start at the ready joints (`rl/assets/ready_joints.json`, Human decision A 2026-10-06), not at home.
- The steel frame must never be hit: voxel safety shield (`rl/rl_utils.py` `SafetyShield`) + contact termination.
- Each curriculum stage must reach **>= 95 %** deterministic success, and the Human must see the GUI (`eval.py watch`) and approve before the next stage starts.
- Do not revert Human/earlier edits. Nothing is under git; keep backups before big rewrites.

## 2. Hardware / geometry (all in robot base frame: +x toward the steel frame, +y robot-left, z up, base bottom z=0)
- Doosan M1013 + **Robotiq 2F-85** via **AGC-DOO-KIT-85** coupling, **20 mm** (Human-confirmed). TCP (pinch point) = 0.1758 m from link_6 along +z. Gripper meshes from DeepMind mujoco_menagerie `robotiq_2f85` (copied to `assets/robotiq_2f85/`).
- Table top z = -0.020 (20 mm steel plate). Frame uprights x=0.85/1.65, y=+-0.56, rails z=1.16 (`calibration/workcell.py`, `calibration/data/real_cell_measured.json`).
- 029 plate = `demo_data/029_plate/tsdf/textured.obj` (266x268x25 mm dish: flat floor r<0.08 m, rim wall r 0.10-0.133 m rising to 25 mm). Sits on a **53.5 mm box** (Human estimate; 80x80 mm footprint is a placeholder) so a finger can slide under the rim. Target box identical, 0.45 m in -y.
- RL scene: plate (1.10, 0.0), target box (1.10, -0.45) (was y=-0.50; 4.5 cm from an upright < 5 cm shield). The real plate position is NOT measured; D455 position (Human: left side, middle of top frame) is only an eyeballed fit in `calibration/sim_home_scene.py`, and the old hand-eye `T_base_cam` (x~1.5, z~0.10) no longer matches -> recalibration needed if the camera moved.

## 3. What exists (this week)
- `calibration/`: real-robot pipeline (FP -> T_base_cam -> IK -> supervised ROS2 mover). `HOME_JOINTS_DEG` is the fallback start/return pose. Real plans still use `--tool_length_m` default **0** (gripper not yet included) — Human decision pending.
- `calibration/sim_home_scene.py` + `run_sim_home_scene.sh`: Isaac Sim kinematic scene (home pose, menagerie gripper, plate on box, target box, D455 view). `--render out.png` saves the camera view.
- Isaac Lab v2.3.2 at `~/IsaacLab` (rsl_rl installed; `flatdict` needed `setuptools<80` + `--no-build-isolation`).
- `rl/` (RL package, run with `~/IsaacLab/isaaclab.sh -p`):
  - `make_m1013_2f85_urdf.py` -> `assets/m1013_2f85.urdf` (finger linkage simplified: two prismatic fingers 0..0.0425 m; pad boxes are the only finger collisions), `m1013_2f85_cfg.py` (ArticulationCfg; **robot gravity disabled** because relative IK compounds sag), `smoke_robot.py`.
  - `make_plate_usd.py` + `assets/plate029_simplified.obj/usd` (open3d decimation to 3000 tris, convex-decomposition collision, mass 0.3 kg ASSUMED, friction 1.0 via startup events).
  - `make_cell_sdf.py` -> `assets/cell_sdf.npz`; `safety_shield.py` (torch FK + 1 cm distance fields; rejects arm targets < 5 cm to frame / < 2 cm to table); `check_shield.py`.
  - `pick_place_env.py` (env v3, stage via env var `PP_STAGE=1..4`), `pick_place_agent.py` (PPO, net 512-256-128), `train_pick_place.py` (registers gym ids `Pick-Place-M1013-2F85-v0/-Play-v0`, wraps Isaac Lab's rsl_rl train/play), `eval_policy.py` (deterministic eval, prints `EVAL_JSON`), `curriculum.py` (driver), `watch_latest.py` (GUI viewer following newest checkpoint), `check_*.py`/`trace_contact.py` (diagnostics).
- Actions: arm = TCP pose deltas via differential IK (`SafeDiffIKAction`, x0.03 per policy step at 30 Hz, shielded) + binary gripper. Observations: joints, TCP pose axes, plate pos (+-5 mm noise as FoundationPose stand-in), plate up-vector, target, progress timers, last action.

## 4. Stages and results (갱신: 2026-10-06 12:53 KST)
Human 확정 task는 **3단계**이며 각 단계는 Human home joints에서 시작한다. Stage 1은 rim 측면 사전파지 자세까지 접근(그리퍼 열림), Stage 2는 그 자세에서 파지 후 +z로 들어 올려 5초 hold, Stage 3은 이동 및 place. 다음 단계 진행 조건은 결정적 평가 성공률 ≥95%.
- 과거 v3 위쪽 파지 실행/지표는 잘못된 task 정의에 대한 결과이며 현재 기준 성과로 취급하지 않는다.
- **오늘 v4 Stage 1 통과:** 1,000 iter, 최근 50 iter train success 100%; 결정적 평가 2,048 episodes에서 success 100%, collision/plate_lost/timeout 0. 체크포인트 `/home/pan/IsaacLab/logs/rsl_rl/m1013_2f85_rim_v4/2026-10-06_11-18-23_stage1/model_999.pt`.
- **Stage 2 진행 중:** 12:19경 위 checkpoint에서 시작. 12:53 KST 관측 로그 iteration 1558/1999, mean reward 30.90, episode length 420, success 0%, timeout 100%, collision/plate_lost 0%; reward `grasp_reach=0.0621`, `close_at_grasp=0`, `lift=0`. 아직 파지·들기 성공이 확인되지 않았다. 학습은 임의 종료하지 않는다.
- Watch GUI가 Stage 2 checkpoint `model_1500.pt`를 로드한 것은 로그로 확인. GUI 한 프레임만으로 완전 정지를 판정할 수 없고, 현재까지 화면에서 실제 멈춤 위치/동작을 확인하지 못했으므로 체크포인트 동작 평가는 로그 지표와 구분한다.
- **정체 원인 후보(미확정):** `pick_place_env.py`의 `at_grasp`는 림 거리 <15 mm 및 tool 방향 <10°, finger 축 <15° 조건을 동시에 요구하며, `close_at_grasp` 보상은 `at_grasp` 또는 held일 때만 지급된다. 정밀 파지 조건이 만족되지 않는 동안 close/lift 보상이 0이 되는 희소 보상 구조와 접근 보상 local optimum 가능성이 있다. 실제 원인은 GUI trajectory/상태계측으로 검증해야 한다.
- 로그/status: `/home/pan/pan/6D_pose/rl/logs/curriculum_v4/{train_s2_00000.log,status.md,state.json}`. Stage 2 설정/기하 정의: `rl/pick_place_env.py`.
- 다음: Stage 2 학습 종료 시점까지 결과를 추적하되, 성공이 계속 0이면 checkpoint의 TCP-림 거리/방향 오차 및 finger closure를 계측해 접근 실패와 보상 설계 문제를 분리한다. 수정이 필요하면 Human review 전 core reward/success 로직 변경을 기록한다.

## 5. Running processes / commands
- Driver: `cd rl && python3 curriculum.py --start_stage N --eval_first --chunk 1000 --max_iters 15000` (resumes newest stage-N checkpoint; watchdog kills an Isaac run whose training log shows no new iteration for 15 min and retries up to 3x; passes a stage only at eval >= 0.95; records a play video). Status: `rl/logs/curriculum/status.md`, `state.json`, per-chunk logs there.
- Viewer: `cd ~/IsaacLab && PP_STAGE=1 ./isaaclab.sh -p ~/pan/6D_pose/rl/watch_latest.py --num_envs 4` (GUI display :1; costs training speed and ~4.5 GB VRAM).
- Eval: `PP_STAGE=1 ./isaaclab.sh -p rl/eval_policy.py --headless --num_envs 2048 --checkpoint <model.pt>`.
- 2026-10-06 current curriculum logs moved to `rl/logs/curriculum_v4/`; active latest run is Stage 2 from the `m1013_2f85_rim_v4` checkpoint. Check status/process before any restart or stop.
- Stage-1 4096 envs ~2.0-2.3 s/iter (RTX 5070 12 GB, ~4.4 GB). 15000 iters/stage ~ 9 h limit.

## 6. Known problems / lessons
- **2026-10-03 hang:** PhysX `totalAggregatePairsCapacity` (16k) overflowed with 4096 envs ("simulation will miss interactions", tens of thousands of lines) and the run froze at 100 % CPU for 2.8 days because the driver had no timeout. Fixed: total 1M / found-lost 16M capacities in `pick_place_env.py` + driver watchdog. Success numbers from 2026-10-03 chunks 1-4 may be affected by missed contacts (unverified).
- Filtered contact sensors cannot be used for one body vs many unique static prims in PhysX; unfiltered net force is used (any arm/gripper-body contact ends the episode, fingers excluded).
- Isaac Sim `app.close()` can exit the process before Python prints a traceback — scripts print it explicitly. Isaac's python has no scipy/trimesh; conda env `foundationpose` has them (open3d too).
- Do NOT `pkill -f <pattern>` from the agent shell: the pattern matches the shell's own command line and kills it (exit 144). Use `ps -eo pid,args | awk '$2 ~ /kit\/python\/bin\/python3$/ {print $1}' | xargs kill`.
- `isaaclab.sh` needs `TERM=xterm` and no conda vars; `sleep N` chained in one call is blocked by the harness (use `timeout N bash -c 'until ...'`).
- Training TCP speed and the shield are simulation conveniences; the real arm needs the slowest-speed rule and a real collision check.

## 7. Open decisions for the Human
1. Side-grasp redesign (proposal in DECISIONS 2026-10-06) — approve before retraining; stage 1 must be redone.
2. Real plan `--tool_length_m` (0 -> 0.1758) and `--tool_radius_m`.
3. Plate/box real measurements: plate xy, box height (53-54 mm; side grasp leaves only ~1 mm between the lower finger linkage and the table at that height), box footprint, plate mass/friction.
4. Whether to lower the learning rate / change the training schedule to stop the success collapses.
5. Camera position -> recalibrate hand-eye if it moved.

## 8. 2026-10-06 오후 추가: 고정 카메라 재캘리브레이션 (Claude)
- 고정 D455(338122300585)가 프레임 상단으로 옮겨져 이전 `T_base_cam*`는 모두 무효. 새 값: `calibration/data/real_handeye/20261006_122330_fromwrist/T_base_cam_pnp.json`(카메라 (1.218, 0.540, 1.127) m, 기대 정확도 5~10 mm). 방법·수치·한계는 `DECISIONS.md` "고정 D455 재캘리브레이션" 참조.
- `real_pose_dryrun.latest_t_base_cam()`은 이제 최신 세션의 `T_base_cam_pnp.json`을 우선 사용. `sim_home_scene.py`는 같은 파일로 카메라를 구동. RL(`rl/`)은 영향 없음(카메라 미사용).
- 로봇 연결(실로봇, 읽기 전용) 절차: PC에 `sudo ip addr add 192.168.127.5/24 dev enp5s0`(재부팅마다) → 터미널 1 `bash calibration/run_robot_connection.sh` → 터미널 2(ROS 환경 source 후) `python3 calibration/ros2_flange_udp.py`. 제어권은 티치패드, AI는 실로봇에 명령을 보내지 않는다.
- 다음에 하면 좋은 것: 마커 높이를 달리한 4~5곳 추가, 새 카메라로 `cube_gt_compare.py` 검증, 실제 배치(접시 (1.19, -0.03))에서의 RL 학습 레이아웃(목표 상자는 프레임 여유 5 cm 이상으로 조정).
