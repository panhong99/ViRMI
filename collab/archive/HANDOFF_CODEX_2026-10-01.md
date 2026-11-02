# HANDOFF to Codex (from Claude) — 2026-10-01

You are taking over a human-in-the-loop robotics project in `/home/pan/pan/6D_pose`. Role: **Builder** (implement, run, report).
The Human writes Korean and English; reply in the language of their last message. Project notes in `collab/` are Korean.

## 0. First actions
1. Read `CLAUDE.md`, `collab/CONTEXT.md`, `collab/DECISIONS.md` (last ~10 entries), `collab/LOG.md` (last ~6 entries), `collab/HUMAN_NOTES.md`.
2. Run the tests: `cd calibration && conda run -n foundationpose python -m unittest test_sim_eye_to_hand` (expect 8/8 OK).
3. `pgrep -af isaacsim_scene_m1013` — an Isaac GUI window may still be open; ask the Human before closing it (closing saves the live scene).
4. Before ending any session append results (metrics + log paths + risks) to `collab/DECISIONS.md` / `collab/LOG.md`. Changing a metric/pass criterion must be logged there as a review request.

## 1. Goal
Eye-to-hand pipeline for a Doosan **M1013** with fixed **RealSense D455** cameras:
`calibrate T_base_cam (flange ArUco) -> YOLOE-seg mask -> FoundationPose T_cam_object -> T_base_object = T_base_cam @ T_cam_object -> IK approach pose above a Rubik's cube`.
Currently **simulation only** (Isaac Sim 5.1, kinematic URDF replay, no PhysX). The Human will move it to the real robot later, so structure/transform chain must carry over.

## 2. HARD RULES
- **Never** send real robot / ROS2 motion commands. Sim only.
- Never overwrite `calibration/data/eye_to_hand/result.json` or `T_base_cam.txt` (v1 hand-eye; sha256 starts `847d6d3278bb4e93`, `c8b0ff63d1fc3fe6`). Full calibration refuses to overwrite without `--overwrite-calibration`.
- Each camera placement is a `SETUPS` entry with its **own data dir** (hand-eye is valid only for the camera pose it was solved at): `v1`->`eye_to_hand`, `side`->`eye_to_hand_side`, `frame_L`->`eye_to_hand_frame_L`, `frame_R`->`eye_to_hand_frame_R`.
- Closing the Isaac GUI **saves the live stage** to `calibration/data/m1013_d455_cube_scene.usda`. The Human edits the scene by hand in the GUI (e.g. placed `Cone_01/02`), so for test runs always pass `--output-usd <scratch>.usda` and never clobber the default.
- Keep position/GT comparison output (estimate / GT / delta mm) in terminal and GUI.
- Do not revert the Human's or earlier edits. Nothing is committed to git (`calibration/` is untracked); do not assume history.

## 3. Environment
- Isaac Sim: `/home/pan/isaacsim-5.1.0` (`python.sh`; wrapper `calibration/run_sim_eye_to_hand.sh` strips conda vars). GUI display `:1`. RTX 5070 12 GB.
- conda env `foundationpose`: YOLOE, FoundationPose, pyrealsense2, trimesh, scipy. `pycollada==0.9.3` was installed there today with `--no-deps`. Isaac's python has **no pycollada**, so the M1013 collision-mesh samples are cached in `calibration/data/m1013_collision_points.npz` (regenerate: `conda run -n foundationpose python calibration/workcell.py`; do not let tests overwrite it).
- Robot URDF: `/home/pan/pan/doosan_ws/install/dsr_description2/share/dsr_description2/urdf/m1013.urdf`. D455 serial `338122300585`. Cube mesh: `demo_data/077_rubiks_cube/google_16k/textured_57mm.obj` (57 mm, origin = centre, 7 flat sticker colours).

## 4. Commands
```
# full pipeline (headless capture -> YOLOE+FoundationPose -> Isaac GUI approach)
SETUP=frame_L DISPLAY=:1 ./calibration/run_yoloe_foundationpose_sim.sh --output-usd /tmp/scratch.usda
#   extra args go to both Isaac runs, e.g. --max-joint-speed-deg 5 --max-joint-accel-deg 5
#   --pregrasp-height-m 0.10 --tool-length-m 0.15 --tool-radius-m 0.06 --cube-yaw-deg 30 --cube-xy X Y --no-show-voxels
# hand-eye calibration for one setup (headless)
./calibration/run_sim_eye_to_hand.sh --setup frame_R --headless --exit-after-run --overwrite-calibration --output-usd /tmp/scratch.usda
# choose a calibration centre after a camera moves
conda run -n foundationpose python calibration/find_calib_centre.py --setup frame_L
# real D455 live preview (s = save, q = quit; read-only)
conda run -n foundationpose python calibration/real_d455_preview.py
# offline UDP/transform chain check (no Isaac, no camera)
conda run -n foundationpose python calibration/sim_udp_pose_pipeline.py
```
Verify the CLI after every edit of `isaacsim_scene_m1013_d455_cube.py` (an edit once deleted 4 options by accident): all 21 `add_argument`s must remain.

## 5. Code map (`calibration/`)
- `isaacsim_scene_m1013_d455_cube.py` — scene entry, `SETUPS`, D455 optics (`D455_REAL` 1280x720 89x58 deg, legacy 1920x1080), CLI, cube render (`add_rubiks_cube`).
- `sim_eye_to_hand.py` — `Kinematics` (URDF FK/IK), `run_pipeline` (calibration, FP approach, grasp frames, motion planning/playback, report JSON, GUI panel).
- `workcell.py` — `FRAME_CELL` (table/frame/cameras), `frame_layout`, `CollisionChecker` (1 cm voxel distance field, RRT + `tidy`), `add_voxel_overlay`.
- `trajectory.py` — PCHIP smooth path + curvature-aware speed/accel time parameterisation (pure numpy).
- `foundationpose_sim_pose.py` (conda) — YOLOE + FoundationPose on the Isaac capture -> `foundationpose_T_cam_object.json`.
- `find_calib_centre.py`, `real_d455_preview.py`, `calib_utils.py` (solver), `capture_handeye.py` + `ros2_*` (real hand-eye tools, untested on robot), `sim_udp_pose_pipeline.py`, `test_sim_eye_to_hand.py`.
- Real-time side: `real_time_project/main.py` (D455 -> detector -> FoundationPose -> UDP 5005 JSON via `pose_sender_kimm.py`, camera-frame `T_cam_object`). `--help` loads in this env; it was not run live.

## 6. What was done (history)
**2026-09-30 (baseline):** Isaac scene M1013 + D455 + cube; hand-eye in sim; YOLOE+FoundationPose path; per-setup data dirs; `side` setup (Human placed camera/cube in GUI); real Rubik CAD rendered (57 mm, removes size mismatch); all-zero start pose; top-down grasp-ready approach (j4~0, j6 aligned to cube yaw mod 90, pregrasp 100 mm; grasp only IK-checked); cube symmetry-aware rotation diagnostics.
**2026-10-01 (today):**
1. Real D455 captured with a live preview tool (`data/real_env/`, intrinsics json). Real depth valid 57-75 %, reflective optical table has holes.
2. Real cell from Human: table 1.80x1.20 m, frame height 1.20 m, 0.75 m between the two uprights carrying the camera bar, robot-object 1.00-1.10 m, two D455 ~45 deg. Modelled in `workcell.py` (box list drives both the Isaac visuals and the collision model). Assumed (NOT measured): robot base 30 cm from table edge, 0.75 m square frame, camera bar height, 40 mm profile.
3. Cameras placed at the Human's cones: L `(1.0722, 0.5706, 1.1654)`, R `(1.0688, -0.1324, 1.1043)`, aimed at the cube (pitch 63.3 / 82.9 deg below horizontal). Human chose to keep look-at aiming (not 45 deg) and to keep the assumed frame.
4. Collision: voxel distance field from the real M1013 collision meshes; margin 50 mm (planning 65 mm); `link_1` excluded (static 60 mm gap to table); hand optional as a cylinder; paths planned with RRT + shortcut + `tidy` (removes random wrist detours), smoothed (PCHIP) and re-timed. Voxels are drawn in the GUI (toggle button) and exported to `data/<setup>/workcell_voxels.npz/.json` for a real-robot dry-run check.
5. Motion: default 8 deg/s, 10 deg/s^2, GUI speed slider, replay glides back instead of teleporting. (Human repeatedly said motion was too fast/unstable.)
6. Calibration: frame setups use 30 poses, wider spread, **150 mm** marker (100 mm gave 7-16 mm optical-axis bias because the marker is only ~80-130 px). Full calibration now saves `mode: hand-eye calibration only` when the 45 mm cube ArUco is not visible.
7. GUI fixes: controls first in a ScrollingFrame (the Replay button had been pushed off-screen); no voxels drawn within 12 cm of a lens.

## 7. Current numbers (sim)
| setup | hand-eye error | cube in base | notes |
|---|---|---|---|
| v1 | 3.57 mm / 0.12 deg | 4.96 mm / (see logs) | original camera |
| side | 1.94 mm / 0.025 deg | 1.97 mm / 0.34 deg | PASS, FP-only 0.27 mm |
| frame_L | 9.42 mm / 0.135 deg (100 mm marker: 16.49) | 8.18 mm / 2.14 deg | position approach PASS, min clearance 63 mm, full FAIL (rotation > 2 deg), FP-only 1.8 mm |
| frame_R | 4.58 mm / 0.067 deg (100 mm marker: 7.42) | FP approach **not yet run** with the newest code | |
Almost all remaining position error is hand-eye bias along the camera optical axis.

## 8. Open risks / TODO (suggested order)
1. **Frame geometry is assumed.** The left cone lies outside the assumed frame; ask the Human for real upright/rail coordinates and robot-base position, then update `FRAME_CELL`. Their remark "the real robot would definitely hit the frame" is unresolved until geometry is real.
2. Run FP approach for `frame_R`; compare L vs R; consider fusing both cameras.
3. Hand-eye optical-axis bias: try bigger marker (200 mm) / ChArUco / solving marker scale; validate on real data independently.
4. Hand unknown: need flange->grasp-centre length, finger-closing axis, open width, radius -> `--tool-length-m/--tool-radius-m` (collision model has no hand until set).
5. frame_L cube rotation 2.14 deg (>2) from the steep view; yaw error -1.8 deg.
6. Not implemented: descend/grasp/retreat motion; node that receives real UDP pose -> base frame -> collision-checked dry-run; real D455 depth/FoundationPose quality test; Doosan posx ZYZ convention check and TCP registration.
7. Metric changes awaiting Human review: `collision_free` is part of position-approach pass in frame setups; per-setup calibration sampling (30/120 poses, spread); held-out threshold 5 mm (R failed it at 5.27 mm with the 100 mm marker).
8. Sim depth is ideal; do not quote sim mm as real accuracy.

## 9. Human preferences
Wants to understand each step (keeps asking "why"), prefers slower and smoother robot motion, wants things visible in the GUI, wants results recorded in `collab/`. Ask before closing windows or overwriting scenes.
