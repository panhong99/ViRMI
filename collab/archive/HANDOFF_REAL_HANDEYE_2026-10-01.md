# Claude handoff: Real M1013 + D455 eye-to-hand

작성일: 2026-10-01. 다음 작업자는 시작 전에 `CLAUDE.md`, `collab/CONTEXT.md`, `collab/DECISIONS.md`, `collab/LOG.md`, `collab/HUMAN_NOTES.md`도 읽는다. Human의 마지막 메시지는 한국어였으므로 한국어로 응답한다. 역할은 Builder다.

## Human 목표

실제 Doosan M1013의 flange에 ArUco를 달고, 고정 RealSense D455에서 `T_cam_marker`를 관측하면서 robot base 기준 flange pose를 얻어 eye-to-hand `T_base_cam`을 검증/계산하려 한다. 당장은 펜던트에서 값을 읽어 수동 입력했고 31개 샘플을 모았다. 이후에는 PC와 Doosan controller 사이의 기존 IP socket 연결을 이용해 flange `x y z Rx Ry Rz` pose를 자동으로 읽고 싶어 한다. 현재 연결의 구체적인 protocol/API는 확인되지 않았다.

## 필수 안전/파일 규칙

- 실제 robot 또는 ROS2 motion/jog/waypoint 명령을 보내지 않는다. 이 프로젝트의 시뮬레이션은 허용되지만, 실기에서는 read-only pose retrieval만 한다.
- `calibration/data/eye_to_hand/result.json` 및 `calibration/data/eye_to_hand/T_base_cam.txt`는 절대로 덮어쓰지 않는다. 기록 당시 SHA256은 각각 `847d6d3278bb4e9353798264936c0fc90466782f572f43fb9f6c2e4b2eb3d13d`, `c8b0ff63d1fc3fe61b690e75431f40b7adb08557026f68c2de43713ecf64757b`.
- 수동 실기 캡처 원본 `calibration/data/handeye_samples.json` 보존. SHA256 `3fa7e47120e1e4e1eedf403f20633ffe018b61ecc1880c85357d05922f8ab750`.
- 최근 잘못된 solve 산출물 `calibration/data/T_base_cam.json`은 현재 실기에 사용하지 않는다. 보호된 기존 파일과 별도 경로이나, 다음 실험 전 덮어쓰지 말고 새 out path를 사용한다.
- GUI scene 저장 위험 관련 기존 규칙 적용: Isaac GUI를 닫거나 default USD를 덮지 말고, 시뮬 테스트는 `--output-usd <scratch>.usda`.
- 모든 metric/decision을 `collab/LOG.md`, `collab/DECISIONS.md`에 기록한다. Human이 정한 metric을 바꿔야 하면 변경점과 이유를 기록하고 Human 리뷰를 요청한다.
- 저장소에는 이전 Human/Claude 작업을 포함한 많은 untracked/deleted 변경이 있다. 이를 정리, revert, stage, commit하지 않는다.

## 현재 실기 캡처

- D455 serial: `338122300585`.
- Marker detector: `DICT_4X4_50`, ID 1.
- Human은 캡처 프로그램 `--robot manual --marker_id 1 --marker_size 0.10 --serial 338122300585`로 수동 입력했다. 실제 검은 정사각 marker edge는 100 mm라고 보고되었으나 handoff 때 재측정/검증은 안 됨.
- `calibration/data/handeye_samples.json`: 31개 accepted samples, `#0`–`#30`. 화면 terminal에 마지막 accepted sample #30, 이어서 rejection 메시지가 보였지만 rejection은 저장되지 않는다. Pose는 여러 거리와 방향이며, marker tilt 대략 18.6–56.5°; 거리 0.764–1.365 m. jitter 대부분 0.12–1.30 mm.
- Solve가 `calibration/data/T_base_cam.json`에 쓴 결과는 사용 불가. Human screenshot에 나온 residual은 position mean **184.08 mm**, max **584.46 mm**, rotation mean **22.30°**, max **108.18°**. (이전 Codex 로그 entry에서 position mean을 148.08 mm로 잘못 옮겼으므로 이 handoff의 184.08 mm가 screenshot 값이다.)
- 출력 `T_base_cam`의 translation은 `[1.47763, 0.40591, 0.09934] m`, 회전행렬은 screenshot에 있음. 이 값은 residual이 커서 믿을 수 없다.
- 샘플 원본에 각 항목의 `posx`, `T_base_flange`, `T_cam_marker`, `jitter_mm`, `tilt_deg`, 거리/시간이 있다. 값 목록을 handoff에 복사해 중복 관리하지 않는다.
- Human은 마지막에 “이렇게 되면 나가리임?”이라 물었고, 이전 답변은 이 변환을 사용하지 말라고 안내했다. 아직 데이터 기반 원인 분석/재해석은 완료되지 않았다.

## 현재 분석/가설 (아직 미확정)

1. 위치와 회전 residual이 동시에 매우 크므로 이 `T_base_cam`은 무효다. 현재 성능 기준에 대한 판단이며 새 metric을 설정하는 뜻은 아니다.
2. 샘플이 실제로 robot base 기준 **tool flange** pose인지 확인해야 한다. 펜던트에서 선택한 항목이 TCP pose, user/tool 좌표계, world frame일 경우 `posx_to_T`와 다르다. `capture_handeye.py` solver는 `posx_to_T`에서 `x,y,z`를 mm→m 변환하고 Euler를 **intrinsic ZYZ degrees**로 해석한다. 실제 Doosan controller/pendant 정의와 일치하는지 확인하지 않았다.
3. 펜던트 값이 flange가 아닌 TCP/tool center 기준이면 tool/TCP offset을 알아야 한다. 현재 사람이 붙여 넣은 값의 좌표 원점은 검증 전이다.
4. `T_cam_marker` 계산은 카메라 intrinsics/ 왜곡 보정과 검은 marker edge 길이에 민감하다. Marker physical black-edge length와 캡처 시 실제 입력값을 재측정하고 `check_marker.py` 설정을 교차 확인한다.
5. 종이판/marker가 실제 flange에 대해 움직이거나 휘면 solver가 일관된 `T_flange_marker`를 찾지 못할 수 있다. rigid mounting 확인이 필요하다.
6. 샘플 index #0–30은 31개다. 앞서 제시된 일부 예상(“6개 찍어볼 것”)은 Human이 이미 찍은 데이터에 적용되지 않는다. 먼저 기존 데이터만 read-only 분석해 pose/측정 일관성, outlier 및 transform convention sensitivity를 점검한다. 임의로 샘플을 삭제하거나 outlier를 제외해 새 calibration이라고 결론 내리지 않는다.

## 코드 및 ROS/IP 현황

- `calibration/capture_handeye.py`: keyboard `C` 캡처, `S` solve; `--robot manual`에서 `x y z a b c` 입력. `--robot udp` reads localhost port 5006 from `ros2_flange_udp.py`.
- `calibration/calib_utils.py`: hand-eye solver and pose conversion. `posx_to_T` Euler assumption described above.
- `calibration/ros2_flange_udp.py`: reads `get_current_tool_flange_posx(DR_BASE)` and `get_current_posj()` then UDP localhost. It does not move robot. Human hit a Doosan module import error (`NameError: SetSingularityHandlingForce is not defined`) while trying it.
- Root cause found: current generated `dsr_msgs2` lacks optional `SetSingularityHandlingForce`, but `DSR_ROBOT2.py` creates its client unconditionally at import. Source and two install copies now have the client creation conditional on that symbol existing. These edits are in `/home/pan/pan/doosan_ws/...`, outside the 6D_pose repository. Only syntax was checked, not actual ROS client/service import/use.
- Correct ROS environment uses `/usr/bin/python3`; Miniconda base hijacks `python3` and caused Python/ROS dependency mismatch (`rclpy` + missing yaml / ABI issue). Sanitized shell with ROS Jazzy + Doosan setup showed system Python imports `rclpy,yaml` successfully. No robot bringup nor service was tested.
- Human clarified PC and robot controller already share an IP socket connection. The exact socket protocol/client software/service and whether it publishes current flange pose remain unknown. Do not assume arbitrary raw socket parsing or invent a packet format.
- Next implement/advise automatic pose retrieval by first identify what existing IP socket connection is (Doosan API, ROS2 TCP connection, status/monitoring socket, or existing app); inspect project/local Doosan SDK/API docs and current connection config, without sending motion commands. Prefer native supported read-only current flange/base pose API. If it returns TCP pose, transform to flange using registered TCP calibration, or configure flange/TCP=0 explicitly. Then adapt `capture_handeye` input source with timestamp/freshness/stillness guard. Keep network receiver and capture over loopback if both processes share same PC; the controller link is a separate connection.

## Suggested next steps

1. Validate sample data with an offline report only (do not overwrite existing results): summarize expected values/conventions, sample outliers, solve residuals under supported Euler conventions and possible TCP offset assumptions. Save exploratory outputs separately and clearly mark unvalidated assumptions.
2. Ask Human for/inspect the pendant screen selected field names and active coordinate frames, plus the actual TCP setup values (Flange/TCP selection, base/user frame). This information is essential before blaming the numerical solver.
3. Verify marker actual black edge with ruler/caliper, camera resolution, depth/camera intrinsics path used, and that all samples used the same settings.
4. Inspect Doosan's existing PC↔controller IP socket setup read-only, determine supported state endpoint/SDK and packet structure from actual configuration or vendor source. Do not use `ros2_run_waypoints.py --execute` or auto waypoints on real hardware.
5. Only after source pose frame/convention is confirmed, re-solve to a separate file; compare held-out/consistency and independent measurement before any use of T_base_cam for object coordinates or robot motion.

## Handoff prompt for Claude

Paste this after Claude reads the project files:

> Continue as Builder on `/home/pan/pan/6D_pose`. Read `CLAUDE.md`, `collab/CONTEXT.md`, `collab/DECISIONS.md`, `collab/LOG.md`, `collab/HUMAN_NOTES.md`, and `collab/HANDOFF_REAL_HANDEYE_2026-10-01.md`. We collected 31 real hand-eye samples, but solve residual is 184.08 mm mean / 584.46 mm max and 22.30° mean / 108.18° max, so the transform is invalid. Do not collect again or discard samples yet. First do an offline, read-only forensic analysis of `calibration/data/handeye_samples.json`: verify solver inputs/conventions, per-sample residuals/outliers, and whether plausible Doosan Euler/frame/TCP conventions explain the result. Preserve the raw file and never overwrite `calibration/data/eye_to_hand/result.json` or `T_base_cam.txt`; save any diagnostic results to a new clearly named file. Human says PC and M1013 controller already share an IP socket, so inspect the existing connection/config and identify a supported read-only API for current flange/base pose; do not invent a socket protocol and do not send robot motion commands. Report what exact pendant/controller details are needed if conventions remain ambiguous. Log findings in `collab/DECISIONS.md` and `collab/LOG.md`.
