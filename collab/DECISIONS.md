# DECISIONS

결정과 이유를 누적한다. 최신 항목을 아래에 추가한다. 핵심 로직(loss, data split, metric 계산) 변경은 반드시 여기에 기록하고 Human 검토를 요청한다.

포맷:

```
## YYYY-MM-DD 제목
- 결정:
- 이유:
- 대안:
- 작성자: Claude | Codex | Human
```

## 2026-09-29 정확도 우선 시 옵션 조정 방침(코드 미변경)
- 결정: 핵심 로직은 바꾸지 않고 CLI 옵션(`--register_iter 10 --track_iter 5 --roi_patience 0 --conf/--imgsz`)으로 정확도를 먼저 비교한다. relock 비활성화 옵션 추가는 Human 확인 후 진행.
- 이유: `FoundationPose-plus-plus` 원본 기본값이 10/5이고, 9/28 커밋(`9f706c6`)의 ROI crop/relock이 속도 개선용이라 정확도에 영향을 줬을 가능성. 단 실측 근거는 아직 없음.
- 대안: 원본과 동일한 Kalman 갱신 순서로 맞추기, `--force_apply_color` 유사 옵션 추가.
- 작성자: Claude

## 2026-09-30 pose 전달 검증은 시뮬레이션 변환 검증부터 분리해 수행
- 결정: 실제 Doosan 연결·이동 전에 FoundationPose UDP packet의 `T_cam_obj`를 수신해 `T_base_obj = T_base_cam @ T_cam_obj` 및 안전 오프셋 flange 목표로 변환하는 시뮬레이션 전용 검증을 먼저 만든다. Isaac Sim M1013 시각화는 이 수치 검증 통과 뒤의 선택 단계로 둔다.
- 이유: 현재 5005 송신기는 존재하지만 수신/변환/목표 생성 노드는 없으며, 로컬 Isaac Sim 설치 경로도 발견되지 않았다. 수치 GT 기반 검증은 Isaac 설치나 실제 로봇 없이 frame 방향·m/mm 오류를 확정적으로 잡을 수 있다.
- 대안: Isaac Sim에서만 바로 검증. 단 설치/robot USD/articulation API 문제와 변환 오류가 섞여 원인 분리가 어렵다.
- 작성자: Codex

## 2026-09-30 simulation-only 접근 목표는 base Z +100 mm, flange 자세는 미검증으로 유지
- 결정: `sim_udp_pose_pipeline.py`는 `T_base_obj`의 translation에 base-frame `[0, 0, +0.100] m`를 더한 position-only 접근점을 생성한다. cube에서 복사한 orientation은 `UNVERIFIED`로 기록하고 실기 로봇 명령에 사용하지 않는다.
- 이유: 고정 카메라 변환과 UDP 전달을 검증하는 현 단계에서는 base 수직 안전 오프셋을 분명히 검증할 수 있다. 반면 M1013 flange 축/도구 좌표계 및 IK·충돌은 아직 실기 검증 전이다.
- 대안: object-local offset 또는 고정 flange orientation을 즉시 확정. 이들은 object orientation·그리퍼/툴 길이·Doosan convention 검증이 선행되어야 한다.
- 작성자: Codex

## 2026-09-30 sim-first hand-eye → target movement → real-world 재현 계획
- 결정: (1) Isaac Sim에서 고정 D455/ArUco-on-flange/M1013/cube로 `T_base_cam`을 hand-eye 방식으로 산출하고 GT 대비 calibration error를 측정한다. (2) 동일 변환 코드와 UDP packet으로 `T_base_obj` 및 접근 flange target을 만들고, 시뮬레이션에서 IK/도달을 검증한다. (3) 검증된 동일 알고리즘을 실제 D455/Doosan으로 옮긴다.
- 이유: simulation에는 GT가 있어 frame order, 단위, calibration error, IK reachability를 실제 로봇 위험 없이 분리 검증할 수 있다. 실기에서는 GT가 없으므로 펜던트/실측으로 별도 교차 검증한다.
- 안전: 실기 이동은 sim에서 수치/도달 검증 후에도 dry-run이 먼저이며, Human의 명시적 승인 전에는 어떠한 robot motion command도 실행하지 않는다.
- 작성자: Codex

## 2026-09-30 Isaac D455 optical axis는 계산된 look-at으로 cube centre를 향하게 함
- 결정: Stage-1 scene의 `/World/D455_Mount/color`는 손으로 정한 Euler angle 대신, USD camera의 local `-Z` optical axis가 cube centre를 정확히 향하도록 계산한 quaternion을 사용한다. GUI의 `Perspective` 뷰는 검증 대상 camera view가 아니므로, 검증 시 해당 camera prim을 viewport camera로 선택한다.
- 이유: 기존 `(66, 0, 43)` Euler 배치는 cube를 실제로 바라보는지 보장하지 않았고, frame/optical-axis convention 혼동을 초기에 제거해야 한다. 코드 내부 방향 검증은 0.000000 deg이다.
- 범위: 이는 simulation scene 배치값일 뿐 실제 D455 외부파라미터나 hand-eye `T_base_cam`의 추정값이 아니다. 실기 캘리브레이션은 별도로 수행한다.
- 작성자: Codex

## 2026-09-30 YOLOE 11s -> v8l 비교 및 코드 정리 (핵심 로직 미변경)
- 결정: `yoloe-v8l-seg.pt`를 repo 루트에 받아(.gitignore 등록됨) `--yoloe_checkpoint`로 11s와 비교한다. 기본값은 비교 결과 확인 후 Human이 결정. `_score()`(매 tracking 프레임 scorer forward)는 수동 재탐지 모드에서 쓰이지 않는 오버헤드로 보이나, 핵심 로직이라 profiling 후 Human 검토를 받고 변경한다.
- 이유: Human이 11s가 이전(v8l 사용) 대비 눈에 띄게 나쁘다고 관찰(시각 판단, 벤치마크 없음). 11s 약 10M / v8l 약 45M params(공식 자료 기억 기반, 미검증; 로컬 측정 53.5M은 모델 전체 포함). validation_interval/loss_patience/drift는 manual 모드에서 미사용이라 DINO/YOLOE 스크립트 차이는 원인 아님으로 정정.
- 대안: 원본 plus-plus 루프와 동일한 순수 모드 비교, mesh 원본(textured.obj) 비교.
- 작성자: Claude

## 2026-09-30 YOLOE 스크립트 자동 재탐지 기본 활성화 (Human 요청)
- 결정: real_time_v1_yolo.py의 --auto_recovery 기본 True(--no-auto_recovery로 수동). validation_interval 0.5, loss_patience 5, drift_score_ratio 0.3을 DINO 스크립트와 동일 기본값으로 옵션화. YOLOEPipeline.detect_bbox() 추가(infer와 동일 비용, mask는 버림). SUSPECT pose는 UDP 미전송.
- 이유: 수동 모드는 틀어진 pose를 lost로 판정하지 않아 화면에 큰 오차 pose가 계속 남음. 기존 YOLOE 스크립트는 validation/drift가 0으로 고정이라 auto를 켜도 감지 불가였음.
- 대안: 수동 유지. 위험: 0.5초마다 YOLOE 추가 추론(v8l이면 FPS 하락 가능), 임계값(0.3/5)은 실물 튜닝 필요.
- 작성자: Claude

## 2026-09-30 속도 shortcut 기본값을 원본(FoundationPose++)으로 복원 (Human 요청)
- 결정: iteration 기본값 10/5(원본), relock_iterations 기본 0(비활성, 항상 full register), roi_patience 기본 0(비활성, 항상 full-frame 검출). PoseTracker는 relock_iterations<=0이면 relock 및 Cutie tier-0 relock을 건너뜀. 옵션으로 다시 켤 수 있음(--relock_iterations 6 --roi_patience 3, --register_iter 5 --track_iter 2).
- 이유: 9/28에 속도 개선용으로 넣은 5/2, relock, ROI crop이 정확도를 해쳤을 가능성(실측 근거 없음). 원본 조건으로 되돌려 기준선 확보.
- 대안: 속도 우선 설정 유지. 
- 작성자: Claude

## 2026-09-30 YOLOE 스크립트 속도 조정: track_iter 3, score_interval 3 (Human 요청)
- 결정: real_time_v1_yolo.py 기본 track_iter 5->3(원본 5에서 완화), 신규 --score_interval 3(drift 검사용 scorer를 3프레임에 1회). PoseTracker.score_interval 추가(기본 1=기존 동작, drift_score_ratio<=0이면 score 계산 생략). 건너뛴 프레임은 직전 판정(last_score_ok)을 유지해 연속 실패 카운트가 초기화되지 않게 함. register_iter 10, relock/ROI off는 유지. DINO 스크립트 기본값은 변경 없음(track 5, score 매 프레임).
- 이유: 합성 프레임 실측 _track 전체 77ms(iter5, score 매프레임) -> 61ms(iter3) -> 51ms(iter3 + score 3프레임당 1회). 정확도 영향은 미측정; track_iter는 정확도와 직접 교환, score_interval은 drift 감지 지연(최대 N프레임) 증가.
- 대안: track_iter 5 유지 + score_interval만 적용(약 67ms 예상).
- 작성자: Claude

## 2026-09-30 real_time_project 파일 간소화 (Human 요청, 로직 변경 없음)
- 결정: real_time_v1_yolo.py -> main.py(주력). camera_rectify_kimm.py + d455_source_kimm.py -> real_time_utils.py로 통합. pose_tracker_kimm.py, recovery_tracker_kimm.py(핵심 로직)와 pose_sender_kimm.py(ros2_ws 테스트가 직접 import)는 유지. real_time_v1_dino.py는 import만 real_time_utils로 변경.
- 이유: 파일 수 축소. 옛 경로 `FoundationPose.real_time_project...` 폴백 import 제거.
- 작성자: Claude

## 2026-09-30 real_time_project 추가 통합 (Human 요청, 로직 변경 없음)
- 결정: pose_tracker_kimm.py(PoseTracker), recovery_tracker_kimm.py(RecoveryTracker)를 real_time_utils.py로 통합(703줄). README_real_time_v1_kimm.md는 루트 README.md 5장(동작 흐름/옵션)으로 요약 이전 후 삭제. pose_sender_kimm.py는 ros2_ws 테스트 import 때문에 유지.
- 검증: 합성 프레임으로 PoseTracker(Cutie+KF enabled) 등록/tracking 실행 확인(등록 2968ms, tracking 51.5ms, 통합 전과 동일). RecoveryTracker 상태 머신은 import/컴파일만 확인, 카메라 실행은 미검증.
- 작성자: Claude

## 2026-09-30 main.py 통합 (--detector yoloe|dino) (Human 요청)
- 결정: main_yoloe.py + real_time_v1_dino.py -> main.py 하나. 기본값은 (a) 통일: register 10, track 3, score_interval 3, validation 0.5, auto_recovery on, relock/ROI off. 검출기 전용 옵션은 argparse 그룹으로 분리, 선택한 검출기만 import. --est_refine_iter/--track_refine_iter는 별칭 유지. DINO 스크립트 대비 변경: publish_pose 옵션 제거(UDP 항상 송신), 기본 mesh는 textured_57mm.obj(필수 아님), DINO 기본 동작이 YOLOE 기본값으로 통일됨(track 5->3, 수동->자동 재탐지).
- 검증: parse_args/--help, YOLOE(v8l) 및 DINO+SAM2 detector 생성과 infer 확인. 카메라 실행 미검증.
- 작성자: Claude

## 2026-09-30 detector_comparison 삭제 및 detectors.py 통합 (Human 요청, 로직 변경 없음)
- 결정: real_time_project/detector_comparison/ 전체 삭제(benchmark.py, live_realsense.py, tests/, install_sam2.sh, requirements.txt, README.md, artifacts.py, yoloe-11s-seg.pt). base.py + pipeline_yoloe.py + pipeline_grounding_dino_sam2.py의 클래스만 real_time_project/detectors.py(400줄)로 통합(각 CLI 예제 main과 패키지 import 분기 제거). main.py 기본 yoloe_checkpoint를 루트 yoloe-v8l-seg.pt로 변경(11s는 이름만 주면 자동 다운로드).
- 이유: 파일 수 최소화. 벤치마크/라이브 비교 도구는 비교가 끝나 불필요하다는 Human 판단.
- 검증: 통합 후 YOLOE(v8l)/DINO+SAM2 생성, infer, detect_bbox 확인. 카메라 실행 미검증.
- 주의: 삭제한 파일은 git에 커밋된 것이면 git으로 복구 가능(미커밋 수정본인 pipeline_yoloe.py의 detect_bbox 추가는 detectors.py에 반영됨). SAM2 설치는 `pip install git+https://github.com/facebookresearch/sam2.git`.
- 작성자: Claude

## 2026-09-30 hand-eye 캘리브레이션 새 구현 (eye-to-hand, 카메라 고정)
- 결정: calibration/ 새로 작성(이전 코드는 Human이 삭제). 체인 T_cam_marker = inv(T_base_cam) @ T_base_flange @ T_flange_marker. OpenCV 5.0에는 cv2.calibrateHandEye가 없어 Park-Martin 닫힌해 + scipy least_squares(soft_l1) 정밀화를 직접 구현. 두산 posx는 [mm, deg, 내재 ZYZ]로 가정. 마커: DICT_4X4_50, ID 1, 25mm(변경 가능). flange pose는 get_current_tool_flange_posx(base, tcp=0)로 읽어 UDP(5006)로 conda 쪽에 전달(ROS2 Jazzy 시스템 python과 conda 3.11 분리).
- 이유: ROS2와 conda의 rclpy 충돌 회피. 합성 검증: 노이즈 0.3도/1mm에서 T_base_cam 오차 0.46mm/0.05도. 마커 크기별 검출 오차(0.5m, 합성): 25mm 평균 2~3.5mm(정면에 가까우면 회전 최대 15도), 50mm 약 1.4mm, 100mm 약 0.7mm -> 25mm는 정확도 한계, 큰 마커 권장.
- 미검증(위험): 두산 ZYZ 오일러 규약을 펜던트 표시값과 대조하지 않음. 실제 로봇/마커 데이터 없음. Human 검토 필요.
- 작성자: Claude

## 2026-09-30 캘리브레이션 자세 수집 자동화 (teach & replay, 저속)
- 결정: Human이 손으로 티칭한 관절 자세(waypoints.json, capture_handeye.py에서 마커가 검출될 때만 W로 저장)를 ros2_run_waypoints.py가 movej로 재생. 도착 시 UDP(5007/5008)로 capture_handeye.py --auto에 캡처를 요청하고 마지막에 자동 풀이. 속도는 기본 3 deg/s, 3 deg/s^2, 하드 상한 10. 기본은 dry-run, --execute + 'MOVE' 입력 필요, 관절 스텝 60도 초과/standby 아님이면 거부, Ctrl+C는 MoveStop(quick stop).
- 이유: 카메라 위치를 모르는 상태에서 마커가 항상 보이는 자세를 자동 생성하기 어렵고 충돌 위험이 있어, 사람이 안전한 자세를 한 번 티칭하는 방식이 안전. 
- 미검증(위험): 실제 로봇 동작 전부. amovej/check_motion 완료 판정, 서비스 이름(motion/move_stop), robot_state==1(STANDBY) 가정, 관절 경로 충돌 여부는 Human이 dry-run과 저속으로 확인해야 함. 핵심 안전 로직이라 Human 검토 요청.
- 작성자: Claude

## 2026-09-30 standalone rendered eye-to-hand + cube approach (Human 요청)
- 결정: 기존 stage-1 승인 대기를 현재 Human의 전체 구현 요청으로 대체했다. 기본 headless=False인 standalone에서 실제 RTX ArUco 영상→기존 solve_eye_to_hand→RGB-D 큐브 자세→base 변환→IK 관절 재생→GT 오차 계산을 자동 실행한다.
- 측정: flange에 DICT_4X4_50 ID 1(100 mm), cube 상면에 ID 2(45 mm). 작은 cube marker의 RGB-only 거리 오차(960×540에서 약 44 mm, 1920×1080에서 약 20 mm)를 해결하기 위해 이상적인 simulated depth 평면과 marker corner ray 교점을 rigid alignment한다. FoundationPose 추론 자체는 실행하지 않는다.
- 핵심 metric: T_a_b는 b→a, metres. translation은 두 pose 원점의 Euclidean 거리×1000, rotation은 R_gt.T @ R_est의 geodesic 각도(deg). 18 sample 중 매 네 번째 5개는 solver에서 제외하고 held-out residual을 계산한다. GT camera/cube는 평가에만 사용한다. depth 평면은 mean-centred SVD(초기 componentwise median이 평면 밖일 수 있음을 독립 수치 테스트로 찾아 수정).
- 종료/통과: 최대 36 자세, 목표 유효 관측 18개, 최소 12개. camera<10 mm/2°, cube<10 mm/2°, reach<1 mm, held-out 최대<5 mm/2°. 이 metric 정의와 임계값은 Human 검토 대상으로 기록하며 작업을 중단하는 승인 단계는 추가하지 않는다.
- 이동 범위: M1013 URDF FK/IK로 USD link를 관절 보간 재생한다. PhysX rigid body/joint를 비활성화한 운동학 검증이며 충돌/동역학/그리퍼 파지는 미검증이다. flange 목표는 측정 cube 중심에서 base Z +180 mm. 실제 ROS2/Doosan 연결·이동 없음.
- GUI: 결과 수치, D455/overview 전환, calibration+approach 재생 버튼. 측정 후 sensor render product 갱신 중지, idle sleep 25 ms. 출력과 사용법은 calibration/README_sim_eye_to_hand.md.
- 작성자: Codex

## 2026-09-30 저장된 eye-to-hand 행렬 재사용 object-approach 모드
- 결정: `--object-approach`는 기존 `calibration/data/eye_to_hand/result.json`의 `T_base_cam`만 읽고, flange ArUco를 장면에 추가하거나 hand-eye를 다시 풀지 않는다. cube 상면 ArUco ID 2와 simulated RGB-D에서 `T_cam_object`를 측정한 뒤 `T_base_object = T_base_cam @ T_cam_object`와 cube centre 위 base Z +180 mm의 flange 접근점을 계산·재생한다.
- 이유: hand-eye 단계와 object localization/motion 단계를 분리해 Human이 요구한 다음 단계를 독립적으로 확인할 수 있어야 한다. 결과는 기존 calibration 결과를 덮지 않고 `object_approach_result.json`에 저장한다.
- replay 수정: `context.save_as_stage()`가 in-memory URDF stage의 cached `Usd.Prim`을 만료시켜 replay callback이 `/m1013/base_link`에서 죽었다. USD 저장을 GUI loop 종료 후로 옮겼다.
- 작성자: Codex

## 2026-09-30 FoundationPose frame 점검 및 cube 대칭 분리 metric (Claude, Human 검토 요청)
- 점검 결과(frame): CAD `demo_data/077_rubiks_cube/google_16k/textured_57mm.obj`은 bbox centre `[0,0,0]`, extents 57 mm×3, 면이 축 정렬(각 축 area frac ≈0.28). Isaac `/World/RubiksCube`는 centre 원점, base축 정렬, 60 mm, 단색(displayColor)이다. 따라서 mesh-origin offset = 0 mm, frame 원점 정의는 동일(둘 다 cube centre). 남는 차이는 크기(half-size 1.5 mm)와 회전 라벨링뿐이다. (`textured.obj`는 centre `[-16.1,-0.5,28.7] mm`로 원점이 다르므로 쓰면 안 된다.)
- 결정: FoundationPose approach report에 `foundationpose_frame_check`를 추가. 회전 오차를 24개 cube 고유회전(signed permutation, det +1) 중 최근접 S와 residual로 분리(`R_gt.T @ R_est = residual @ S`). 위치 오차는 camera frame FP-only(`T_cam_object - inv(T_base_cam_GT) @ T_base_cube_GT`, ray 방향/횡방향)와 hand-eye-only(`T_base_cam_est @ T_cam_object_GT - GT`)로 분리.
- 결정(pass metric 변경): `passed`의 cube 회전 조건을 FP 모드에서만 raw가 아니라 symmetry-reduced 각도 < 2°로 판정. 새 필드 `position_approach_passed`는 회전을 제외한 기존 조건(calibration<10 mm/2°, cube 위치<10 mm, reach<1 mm, held-out)만 본다. 이유: Isaac cube가 단색이라 24-fold 회전은 관측으로 구분 불가하며, 현재 approach는 위치만 사용한다. ArUco 경로(`--object-approach`, full calibration)는 기존 판정 그대로.
- 유지: approach는 position-only(`cube 위치 + base Z 180 mm`, nominal safe flange orientation). FoundationPose 회전을 flange target에 적용하는 옵션은 만들지 않았다(별도 옵션·검증 전 적용 금지).
- Human 검토 요청: symmetry-reduced 회전을 pass에 쓰는 것이 타당한지, 실제 Rubik's cube(색 면 구분 가능)에서는 raw 회전을 써야 하는지.
- 작성자: Claude

## 2026-09-30 실제 Rubik CAD 렌더, cube +50 cm, 시작 자세 all-zero (Claude, Human 요청)
- 결정(cube 렌더): 단색 60 mm `UsdGeom.Cube` placeholder를 `demo_data/077_rubiks_cube/google_16k/textured_57mm.obj` 렌더로 교체(`add_rubiks_cube`). mtl은 이미지 텍스처 없이 7개 flat Kd 색이라 색별 `UsdGeom.Mesh` 7개(16384 face 전부, 누락/중복 0)로 만들고 translate만 적용. 크기 57 mm로 CAD와 정확히 일치(half-size mismatch 0 mm).
- 결정(GT 단일 출처): cube 중심/크기를 scene main에서 `args.cube_center_m`/`args.cube_size_m`로 넘겨 `sim_eye_to_hand.py`의 `cube_gt`/`cube_marker`가 하드코딩(.42,.10,.03 / .0305)이 아니라 같은 값을 쓰도록 변경. cube ArUco 높이 = size/2 + 0.5 mm.
- 결정(거리): D455 pose는 캘리브레이션 당시와 완전히 동일하게 고정(position `[0.72,-0.72,0.78]`, aim `[0.42,0.10,0.25]` 상수화). 이유: 카메라를 움직이면 저장된 `result.json`의 `T_base_cam`이 무효가 되고, 덮어쓰기는 금지. 대신 cube를 카메라 수평 방위를 따라 바닥 높이 유지하며 밀어 카메라 거리 1.152 → 1.652 m(+0.50 m), 위치 `[0.2145, 0.6616, 0.0285]`. 로봇 base 평면거리 0.70 m, IK 도달 확인.
  - 기각안 1: cube를 3D ray로 연장 → 바닥 아래로 내려가 불가. 기각안 2(1차 시도): 기존 IK nominal 대기자세에서 cube 이동 → flange가 시선을 가려 YOLOE가 flange를 cube로 검출(conf 0.025), FP 810 mm 오차. 기각안 3: 카메라를 0.5 m 뒤로 → T_base_cam 무효.
- 결정(시작 자세): 기본 시작/복귀 자세를 모든 joint 0°(FK: flange z=1.45 m, +Z 방향 = 하늘)로 변경. `--start-joints-deg J1..J6` 옵션, report `start_joints_deg`. IK seed는 기존 nominal IK 해(`q_nominal`)를 유지해 elbow-up branch 선택을 보존(0 자세 seed는 다른 branch로 수렴할 위험). 0 자세 팔은 camera-cube 시선에서 0.435 m 떨어져 가림 없음.
- 주의: 이 cube 이동은 full calibration 모드(result.json 작성)의 cube ArUco 위치에도 적용된다. full calibration을 다시 돌리면 result.json이 덮어써지므로 실행 전 Human 확인 필요.
- 작성자: Claude

## 2026-09-30 카메라 배치별 setup 분리, side 배치 재캘리브레이션, 관절 속도 제한 (Claude, Human 요청)
- 배경: Human이 Isaac GUI에서 cube와 D455를 직접 옮김(USD 저장본에서 읽음). cube `[0.64796, 0.01169, 0.0285]`, D455 `[1.13712, 0.07371, 0.37973]`, quat wxyz `(0.6527842, 0.3673936, 0.32492983, 0.57733464)`. 카메라가 옮겨졌으므로 기존 `T_base_cam`은 무효.
- 결정(setup 분리): `isaacsim_scene_m1013_d455_cube.py`에 `SETUPS`(v1 = 기존 카메라, `data/eye_to_hand`; side = Human 배치, `data/eye_to_hand_side`), `--setup`(기본 side). hand-eye 결과는 해당 카메라 pose에만 유효하므로 setup마다 data dir 분리. `foundationpose_sim_pose.py --setup`, runner는 `SETUP` env(기본 side). full calibration은 기존 result.json이 있으면 `--overwrite-calibration` 없이는 중단(v1 보호).
- 결정(캘리브레이션 중심): v1의 flange 중심 `[.48,-.05,.43]`은 side 카메라 화면 밖(v=-603 px)이라 setup별 `calib_flange_centre`로 분리. side = 마커가 렌즈 앞 0.5 m, 이미지 (960,300)에 오도록 `[0.6061, 0.1391, 0.1435]`(오프라인 검사: 36개 중 26개 화면 안, 링크 원점 최저 z 0.006 m). 샘플링 범위(±9–10 cm, ±25°)와 18 목표/12 최소는 동일, **최대 시도 36 → 60**(side는 거부율이 높음; v1은 18개에서 조기 종료하므로 결과 불변).
- 결정(속도): 운동학 재생을 프레임 수 고정(approach 90, calib 12, replay 25)에서 **관절 최대 속도 제한**으로 변경. `joint_path`가 가장 많이 도는 관절이 `--max-joint-speed-deg`(기본 30°/s)를 넘지 않게 dt=1/30 s 간격으로 보간, GUI에서는 벽시계로 속도 유지, headless는 대기 없음. replay도 벽시계 인덱싱. report에 `max_joint_speed_deg_s`.
- 기타: `--cube-xy X Y` 옵션(setup 기본값 덮어쓰기), runner는 추가 인자를 두 Isaac 실행에 전달.
- Human 검토 요청: side 카메라가 낮아(z 0.38 m) 캘리브레이션 자세가 바닥 근처(링크 원점 z≈0)까지 내려감. sim에서는 충돌 검사가 없지만 실기에서는 충돌 위험 → 실기 배치 시 카메라를 높이거나 캘리브레이션 볼륨을 테이블 위 15 cm 이상으로 두는 것을 권장. approach의 flange 방향은 여전히 "카메라를 향하는 nominal"이라 side 배치에서는 수평에 가까움; top-down(flange z = -Z) 접근 IK도 도달 가능 확인됨 — 별도 옵션으로 만들지 결정 필요.
- 작성자: Claude

## 2026-09-30 grasp-ready top-down 접근 자세 (Claude, Human 요청 "조인트 5번까지 그리퍼 있는 것처럼")
- 결정: 기본 접근 자세를 `--approach-orientation grasp`로 변경. j1–j5는 top-down 손목(j4≈0°, flange z = base −Z, IK seed `[atan2(y,x), 0, 90, 0, 90, 0]`), j6은 cube 옆면에 맞춤. cube yaw는 pose 추정(FP 또는 ArUco)의 가장 수직인 축을 뺀 나머지 축을 base XY에 투영해 mod 90°로 구하고, 90°마다 반복되는 후보 4개 중 IK 가능하고 j6 비틀림이 가장 작은 것을 선택. 이것은 FoundationPose 회전(yaw만)을 flange 목표에 적용하는 첫 번째 옵션이며, 90°/180° 대칭은 평행 그리퍼에 무관하므로 yaw mod 90만 사용.
- 옵션: `topdown`(yaw 고정 180°), `camera`(기존 카메라 향함 자세). `--tool-length-m`(기본 0): TCP를 flange +z로 tool 길이만큼 두고, pregrasp = TCP가 cube 중심 + 180 mm, grasp = TCP가 cube 중심. grasp 자세는 **IK 도달성만 확인**하고 재생은 pregrasp에서 멈춤(하강/파지 미실행 유지). `--cube-yaw-deg`로 scene cube를 회전시켜 검증 가능(GT도 같이 회전).
- report `grasp`: yaw 측정/GT/오차, tilt, flange yaw, q_pregrasp/q_grasp, pregrasp/grasp flange T.
- Human 확인 필요: 실제 그리퍼 TCP 길이와 finger 닫힘 축(flange x 또는 y). 현재는 flange x축을 cube 면 법선과 평행하게 둠. 닫힘 축이 y라도 90° 대칭이라 cube에는 동일하게 유효.
- 작성자: Claude

## 2026-09-30 pregrasp 높이 180 → 100 mm (Claude, Human 요청 "너무 멀다, 핸드로 할 것")
- 결정: `--pregrasp-height-m`(기본 0.10) 추가, TCP를 cube 중심 위 100 mm(cube 윗면 위 71.5 mm)에 정지. 0.0285 m(cube 윗면) 이하는 argparse에서 거부. tool 길이 0이면 flange 기준. GUI 문구도 높이 값을 표시.
- Human 확인 필요: 핸드 장착 시 TCP(손바닥/파지 중심) 길이를 `--tool-length-m`로 넣어야 flange가 아니라 핸드 파지점이 100 mm에 옴. 넣지 않으면 핸드가 cube와 겹칠 수 있음.
- 작성자: Claude

## 2026-10-01 실기 셀(테이블+알루미늄 프레임+D455 2대) sim 구성 및 voxel 충돌 검사 (Claude, Human 요청)
- 입력(Human 실측): 테이블 1.80×1.20 m, 프레임 높이 1.20 m, 카메라 바 양쪽 수직 프레임 간격 0.75 m, 그 바에 D455 2대(양쪽, 약 45° 기울임), 로봇–물체 1.00–1.10 m. 실기 D455 사진 `calibration/data/real_env/real_color_20261001_0917*.png`, intrinsics `real_intrinsics_*.json`(1280×720, fx 647.8, fy 646.9 → 89°×58°).
- 결정(형상, 단일 출처): `calibration/workcell.py` `FRAME_CELL` + `frame_layout()`이 테이블·프레임·D455 박스와 카메라 pose를 만들고, 같은 박스 목록으로 Isaac visual과 충돌 모델을 생성. **가정(실측 필요)**: 로봇 base는 테이블 짧은 변에서 30 cm·중앙선, 물체 x=1.05 m, 프레임은 물체 중심 0.75×0.75 m 정사각(깊이 미측정), 프로파일 40 mm, 카메라 바 = 로봇 반대편 윗 레일(1.20 m), 카메라는 기둥에서 6 cm 안쪽·바 아래 5 cm, pitch 45°·yaw는 물체 방향.
- 결정(카메라 광학): frame 셋업은 실기 D455 intrinsics로 렌더(1280×720, focal 1.88 mm, aperture 역산). 기존 v1/side는 원래 광학(1920×1080, 62° HFOV) 유지 — 저장된 캘리브레이션 유효성 보존.
- 결정(셋업): `frame_L`, `frame_R`(두 카메라 모두 렌더, setup이 측정 카메라 선택, data dir `eye_to_hand_frame_L/R` 분리). 캘리브레이션 중심은 오프라인 탐색(IK·충돌·시야)으로 `[0.882, ±0.0635, 0.6138]`.
- 결정(충돌): M1013 URDF collision mesh(.dae, mm→m) 표면 샘플 링크당 750점(캐시 `calibration/data/m1013_collision_points.npz`; Isaac Python에 pycollada가 없어 conda env에서 1회 생성, conda env에 `pycollada==0.9.3`를 `--no-deps`로 설치)을 FK로 변환, 1 cm voxel occupancy의 거리장(EDT)으로 clearance 계산. margin 20 mm. 캘리브레이션 중에는 ArUco 판(0.085 m, r 0.072 m)을, 접근 중에는 `--tool-length-m/--tool-radius-m` 원통을 툴로 포함. 이동은 직선 관절 보간이 충돌이면 bidirectional RRT(2° 간격 검사)+shortcut. 충돌 자세의 캘리브레이션 샘플은 거부. voxel/박스 목록은 `data/<setup>/workcell_voxels.npz/.json`로 export(실기에서 경로 사전 검사용).
- 결정(pass metric 변경, Human 검토 요청): frame 셋업에서 `collision_free`(재생된 접근 경로 최소 clearance ≥ margin)를 `position_approach_passed`에 포함. 다른 셋업은 영향 없음.
- 결정(캘리브레이션 샘플링, Human 검토 요청): setup별 `calib_samples/attempts/spread_m/spread_deg`. 기존 기본값(18, 60, ±9/10 cm·z −6..+9 cm, ±25°)은 난수 시퀀스까지 동일 재현(검증). frame 셋업은 30개, 최대 120회, ±15/15/12 cm, ±35°.
- 결정(속도): 기본 관절 속도 30 → 15°/s (Human 요청).
- 발견: 실기 FOV(89°)에서는 100 mm ArUco가 0.6–0.8 m 거리에서 84–115 px에 불과하고, hand-eye 오차가 카메라 광축 방향 스케일 편향(−6~−8 mm)으로 지배됨. 45° pitch에서는 물체가 이미지 아래쪽(v≈640/720). 카메라 바가 1.2 m 윗 레일이라면 물체를 화면 중앙에 두려면 pitch ≈ 68° 필요.
- 작성자: Claude

## 2026-10-01 움직임 부드럽게(경로+시간 배분), 충돌 여유 50 mm, voxel 시각화, 카메라 위치=Human cone (Claude, Human 요청)
- Human 지적: 로봇 속도 아직 빠름, 움직임이 불안, 실기에서는 프레임에 부딪힐 것 같음, "voxel이 맞나?", 카메라는 GUI의 cone 위치에 달 것.
- 원인 분석(코드로 확인): (1) 경로 계획기(RRT)가 무작위 중간 자세(j5 76°, j6 −29°)를 남기고, (2) 그 점들을 일정 속도 직선으로 이어 각 점에서 급정지·급출발, (3) GUI Replay가 현재 자세에서 시작 자세로 순간이동(teleport), (4) 충돌 여유가 20 mm로 작았음(최소 22 mm). 직선 경로는 near 상단 레일(x≈0.68, z≈1.19)에 닿아 막힘.
- 결정(경로): `workcell.CollisionChecker.tidy()` — 경유점을 이웃 점의 중점 쪽으로 당기되 양쪽 edge가 충돌 여유를 유지하는 만큼만 이동(elastic band). RRT+shortcut 뒤에 적용(`rrt+tidy`). 이후 PCHIP로 코너 없는 C1 경로(`trajectory.smooth_path`)를 만들고, 부드러운 경로가 여유의 90%를 침범하면 polyline을 세분(1/2/4/8)해 재시도, 마지막은 코너 유지.
- 결정(시간 배분): `trajectory.time_parameterize` — 구간별 곡률/관절 속도 한도, 가감속 한도(forward/backward pass), 0.4 s 이동평균(저크 제한), 시작·끝 속도 0. 이전: 구간마다 등속+급정지. 측정(5 waypoint 경로): peak a 171 → 10.8°/s², peak jerk 5143 → 낮음, 시작 첫 프레임 속도 6.5 → 0.1°/s.
- 기본값: 관절 최대 속도 15 → **8°/s**, 가속 한도 **10°/s²**(`--max-joint-speed-deg`, `--max-joint-accel-deg`). GUI에 속도 슬라이더(1–40°/s, Replay에 적용). Replay는 현재 자세에서 시작 자세로 이어서 이동한 뒤 재생(순간이동 제거).
- 결정(충돌 여유): 기본 margin 20 → **50 mm**(`FRAME_CELL['collision_margin_m']`). 계획 경로 최소 clearance 50.0 mm 확인. 샘플 간 이산화로 실제 최소는 ≈36 mm까지 내려갈 수 있어 테스트는 0.6×margin 기준. 툴 길이 0이면 핸드는 충돌 모델에 없음 → 핸드 장착 시 `--tool-length-m/--tool-radius-m` 필수.
- 결정(voxel 표시): 이전 GUI의 흰 프레임은 렌더용 박스이고 voxel(거리장)은 눈에 보이지 않는 내부 자료였음. 이제 GUI에서 점유 voxel 표면층(1 cm)을 빨간 반투명 큐브로 그림(`--show-voxels`, 버튼으로 토글). 센서 촬영이 끝난 뒤에 생성하므로 RGB-D/YOLOE/FoundationPose 입력에 섞이지 않음.
- 결정(카메라 위치): Human이 GUI에 놓은 `Cone_01 (1.0722, 0.5706, 1.1654)`, `Cone_02 (1.0688, −0.1324, 1.1043)`을 `frame_L`, `frame_R` 카메라 위치로 사용(`FRAME_CELL['camera_positions']`). 방향은 큐브를 바라보게(`camera_aim='object'`): pitch L 63.3°, R 82.9°(수평 아래). Human이 말한 "45°"를 유지하면 R은 큐브가 시야 밖(위치에서 38° 벗어남)이므로 적용하지 않음 — Human 확인 필요. 새 위치마다 캘리브레이션 중심 재탐색 필요 → `find_calib_centre.py` 추가.
- 가정 주의: 프레임 기둥/레일 배치는 여전히 내 가정(0.75 m 정사각, 로봇 base 30 cm 안쪽)이며 cone 위치와 맞지 않음(L cone은 가정 프레임 바깥). 실제 프레임 배치 확인 필요.
- 작성자: Claude

## 2026-10-01 (후속) 계획 여유, GUI 수정, 150 mm 마커 (Claude)
- 충돌: 검증 margin 50 mm는 유지하되 계획은 1.3배(65 mm)로 여유를 두고(edge 끝점은 50 mm), 회전만 하는 `link_1`(테이블과 60 mm 고정)은 검사에서 제외(`STATIC_LINKS`). 결과: 접근 경로 최소 clearance 42.4 → **63.2 mm**, 코너 없이 부드러운 경로 유지.
- GUI: 패널을 ScrollingFrame으로 바꾸고 버튼(D455 image, overview, 속도 슬라이더, Replay, voxel 토글)을 맨 위로(슬라이더 추가 후 Replay 버튼이 창 아래로 밀려 안 보였음). voxel 표시에서 D455 렌즈 주변 12 cm는 그리지 않음(카메라 시점이 자기 몸체 voxel에 갇혀 검게 보였음; 충돌 검사에는 포함).
- 카메라 방향: Human 선택 = 큐브 중심을 바라보기 유지(L 63.3°, R 82.9°). 프레임 형상: Human 선택 = 현재 가정(0.75 m 정사각) 유지 — cone L은 가정 프레임 밖이라 실기 충돌 모델로는 부정확, 실측 필요.
- 캘리브레이션 마커: frame 셋업 기본 150 mm(`flange_marker_m`, `--flange-marker-m`). cone 위치에서 100 mm → 150 mm: L 16.49 → **9.42 mm**(0.319 → 0.135°), R 7.42 → **4.58 mm**(0.309 → 0.067°). 여전히 카메라 광축 방향 편향이 대부분. frame 셋업은 cube ArUco(45 mm)가 안 보여도 hand-eye 결과만 저장하도록 함(`mode: hand-eye calibration only`).
- 작성자: Claude

## 2026-10-01 (Codex: 실기 flange pose 자동 수집 경로)
- 결정: 수동으로 펜던트 값을 입력하는 대신, 사람이 자세를 바꾼 뒤 `ros2_flange_udp.py`가 Doosan ROS2의 `get_current_tool_flange_posx(DR_BASE)`와 `get_current_posj()`를 읽어 localhost UDP 5006으로 보낸다. `capture_handeye.py --robot udp`는 C 키 시 최신 pose를 자동으로 샘플에 기록한다. 두 스크립트 모두 이동/jog/waypoint 실행을 하지 않는다.
- 호환: 설치된 `DSR_ROBOT2.py`가 현재 `dsr_msgs2`에 없는 선택 서비스 `SetSingularityHandlingForce`를 import 단계에서 생성해 `NameError`가 났다. 해당 선택 서비스 client만 조건부 생성하게 source 및 install 복사본을 수정했다. pose 조회 API에는 영향이 없다. `py_compile` 통과. Doosan workspace를 재빌드하면 install 복사본이 source 변경으로 다시 생성된다.
- 환경: ROS Jazzy는 시스템 `/usr/bin/python3`으로 실행해야 한다. Miniconda Python이 PATH를 우선하면 ROS Python ABI/패키지 오류가 날 수 있다.
- 작성자: Codex

## 2026-10-01 (Codex: Claude용 실기 hand-eye 인수인계)
- Human은 펜던트 수동입력으로 31개 marker/flange 샘플을 수집했다. solve residual **184.08 mm mean / 584.46 mm max**, **22.30° mean / 108.18° max**이므로 산출 transform은 사용하지 않는다. 샘플은 보존하고 우선 좌표계/Euler/TCP/마커 치수 가정을 오프라인 분석한다.
- 후속 handoff는 `collab/HANDOFF_REAL_HANDEYE_2026-10-01.md`. Human은 PC와 Doosan controller가 IP socket으로 연결되어 있다고 알렸으나 pose state API/protocol은 확인되지 않음. 추정 protocol을 만들지 말고 read-only 지원 API를 조사한다.
- 결과 파일 보호: `data/eye_to_hand/result.json`, `T_base_cam.txt` 금지된 overwrite 유지. 실패한 fresh solve는 `data/T_base_cam.json`에 별도 저장되어 있으며 실사용 금지.
- 작성자: Codex

## 2026-10-01 (Codex: 31개 수동 샘플 재검사)
- 관찰: 저장된 all-sample 해의 residual이 높고 #0–13 vs #14–30 상대 회전 일관성이 크게 달라진다. 각 flange relative rotation `R_fiᵀ R_fj`와 marker-camera relative rotation `R_miᵀ R_mj`의 geodesic angle은 고정 eye-to-hand/rigid mount에서 동일해야 하므로 이 검사는 `T_base_cam` 추정치와 무관한 데이터 정합성 진단이다.
- #0–13 pair median angle mismatch 0.98° (92.3% <5°); #14–30 median 20.80° (15.4% <5°). 따라서 문제를 단순히 “몇 개의 isolated outlier”로 간주해 삭제하지 않는다. #14 이후 구간의 pose pairing/convention/rigidity 변화를 확인한다.
- 결정: 31개 원본 보존. 신규 transform 생성이나 use 권장하지 않음. pendant field/frame/TCP semantics를 확인하기 전에 solver/loss/threshold를 변경하지 않는다.
- 작성자: Codex

## 2026-10-01 실기 hand-eye: 실패 원인 = pose 짝 어긋남, 자동 캡처로 전환 (Claude)
- 결정: 31샘플 solve 실패(184 mm)의 원인은 수기 입력 pose와 영상의 인덱스 어긋남으로 판단(LOG 참조). 기존 샘플은 보존하고 재짝지은 해는 탐색용으로만 기록, 사용하지 않음. 재수집 권장.
- 결정: 실기 수집은 `auto_handeye_capture.py`로 — 로봇은 Human이 움직이고, pose는 ROS2 `get_current_tool_flange_posx(DR_BASE)` read-only. 로봇 이동 명령 경로(`ros2_run_waypoints.py`, `capture_handeye.py --auto`)는 사용하지 않음.
- 새 캡처 게이트(Human 검토 요청, 진단용 기본값): 정지 1.5 s(0.3 mm/0.1°), 저장 샘플과 ≥40 mm 또는 ≥10° 차이, 회전각 짝 검사 중앙값 ≤8°(저장 3개 이상), 저장 샘플 해로 예측한 marker 위치 오차 ≤25 mm(저장 6개 이상), 기존 tilt ≥15°·jitter ≤1.5 mm 유지. 리포트에 leave-one-out 추가.
- 리스크: 25 mm 예측 게이트는 초기 샘플이 나쁘면 이후 좋은 샘플을 거부할 수 있음(U로 되돌리기/세션 새로 시작). 100 mm marker는 0.8–1.4 m에서 회전 잡음 ~2°라 150 mm 이상 권장(sim 결과와 같은 결론).
- 작성자: Claude

## 2026-10-01 real_cam 시뮬, 복귀 경로, 펜던트 경유점, link_2 제외 (Claude)
- 결정(Human 요청 반영): 실기 hand-eye `T_base_cam`(data/real_handeye/20261001_160439)의 위치·방향으로 단일 카메라 setup `real_cam`(data_dir `eye_to_hand_real_cam`) 추가. 프레임은 카메라와 바깥 면이 맞도록 x로 +7 cm 이동(해석 — Human 확인 필요). 검증 margin 0.10 m(`cell_overrides`), 환경변수 `CELL_MARGIN_M`로 덮어쓰기(캘리브레이션 자세는 0.10에서 불가해 0.05로 실행). sim 캘리브레이션 9.18 mm / 0.245°.
- 결정(안전 검사 변경, Human 검토 요청): `STATIC_LINKS`에 `link_2` 추가. 근거: 영점·접근 자세 모두 어깨 하우징이 테이블과 80 mm로 일정하고 프레임에서 0.7 m 이상 떨어져 있어, margin 0.10 m는 로봇 자체 구조로 충족 불가였음. 영향: 모든 frame setup의 clearance 수치가 달라질 수 있음(link_2 제외). unittest 8/8.
- 결정: 접근 후 시작 자세로의 **복귀 경로를 별도 계획·검증**(`return_min_clearance_m`), `collision_free`에 포함(Human 검토 요청). 펜던트 경유점 표(`pendant_waypoints.csv`)와 GUI 재생 버튼 추가 — 경유점 사이는 관절 공간 직선.
- 기본 관절 속도 3°/s, 가속 3°/s²(Human: 항상 가장 느리게).
- 미해결: real_cam에서 큐브가 보이지 않음 — 카메라가 프레임 기둥 바로 옆이라 시야 왼쪽 절반이 기둥에 가려지고 큐브(1.05, 0)가 시야 가장자리(축에서 ~41°)임. FoundationPose가 엉뚱한 pose를 반환해 접근 결과 무효(큐브 오차 1077 mm). 실제 프레임/큐브 위치 실측 필요.
- 작성자: Claude

## 2026-10-01 6D pose → 로봇 ROS2 파이프라인: 드라이런까지 (Claude, Human 선택)
- Human 질문: "6D pose 좌표값을 로봇에 ROS2로 보내는 파이프라인". 초기 규칙(실기 이동/ROS2 명령 금지)이 있어 범위를 확인 → Human이 **드라이런까지** 선택(이동 명령은 만들지 않음).
- 구현: `calibration/real_pose_dryrun.py`(conda) — `real_time_project/main.py`가 UDP 5005로 보내는 `T_cam_object` 수신 → `T_base_object = T_base_cam @ T_cam_object`(실기 hand-eye 결과 기본) → 안정성 판정(8개 연속 3 mm/3°) → 상식 검사(거리·높이·기울기) → top-down 접근 자세 IK → work-cell 충돌 검사(margin 0.10 m, 카메라 몸체 박스 포함, 실측 `--cell_json`) → RRT 직선 경유점 표(접근+복귀) → `data/real_dryrun/<time>/plan_NNN.{json,csv}` 저장, UDP 5010 전달. `calibration/ros2_dryrun_publisher.py`(시스템 python) — `/sixd/dryrun/{target_pose,target_joints_deg,waypoints_deg,status}`만 발행. **로봇 제어 토픽/서비스/`/dsr01`은 사용하지 않음.** 현재 관절은 `ros2_flange_udp.py`(UDP 5006, 읽기 전용)로 받아 시작 자세로 사용, 없으면 all-zero.
- 검증: 합성 pose(큐브 (1.05, 0.10) 기준)로 end-to-end 시험 — ACCEPTED, 접근/복귀 직선 최소 clearance 125 mm, ROS2 `/sixd/dryrun/status`와 `target_joints_deg` 수신 확인. 실제 D455/FoundationPose/로봇은 연결하지 않음.
- 리스크/가정: 프레임·기둥 위치는 아직 가정(`data/real_cell_template.json`에 실측 기입 필요). `T_base_cam`은 줄자 대조 전. 실기 depth/FoundationPose 정확도 미검증. 관절 한계 검사는 아직 없음(경유점 표를 사람이 확인).
- 작성자: Claude

## 2026-10-01 펜던트 경유점 단순화 (Claude, Human 요청)
- Human 요청: 경로를 point-to-point 몇 구간으로 나눠 안전하게. 결정: `CollisionChecker.simplify` — 플래너 경유점(13~17개)을 그리디로 줄여, 각 구간이 직선 관절 이동으로 **margin×1.15(115 mm) 이상** 유지하고 **한 관절 최대 45° 이하**가 되게 함(건너뛸 수 없는 점은 유지). 적용: `real_pose_dryrun.py`(`--max_segment_deg`), sim 펜던트 표/재생 버튼.
- 측정(큐브 (1.05, 0.10), 시작 영점): 구간 수 vs 직선 최소 clearance — margin 100 mm 요구: 6/5 moves 103 mm(45°/…); 115 mm 요구: 접근 6 moves 117 mm, 복귀 5 moves 125 mm. 3 moves(60°+)는 108 mm로 여유가 줄고 구간 하나가 57° 이상 돌아 육안 확인이 어려움 → 채택하지 않음. 이 기준(45°, 1.15배)은 내가 정한 값이며 Human 검토 요청.
- 작성자: Claude

## 2026-10-01 규칙 부분 해제: 감독형 실기 point-to-point 이동 스크립트 (Claude, Human 선택)
- Human이 "point-to-point로 무빙 실행, 무조건 가장 느리게"를 요청, 실행 대상 질문에 **실제 로봇(규칙 해제 필요)**을 선택 → 초기 규칙(실기 이동/ROS2 명령 금지)을 이 범위에 한해 해제한 것으로 기록. 범위: 감독형·최저속·한 점씩 확인 이동. 그 외 자동/연속 이동은 여전히 금지.
- 구현: `calibration/ros2_pendant_mover.py`(시스템 python) — 드라이런 plan의 한 구간(approach/return)을 `MoveJoint`로 **경유점마다 사람이 `go`를 입력해야만** 이동. `--execute` 없으면 검사만(ROS 불필요). 안전장치: 속도·가속 기본 3, 하드 한도 5 deg/s; plan accepted·최신(기본 30분)·구간당 한 관절 ≤45°·URDF 관절 한계 2° 여유·현재 관절이 직전 경유점과 ≤3°·도착 오차 ≤1°; 제어권 거부 시 중단(재시도 없음); Ctrl+C = quick stop 요청; 시작 선언문 입력. **어시스턴트는 이 스크립트를 로봇에 대해 실행하지 않음(사람이 실행).**
- 검증: 로봇 연결 없이 검사 모드만 시험 — 정상 plan 승인 표(접근 6 moves) 출력, 속도 10 deg/s·미승인 plan·누락 구간 거부 확인. 실제 이동은 시험하지 않음. 메모: 시작 자세는 plan 기준(영점)과 현재 관절이 맞아야 함.
- 리스크(실행 전 Human 확인 필요): (1) 큐브 pose가 실행 중 크게 변동(17:29–17:32 사이 x 1.08→0.92, y 0.02→0.25, yaw −9→−26°) — 추적 불안정 또는 큐브 이동. 줄자 실측과 대조 전 실행 금지. (2) 프레임·기둥은 가정값, 실측 필요. (3) 큐브 높이 z 0.01 m(예상 0.0285). (4) 제어권(`Access control deny`)은 펜던트 쪽 — Human이 이전해야 함. (5) 비상정지 손 닿는 곳, 주변 인원/물체 확인.
- 작성자: Claude

## 2026-10-01 높이 보정 변환을 기본값으로, 이동 속도 2 deg/s (Claude, Human 지시)
- Human 지시: 보정변환(높이 보정)을 기본값으로 저장. `real_pose_dryrun.latest_t_base_cam()`이 최신 세션의 `T_base_cam_zfix.json`(기본 해에서 카메라 z만 줄자값 0.115 m로 교체; 원본 `T_base_cam.json`은 보존)을 우선 사용. 근거: 줄자 렌즈 높이 11.5 cm vs 계산 9.2 cm, 큐브 중심 z 예측 7 mm vs 예상 28.5 mm가 같은 방향·크기로 어긋남 → zfix로 큐브 z 30.4 mm. 단 줄자 한 값(±5–10 mm) 의존, 수평 오차는 눈 맞춤 한계로 1–2 cm 미해결.
- Human 지시 "속도 무조건 가장 느리게" → `ros2_pendant_mover.py` 기본 속도/가속 3 → **2 deg/s, 2 deg/s²**(하드 한도 5 유지).
- 새 plan 시도(큐브 `[0.921, 0.305, 0.030]`): 가정 프레임과 0 mm로 **거부**(실제 프레임 미실측). 큐브를 중앙(y≈0.1)으로 옮기거나 프레임 실측 필요.
- 작성자: Claude

## 2026-10-01 프레임 실측 반영 (Claude, Human 실측)
- Human 실측(로봇 base 중심 기준): 기둥 x = 0.85 m(가까운 쪽), 1.65 m(먼 쪽), y = ±0.56 m(좌우 대칭, Human 확인). `data/real_cell_measured.json` — `frame_centre_xy [1.25, 0]`, `frame_size [0.84, 1.16]`(기둥 중심선 간격 + 프로파일 40 mm), 높이 1.20 m, 테이블 두께 0.10 m. `workcell.frame_layout`에 `frame_centre_xy` 옵션 추가. **가정 유지**: 프로파일 40 mm, 로봇-테이블 가장자리 거리 0.30 m(원 가정; 먼 쪽 기둥 x=1.65 m는 이 테이블 모델 가장자리 1.50 m 밖), 레일 배치.
- 결과: `real_pose_dryrun.py --cell_json data/real_cell_measured.json` — 큐브 `[1.0406, 0.0474, 0.0289]`(높이 보정으로 z 28.9 mm), 접근 3 moves/복귀 3 moves, 직선 구간 최소 clearance 120 mm, ACCEPTED. plan `calibration/data/real_dryrun/20261001_195225/plan_001.json`. 로봇 현재 관절 영점(읽기 전용 확인). `ros2_pendant_mover.py` 검사 모드 접근/복귀 모두 CHECK OK(속도 2 deg/s). 이동 명령 미실행.
- 작성자: Claude

## 2026-10-01 calibration/ 정리 (Claude, Human 요청)
- 요청: 핵심(ROS 통신, YOLOE pose, 동작 명령)만 남기고 sim/테스트 자료 정리. 결정: **삭제하지 않고** `~/pan/6D_pose_archive/calibration_2026-10-01/`(저장소 밖)로 이동. 보호 파일 sha256(`eye_to_hand/result.json` 847d6d32…, `T_base_cam.txt` c8b0ff63…, `handeye_samples.json` 3fa7e471…) 이동 후 동일 확인.
- 유지: 실기 파이프라인 18개 파일(인식 `real_time_project/main.py` + `calibration/`: hand-eye 수집·검증, ROS2 읽기/발행/감독형 이동, 드라이런 계획, 기구학, 작업 셀, 궤적, `test_core.py`), 데이터 `real_handeye/`(2 세션), `real_cell_*.json`, `real_d455_intrinsics.json`, 충돌 샘플 캐시, 성공 계획 1개.
- 이동: Isaac scene/sim 파이프라인·FoundationPose sim·시험, 수기 입력 캡처(`capture_handeye.py`)와 **자동 이동 스크립트 `ros2_run_waypoints.py`**, sim 캘리브레이션/USD(약 360 MB), 실패 샘플/분석, 사진, 이전 계획.
- 코드 변경: 순수 기구학을 `kinematics.py`로 분리(`Kinematics`, `cube_yaw_deg`, `grasp_frames`, `DEFAULT_URDF`), `workcell.py`에서 Isaac/카메라 배치 코드 제거(`frame_layout`은 박스 목록만 반환), `FRAME_CELL` 기본값을 실측값으로(여유 0.10 m). 시험 `test_core.py` 12개 통과(영점 FK=실기 flange, IK 왕복, top-down 자세, hand-eye 복원, 실측 프레임 배치, 계획 수락/거부, mover 거부 규칙·기본 속도 ≤2 deg/s, 궤적 한계). 각 스크립트 `--help`/구문 확인. 커밋/푸시는 하지 않음(Human 확인 후).
- 작성자: Claude

## 2026-10-01 정정: 높이 보정 변환(zfix)의 캘리브레이션 오차 (Claude)
- Human 질문(캘리브레이션 오차)에 답하며 3개 변환을 같은 25샘플에 평가: 기본 `T_base_cam` pos 평균 **5.41 mm**(최대 8.41)/rot 1.28°(3.00), leave-one-out 5.88/9.27 mm; marker-scale `T_base_cam_markerscale` **3.12 mm**(5.62)/1.26°; **`T_base_cam_zfix`(현재 기본값) 23.30 mm**(27.20)/1.28° — 평균 dz +22.7 mm. 즉 zfix는 marker 데이터(로봇 flange 기준)와 z로 23 mm 불일치하는 변환이며, 이를 알리지 않고 기본값으로 권고·적용한 것은 내 누락. 근거가 서로 충돌: (a) marker 데이터는 기본/보정본(z 9.2/10.3 cm) 지지, (b) 줄자 렌즈 높이 11.5 cm와 큐브 z 예측 7 mm(예상 28.5)는 +12~23 mm 지지.
- 조치: Human에게 보고하고 기본값 선택을 요청(기본 `T_base_cam` / marker-scale / zfix 유지). 선택 전까지 파일은 보존, 이동 계획에 쓰이는 값은 zfix(실행 시험 완료)임을 명시.
- 작성자: Claude

## 2026-10-02 두 터미널 통합 파이프라인 (Codex, Human 요청)
- 요청: ROS 연결 1개 + 인식부터 경로·ROS 전달·이동까지 실행 1개로 단순화. 이번 구현 중 로봇 이동 금지.
- 구현: `run_robot_connection.sh`(ROS 환경+bringup), `run_cube_pipeline.sh`(conda 통합 실행), `cube_pipeline.py`(인식/상태/발행 자식 관리 + 기존 계획 함수 + 감독형 mover). 기본은 이동 없는 계획/발행, Human의 `--execute` 실행에서만 기존 선언문·경유점별 `go`를 유지. 접근 후 `return` 선택 시 복귀 승인; 자동 복귀·연속 자동 이동은 만들지 않음.
- 경계: 현재 관절 1초 이내 수신 필수, 영점 fallback 금지. 신선한 카메라 pose·기존 8개/3 mm/3° 안정성·양방향 충돌/관절 검사 후 계획 고정. 프로세스 실패는 중단·정지 요청. 오프라인 결과에는 실제 이동 금지 표시. `go` 입력 후 현재 관절과 plan 나이를 재검사.
- 변환 선택/캘리브레이션 데이터는 기존 동작 유지. zfix의 마커 잔차 평균 23.3 mm·큐브 GT 미검증을 시작 화면/README에 명시. GT 확보는 이번 변경 범위 밖.
- README 두 개를 실행 명령·출력·승인·복귀·종료·진단 순으로 재작성. 실제 로봇 이동/서비스 호출 없이 하드웨어 없는 검사만 수행.
- 작성자: Codex

## 2026-10-02 기본 T_base_cam을 markerscale + xy 상수 오프셋으로 변경 (Human 결정, Claude 구현)
- 근거: 큐브 5곳(`calibration/data/gt_compare/20261002_112500/positions.json`)에서 wrist ArUco GT 대비 FoundationPose×T_base_cam 수평 오차 평균 — 기본/zfix 19.4 mm, markerscale 6.9 mm. 오차가 위치와 거의 무관한 상수 편향(산포 ~1 mm) → markerscale 변환에 base 기준 x +4.56, y +5.07 mm를 더한 `T_base_cam_markerscale_xyoffset.json` 생성(원본 파일 보존). leave-one-out 수평 오차 평균 2.1 mm(최대 2.8).
- **z는 보정하지 않음**: GT 큐브 중심 z가 약 15 mm로 나옴(테이블 위면 28.5 mm여야 함). 마커 실측 길이·base z=0 위치 미확인이라 GT z가 신뢰 불가. 회전은 법선 1.5–4°, yaw ±3° 수준으로 비교만 했고 보정 안 함.
- 코드: `real_pose_dryrun.latest_t_base_cam()`이 `T_base_cam_markerscale_xyoffset.json` → `T_base_cam_zfix.json` → `T_base_cam.json` 순으로 선택. 카메라를 옮기면 재캘리브레이션과 재보정 필요. 단위시험 20개 통과.
- 작성자: Claude

## 2026-10-02 작업 셀 테이블 높이: 베이스 플레이트 20 mm 반영 (Human 측정, Claude 구현)
- Human 측정: 로봇 베이스와 테이블 사이에 철판(플레이트) 20 mm. base 좌표 z=0은 베이스 바닥면이므로 테이블 상면은 z=−0.020 m. 근거 일치: GT 마커 윗면 z로 역산한 테이블 면 −14~−18 mm, 줄자 렌즈 높이 11.5 cm − 2 cm ≈ 9.5 cm vs 기본 변환 카메라 z 9.2/10.3 cm. FoundationPose z 자체는 로봇 좌표계에서 일관(FP−GT dz +0.9 mm)이라 변환은 수정하지 않음.
- 코드: `workcell.py`에 `base_plate_thickness`(기본 0.020) 추가. 테이블 상면과 그 위에 선 프레임(기둥·레일)을 모두 z=−0.020 기준으로 배치(기둥 아래 −0.02, 프레임 높이 1.20 m는 테이블 면 기준 가정). `data/real_cell_measured.json`에 같은 키 추가. 단위시험 20개 통과.
- 영향: 충돌 모델이 이전(테이블 z=0)보다 20 mm 낮아 테이블 근처 여유가 커짐(실제에 맞춤). 프레임 높이가 테이블 면 기준인지는 가정. 큐브 중심 예상 z는 −0.020+0.0285 = 8.5 mm이고 FP(markerscale)는 약 15 mm로 남은 차이 약 6 mm는 GT/배율 불확실성 범위.
- 작성자: Claude

## 2026-10-02 pick-and-place RL 환경 v1 (Claude, Human 검토 요청)
- 결정(알고리즘): Isaac Lab + rsl_rl PPO, 기본 하이퍼파라미터는 Franka Lift와 동일(`rl/pick_place_agent.py`). 상태 기반 관측(관절, TCP, 접시 위치+5 mm 균일 노이즈, 목표 위치, 직전 액션). 카메라/FP는 sim에서 쓰지 않음(GT 위치 + 노이즈로 FP 오차 대체).
- 결정(장면): 접시 상자 (1.0, 0.33), 목표 상자 (1.0, -0.17)(Human: 접시와 상자 위치 맞교환 후 y 0.5 m 차). 상자 높이 53.5 mm, 상자 가로세로 80 mm는 임시. 접시는 스캔 메시가 아니라 원기둥(r 0.133, h 0.025, 0.3 kg, 마찰 1.0) 근사 — 질량·마찰은 가정. 커플링 20 mm는 Human 확인.
- 결정(시작 자세): 에피소드를 홈 자세가 아니라 접시 테두리 위 사전파지 자세(IK, TCP 0.10 m 위, `Q_PREGRASP_DEG`)에서 시작 — 실제 파이프라인의 FP + ROS 접근이 그 자세까지 데려온다는 가정. 관절 ±0.03 rad, 접시 xy ±2 cm, yaw 무작위.
- 결정(보상, 1차 초안): 테두리 근접(std 0.10/0.02 m, 가중치 1/2), 툴 하향 정렬(0.5), 접시 들어올림(+3 cm AND TCP가 테두리 4 cm 이내일 때만, 15), 들린 상태에서 목표 xy 근접(std 0.3/0.05, 16/5), 목표 상자 위 안착(xy 2.5 cm, z 1 cm, 속도 <0.05 m/s, 50, 성공 시 종료), action_rate/joint_vel 벌점(1만 스텝 후 -0.1). 첫 시험에서 "들림 AND 테두리 근처" 조건이 없으면 접시를 쳐서 날리는 편법이 보상됨을 확인해 조건 추가.
- 한계/미검증: 학습 성공 여부 미확인(5 iteration 스모크만). 4절 링크는 단순화(손가락 2개 prismatic). 5초 유지 조건은 안착 성공 조건으로 대체. 접시 파지가 물리적으로 가능한지(테두리 두께, 패드 높이) 미검증. 보상 가중치와 임계값은 내가 정한 값이며 Human 검토 대상.
- 작성자: Claude

## 2026-10-02 RL 환경에 철제 프레임 충돌 추가 (Claude, Human 지적으로 수정)
- 문제: pick-and-place 환경 v1에 철제 프레임이 없었음(Human 지적). 수정: `calibration/workcell.frame_layout`의 실측 기둥 4개 + 레일 4개(+ `real_cell_measured.json`)를 각 env에 정적 충돌체로 추가(`rl/pick_place_env.py`).
- 충돌 처리: 팔 link_1~6과 gripper_base(손가락 제외)에 접촉 센서. 순수 net force > 1 N이면 보상 -20/스텝 + 에피소드 종료. 필터형 센서는 PhysX가 "센서 body 수 = 필터 prim 수"를 요구해 정적 프레임 8개에 쓸 수 없어 비필터 net force 사용(자기충돌 꺼져 있으므로 접촉은 프레임/테이블/상자/접시). 부작용: 팔 몸통이 접시·상자에 닿아도 종료됨(의도적으로 보수적).
- 검증(`rl/check_contacts.py`): 사전파지 자세 0 N, 팔을 테이블에 박으면 742 N, 그리퍼를 기둥 (0.85, 0.56, 0.35)에 대면 약 4만 N, 모두 충돌 플래그 1.
- 미반영: 프레임 단면 40 mm와 레일 높이는 기존 가정값 그대로, 카메라 몸체 박스(D455)는 RL 환경에 아직 없음.
- 작성자: Claude

## 2026-10-03 RL 환경 v2: 실제 접시 메시, IK 액션, 홈 자세 시작 (Claude, Human 지시)
- 접시: 원기둥 근사 폐기. `demo_data/029_plate/tsdf/textured.obj`(266×268×25 mm, 바닥 평면 + 가장자리 r 0.12~0.133 m에서 25 mm로 상승하는 접시형)를 open3d로 3000 삼각형 단순화(`rl/assets/plate029_simplified.obj`, 원점=바운딩박스 바닥 중심) → `rl/make_plate_usd.py`로 USD 변환(질량 0.3 kg 가정, 볼록 분해 충돌 16개로 접시 안쪽이 메워지지 않게 함). 마찰은 접시·손가락 모두 1.0(가정, 시작 이벤트로 설정). 질량·마찰은 실측 아님. 원본 메시는 이음새가 열린(non-watertight) 표면이라 볼록 분해 품질은 미검증.
- 시작 자세: 사전파지 자세 시작은 내(Claude) 임의 결정이었고 Human이 홈 관절값(-5.98, -14.75, 117.74, -3.92, 7.46, -85.07)에서 시작하라고 정정 → 홈 자세 시작으로 되돌림(관절 ±0.03 rad 노이즈). TCP→테두리 거리 약 0.63 m로 어려워짐. 에피소드 길이 12→15 s.
- 액션: 관절 델타 대신 TCP 포즈 상대 명령(차분 IK, DLS), [dx,dy,dz,droll,dpitch,dyaw]×0.04(m, rad)/스텝(Human 제안). 상대 IK가 중력 처짐을 스텝마다 누적해 영 액션에서도 TCP가 10 cm 흘러서(측정) 시뮬 로봇 링크의 중력을 끔(`disable_gravity=True`; 실제 로봇은 컨트롤러가 중력 보상). 
- 검증: 영 액션 90스텝에서 TCP 정지, 접시 정지(z 0.0335), 충돌 0/8, TCP-테두리 거리 0.635 m. 4096 env 1.3 s/iter.
- 보상은 v1 그대로(들기 + 목표 근접 + 안착). 어제 결과 분석의 보상 구조 문제(안착 시 이득 없음, 하강 보상 없음)는 아직 미수정 — 이번 3000 iter 실행으로 IK+홈 시작에서 접근/파지가 학습되는지 먼저 확인한 뒤 수정 예정(Human 검토 요청).
- 작성자: Claude

## 2026-10-03 RL v3: 단계별 커리큘럼 + 복셀 안전 실드 (Human 지시, Claude 설계)
- Human 지시: 학습을 4단계로 나누고 각 단계 성공률 95% 미만이면 다음 단계로 넘어가지 않음. 로봇은 Human 홈 관절값에서 시작. 프레임은 복셀로 반드시 충돌 방지. 1·2단계 접시는 테이블 가운데(프레임에서 멀리).
- 단계(누적, 각 단계는 이전 단계 정책에서 이어 학습, 모두 홈에서 시작): 1 도달(테두리 파지 자세, 위에서 접근, 그리퍼 열림, 툴 하향 ≤10°, 손가락 반지름 방향 ≤15°, TCP-테두리 ≤12 mm, 0.5 s 유지) / 2 파지+들기(5 cm 이상 들고 5 s 유지, Human 기준) / 3 옆으로 0.5 m 이동(들린 채 목표 상자 위 xy ≤3 cm, 1 s) / 4 내려놓기(목표 상자 위 xy ≤2.5 cm, z ≤1 cm, 그리퍼 열고 5 cm 후퇴, 1 s 정지). 성공 시 보너스(+100/+200/+200/+300) 후 종료.
- 내 추가 의견 반영: 1단계에서 "위에서 접근" 강제(테두리 위가 아니면 요구 높이 미달분에 벌점), 4단계에 손 놓고 후퇴까지 포함.
- 배치: 접시 (0.60, 0.00), 목표 상자 (0.60, -0.50) — 전 단계 동일(단계 간 전이 위해). 실제 배치(1.0, 0.33)와 다름 → 나중에 위치 랜덤화로 넓혀야 함.
- 안전 실드(`rl/safety_shield.py`): 1 cm 거리장(프레임/테이블 따로, `rl/make_cell_sdf.py`) + torch FK(링크 6개 × 200 점 + 그리퍼 상자 표면점). IK가 낸 관절 목표가 프레임 5 cm / 테이블 2 cm 이내면 거부하고 직전 안전 목표 유지, 거부 스텝 벌점. 검증(`rl/check_shield.py`): FK 오차 <0.001 mm, 홈 클리어런스 0.46 m(numpy 체커 0.462와 일치), 프레임 쪽 전속 돌진 시 5.0 cm에서 정지·접촉 0, 테이블 쪽 하강 시 2 cm에서 정지. 접시·상자는 실드에 없음(접촉 시 종료+벌점).
- 액션: TCP 상대 포즈 ×0.03/스텝(30 Hz). 망 [512,256,128]. 평가: `rl/eval_policy.py` 결정적 정책 2048 에피소드(관측 노이즈 켠 상태). 자동 진행: `rl/curriculum.py`(1000 iter 단위, 학습 중 성공률 ≥0.93이면 평가, 평가 ≥0.95면 통과, 단계당 최대 15000 iter, 실패 시 중단).
- 주의: 학습 속도(최대 0.9 m/s급)는 실기 "항상 최저 속도" 규칙과 다름 — 실기 적용 시 속도 제한/시간 스케일링 필요.
- 작성자: Claude

## 2026-10-03 접시 위치 x 0.60 → 1.10 m (Human 지시)
- Human: 접시가 로봇 베이스에 너무 가까움 → x = 1.10 m(접시 (1.10, 0.00), 목표 상자 (1.10, -0.50)). 1단계 처음부터 재시작(이전 1단계 실행은 `~/IsaacLab/logs/rsl_rl/m1013_2f85_pick_place/_old/`, `rl/logs/old/`로 보관).
- 도달성(top-down, TCP 0.1758 m): 접시 테두리 파지·위 8 cm·10 cm 들어올림 모두 도달, 프레임 클리어런스 0.313 m. 목표 상자 테두리 파지 자세는 도달하지만 프레임(기둥 (0.85, -0.56)) 클리어런스 0.045 m로 실드 여유 5 cm보다 작음 → 3·4단계에서 그 자세가 차단됨. 3단계 전에 결정 필요: 목표를 y=-0.45로, 실드 프레임 여유 4 cm로, 또는 접근 방향 변경(Human 검토).
- 검증: 홈에서 영 액션 90스텝 동안 접시 정지(z 0.0335), 충돌 0/8, TCP→테두리 0.566 m.
- 작성자: Claude

## 2026-10-06 파지 방식 재설계 제안: 접시 가장자리 옆면 파지 (Human 지적, Claude 제안 — Human 승인 필요, 아직 코드 미변경)
- Human 지적(1단계 GUI 확인): 학습된 파지는 접시 위에서 그리퍼를 눌러 잡으려는 형태이며 절대 잡히지 않음. 원하는 것은 **옆면에서 접시 끝자락(림)을 잡는 것**. 내 v3 설계가 "위에서 접근, 도구 하향, 손가락이 반지름 방향(림 안쪽/바깥쪽을 위에서 걸치는 파지)"이라 잘못 정의됨 → 1단계 통과(100%)는 잘못된 파지에 대한 것이라 무효, 1·2단계 재학습 필요.
- 새 파지 정의(제안): 로봇 쪽(-x)에서 수평 접근. 도구 z = +x(접시 중심 방향), 손가락 닫힘 축(TCP y) = 세로(±z) → 위 손가락이 림 위, 아래 손가락이 림 아래(접시 밑 상자 위 빈 공간 5.35 cm)로 들어가 림을 위아래에서 집음. TCP 목표 = 림 가장자리에서 12 mm 안쪽(패드가 TCP 뒤 33 mm까지 있으므로), 높이 = 접시 바닥 위 22 mm(림 얇은 껍질 z 16~25 mm 구간).
- 코드 변경 계획: ① `tool_down` 보상/`at_grasp` 조건을 "tool z·(+x) > cos10° AND |finger axis·z| > cos15°"로 교체 ② `r_above_first`(위에서 접근 강제) 삭제, 대신 림을 지나쳐 접시 안쪽으로 들어가거나 접시 위에서 누르는 자세(TCP가 접시 반경 안이고 림 상단보다 높음)에 벌점 ③ `held` 조건의 손가락 폐합 상한(<0.98)을 얇은 림 파지(폐합비 약 0.95~0.99)에 맞게 완화하고 "접시가 TCP를 따라 올라감"으로 판정 ④ 안전 실드의 그리퍼 외곽을 열린 손가락 실제 치수로 갱신(링크 6 기준 x ±0.074, y ±0.011, z 0~0.18) ⑤ 1~4단계 성공 조건·보상 재작성 후 1단계부터 다시.
- 가능성 검증(수치): 이 파지 자세로 IK 도달 가능(TCP (0.984, 0, 0.0555) 접시 (1.10, 0)). 프레임 여유 0.33 m, 아래쪽 8 cm 바깥 대기 자세도 도달. IK 해의 손목은 j5 ≈ -66°, j6 ≈ -179°로 홈(j5 7°, j6 -85°)에서 크게 돌아감 → 정책이 j6를 약 95° 돌려야 함. 접시 림 5 cm 들어올린 자세도 도달.
- **물리적 위험(중요):** 2F-85 열린 상태에서 팔로우어 링크가 TCP 기준 손가락 닫힘 방향으로 ±74.2 mm까지 튀어나옴(패드 면은 ±42.7 mm, 패드 바깥 ±49.3 mm). 림 높이(상자 53.5 mm → TCP z 0.0555 m)에서 아래 팔로우어의 최저점은 z = -0.0187 m = 테이블 윗면(-0.020 m) 위 **1.3 mm**. 실드 테이블 여유 2 cm를 못 맞춤. 시뮬 URDF는 팔로우어 충돌이 없어(패드 상자만) 시뮬에서는 안 걸리지만 실제 그리퍼는 테이블에 닿을 수 있음. 여유 1 cm를 확보하려면 상자 높이 ≥ 약 63 mm, 2 cm는 ≥ 약 70 mm 필요. Human이 상자 높이를 정해야 함.
- 실제 림 두께·단면은 스캔(open3d 단순화 3000 삼각형)에서 추정한 값이며 얇은 껍질로 보임. 림 곡률/두께는 미측정.
- 작성자: Claude

## 2026-10-06 고정 D455 재캘리브레이션(카메라 이동 후): 손목 카메라로 마커 위치를 재고 PnP로 풀기 (Claude, Human 승인: sim 반영)
- 배경: Human이 고정 D455를 프레임 상단(로봇 기준 왼쪽, 앞뒤 중간)으로 옮김 → 10/1~10/2 hand-eye(`T_base_cam*`, 카메라 x≈1.5, z≈0.10)는 무효. 그리퍼를 달아 플랜지에 ArUco를 붙일 수 없어 방식 변경: 느슨한 마커(DICT_4X4_100 id 1, 100 mm 출력, 크기 비율 0.98 가정, 줄자 약 98 mm)를 테이블 위 여러 곳에 놓고 손목 D455(`T_flange_cam`)로 base 좌표를 구한 뒤 고정 카메라가 같은 마커를 봄. 로봇은 Human이 티치패드로 이동(AI는 로봇 명령 없음, 위치 읽기 전용).
- 손목 hand-eye 재캘리브레이션 필요했음: 10/2 값으로 마커 base 좌표가 z 0.17~0.33 m(실제 ~0.05 m), 법선이 수평 → 그리퍼 장착 후 카메라 위치 변동. 새 세션 `data/wrist_handeye/20261006_111318`(20샘플 중 0번이 33.6°/372 mm 이상치) → 0번 제외 재풀이 `20261006_111318_clean`: 피팅 0.88 mm/0.37°, leave-one-out 1.13 mm/0.40°(최대 2.21 mm), 마커 base (1.194, -0.032, 0.055), 법선 z 0.998. 한계: 샘플이 로봇 위치 약 15 cm, 기울기 36~48°에 몰림.
- 고정 카메라: 세션 `20261006_122330_fromwrist` 13곳(실제 약 11곳, 9=10·11=12 중복, 마커 높이가 전부 4~6 cm 같은 평면). 위치별 `T_base_cam`을 평균하면 위치마다 11~111 mm/0.6~6° 흩어짐(마커 하나의 방향 오차×거리가 증폭) → **전체 꼭짓점 PnP**(base 3D는 손목 측정, 픽셀은 고정 카메라 관측)로 변경. 결과 `T_base_cam_pnp.json`: 카메라 (1.218, 0.540, 1.127) m, 광축 (-0.024, -0.533, -0.846). 위치 하나 빼고 풀기 변화 대부분 0.4~5 mm(최대 8.8 mm/0.34°), 빠진 위치 예측 재투영 오차 평균 5.4 mm·최대 10.5 mm → 기대 정확도 약 5~10 mm. 독립 확인: 줄자 렌즈 높이 약 113 cm vs 결과 114.7 cm(테이블 윗면 기준). 마커 크기 비율 스캔 최적 0.98~1.00(평탄), 0.98→0.99에서 카메라 y가 4 mm 변함. 6곳 결과(1.208, 0.559, 1.110)와 약 27 mm 차이 → 6곳으로는 부족했음.
- 코드: `calibration/fixed_cam_from_wrist.py`(G 손목/F 고정/N 저장/S 계산, 고정 카메라 라이브 창, `--resolve <세션>`로 PnP 재계산, `--selftest`), `board_probe.py`(딕셔너리·ID 자동 탐지), `marker_observe.py`, `calib_utils.make_detector(pixels_per_cell)`(OpenCV 기본 4로는 비스듬한 100 mm 마커가 해독 실패, 10으로 해결; `wrist_handeye.py`도 기본 10). 버그 수정: N에서 키 이름 충돌(wrist/fixed 접두사).
- 반영: `real_pose_dryrun.latest_t_base_cam()`이 가장 최근 세션의 `T_base_cam_pnp.json`을 우선 선택(수정 전에는 6곳 평균 파일 `112054_fromwrist/T_base_cam.json`이 자동 선택될 위험이 있었음). 이전 세션의 xy 오프셋/zfix 변형은 옛 카메라 위치용이라 새 세션에는 적용 안 함. `cube_gt_compare.py` 기본 비교 대상에 PnP 파일 포함. `sim_home_scene.py` 카메라를 PnP 결과로 구동(기본 `--t_base_cam` 최신 PnP, 수동 `--cam-xyz/--cam-target` 덮어쓰기 유지), 렌더 1280x720(실제와 같은 시야).
- 검증: sim 렌더와 실제 고정 카메라 영상을 겹쳐 봄 — 프레임 기둥 2개, 테이블 가장자리, 접시 위치(실제 약 1.19, -0.03 m; 오차 약 1 cm 수준)가 일치. 로봇 자세는 실제가 홈이 아니라 달라 겹치지 않음.
- 실제 접시 위치(손목이 잰 마커 위치 기준 (1.19, -0.03)): 측면 파지 자세 IK 가능, 프레임 여유 0.33 m. 같은 방향 0.45 m 옆 목표 (1.19, -0.48)는 도달은 되지만 프레임 여유 2 cm < 실드 5 cm → 실제 배치로 학습하려면 목표 위치 조정 필요.
- 미해결/주의: ① 실제 로봇 동작에 쓰기 전에 FP-vs-손목 GT 위치 오차를 새 카메라로 다시 측정(`cube_gt_compare.py`)해야 함(옛 오프셋 보정은 폐기) ② 마커 높이를 다양하게 한 4~5곳 추가하면 더 좋아짐 ③ 손목 카메라 마운트를 건드리면 손목 캘리브레이션부터 재실행 ④ RL 환경(`rl/`)은 이 카메라를 쓰지 않음(상태 기반 관측).
- 작성자: Claude

## 2026-10-06 RL v4 현황 및 Stage 2 파지 정체 진단 기록 (Codex)
- 결정/기록: 현재 Human 확정 task를 3단계(Stage 1 측면 사전파지 위치 접근, Stage 2 파지 후 +z lift 및 5초 hold, Stage 3 이동/place)로 정리한다. 오늘 Stage 1은 deterministic 2048회 평가에서 100% 통과했으며 Stage 2는 1558/1999 iter 관측 시 success 0%, timeout 100%, close/lift reward 0임을 기록한다. `at_grasp`의 기하 tolerance와 close reward gating은 정체의 원인 후보로만 둔다. reward/success 조건은 이번 업데이트에서 변경하지 않았다.
- 이유: 기존 공유 handoff가 이전 top-down/4-stage RL v3 내용을 현재 작업처럼 적고 있어 Claude와 Codex 간 상태 혼선을 막아야 한다. 지표상 Stage 2가 아직 파지하지 못하지만 GUI checkpoint 로그만으로 실제 관절 정지 위치까지 입증되지는 않는다.
- 다음: Stage 2를 임의 중단하지 않고 현재 청크 결과를 확인한다. 실패 지속 시 GUI trajectory 및 rim 거리/tool·finger 각도/finger closure를 계측한 뒤 변경안을 검토한다. 핵심 reward 로직 변경 전 결과/근거를 기록하고 Human review를 요청한다.
- 작성자: Codex

## 2026-10-06 Stage 2 파지 정체 원인 확정 및 v5 수정(opt-in `PP_V5=1`) (Claude, Human 요청 — 보상·성공조건 변경이므로 Human 검토 필요)
- 진단 방법: `rl/check_rim_grasp.py`(신규) — 1단계 정책(`rim_v4/2026-10-06_11-18-23_stage1/model_999.pt`)으로 ready까지 간 뒤 스크립트로 림 전진 → 닫기 → TCP 8 cm 상승 → 5초 유지, 8 env. 학습 없이 과제가 물리적으로 가능한지 확인.
- 원인 1 (결정적): **안전 실드가 파지 자세 자체를 금지.** 상자 54.6 mm에서 림 쪽 전진 시 4초 중 117~118 스텝 차단, 림 50~69 mm 앞에서 정지(테이블 여유 20.1 mm = 실드 한계). 실드를 끄면 림 도달 시 테이블 여유 10.6 mm(SDF 기준, 실제 약 5 mm) — 열린 2F-85의 아래쪽 팔로우어가 테이블 가까이 내려감. 정책은 at_grasp에 도달할 방법이 없어 ready에 머무는 것이 최선(로그 `grasp_reach` 0.06/s = 림까지 약 72 mm, `close_at_grasp`·`lift` 0). 10/6 오전 DECISIONS "파지 방식 재설계"에서 예고한 상자 높이 문제와 같음.
- 원인 2: `FINGER_CLOSED=0.030`은 간격 25.4 mm를 남겨 림에 닿지 않음(실드 끈 시험: 손가락 30.0 mm까지 닫히고 접시 상승 0). 끝까지 닫기(0.0425)면 약 37 mm에서 림에 걸려 멈추고 접시가 들림.
- 원인 3: `held`가 world 좌표 기준이라, 한쪽 림을 잡고 들면 접시가 약 20° 기울어 즉시 false → 들어도 lift/성공 보상 없음.
- 원인 4: at_grasp 방향 허용치 10°인데 1단계 정책은 10.0~10.2°에 머묾(1단계 게이트 경계) → 림에 가도 at_grasp가 거의 안 켜짐.
- 원인 5: 보상 골짜기 — 2단계에서도 reach_fine/axis가 ready 점으로 당겨 ready ≈ 1.56/s, 중간 ≈ 1.22/s, 림 ≈ 2.5/s(계산). 탐색 노이즈 0.34로 골짜기를 넘기 어려움.
- 부가 버그: v4 `plate_moved`가 명목 위치 기준인데 reset이 이미 최대 1.4 cm 이동시킴(1단계 오종료 위험). `watch_latest.py`의 시작 2초 "그리퍼 열기"가 실제로는 닫기(Isaac Lab binary: 음수 = 닫기) — 정책은 실제 관절값으로 보상받아 영향 없음, 뷰어만 수정.
- v5 수정(`PP_V5=1`일 때만, 미설정 시 v4 동작 그대로 — 실행 중 커리큘럼 보호): ① 닫기 = 전체 스트로크 0.0425, `fingers` 정규화도 동일 ② `held` = 접시 좌표계에서 TCP가 림 원(반경 128 mm, 높이 16 mm)에서 3 cm 이내 AND 손가락이 무언가에 걸려 멈춤(0.5 < 닫힘비 < 0.97) ③ at_grasp 방향 20°/25° ④ 2단계 이상에서 reach_fine·axis 가중치 0, 넓은 림 접근항(std 0.10, w 1.0) 추가 ⑤ `plate_moved`는 에피소드 시작 위치 기준. 실험 이름 `m1013_2f85_rim_v5`, 로그 `rl/logs/curriculum_v5/`로 분리.
- 검증(실드 켬, v5, 스크립트): 상자 54.6 mm → 여전히 전부 차단(at_grasp 0/8). **상자 70 mm(`PP_BOX_H=0.070`) → 차단 0, 테이블 여유 26 mm, at_grasp 8/8, held 8/8 (닫기·상승·5초 유지 내내), 손가락 35~39 mm에서 정지, 접시 중심 상승 16~46 mm(TCP 80 mm 상승, 기울어짐).** 성공 조건 LIFT_OK 5 cm(중심 기준)는 TCP를 약 10~12 cm 올려야 충족 — 정책이 학습할 몫.
- Human 결정 필요: 실제 상자를 약 70 mm로 높일지(권장, 실제 그리퍼–테이블 여유 약 2 cm). 54.6 mm 유지 시 실드 여유를 낮춰야 하는데 실제 여유 약 5 mm라 캘리브레이션 오차(5~10 mm) 대비 위험 → 비권장.
- 남는 한계: sim 림 충돌체는 두께 없는 스캔을 볼록 분해한 것이라 약 11 mm 두께로 동작(실제 림은 수 mm) → 실제 손가락은 더 닫힘. 접시 질량 0.3 kg·마찰 1.0은 가정값.
- 작성자: Claude

## 2026-10-06 접시 받침 높임(Human: 접시 높이 14 cm) — 캘리브레이션 쪽 반영 (Claude)
- Human: 접시를 14 cm 높이로 올림. RL(`rl/`) 수정은 Codex 담당(Human이 전달).
- 해석: 고정 D455 깊이 + `T_base_cam_pnp`로 측정한 결과, 마커가 놓인 접시 바닥면은 테이블 위 118 mm, 림 위쪽은 최대 142 mm(원시값). 같은 방법으로 테이블 자체가 +6~9 mm로 읽힘(캘리브레이션 z 편차) → 보정하면 **접시 윗면 약 13~14 cm, 밑면 약 10.5~11.5 cm**. 따라서 "14 cm"는 접시 윗면(림) 높이로 판단, 받침 높이 = 0.14 − 0.0254 = **0.1146 m**. Human이 "상자 높이 14 cm"를 뜻했다면 정정 필요(측정과 약 3 cm 차이로 가능성 낮음).
- 접시 중심(림 가장자리 원 맞춤, 깊이 기반): **(1.215, -0.009) m**, 반지름 126 mm(스캔 133 mm, 가장자리 깊이 혼합으로 작게 나옴). 이전 손목 측정 (1.191, -0.033)에서 약 3.5 cm 이동. 단일 마커 PnP(50 px)는 마커 크기 0.98/1.00 가정에 따라 높이가 98~119 mm로 변해 깊이 측정을 채택.
- 캘리브레이션 유효성: 고정 카메라 PnP는 마커 높이 4~6 cm 한 평면으로만 풀었음. 같은 데이터로 한 위치씩 빼고 풀면 접시 위치의 3D 변화가 z 3.5 cm에서 최대 1.8 mm, z 12 cm에서 2.3 mm, 14.5 cm에서 2.5 mm(부트스트랩 95% 8.4 → 9.6 mm) → **높인 위치에서도 캘리브레이션 재실행 불필요**. 손목 hand-eye(`T_flange_cam`)는 접시 높이와 무관. 실제 파이프라인(FP 6D → `T_base_cam`)은 접시 높이 사전값을 쓰지 않아 코드 변경 없음. 직접 확인을 원하면 높인 상자 위에 마커를 놓고 손목 G/고정 F 2~3곳 추가 후 `--resolve`.
- 반영: `calibration/sim_home_scene.py` 기본값 `--plate-lift-m 0.1146`, `--plate-xy 1.215 -0.009`. 헤드리스 렌더를 실제 고정 카메라 프레임(`calibration/data/board_probe/20261006_131758_raw.png`)과 겹쳐 접시 위치·크기 약 1 cm 이내 일치 확인.
- Codex 참고: `calibration/preview_rim_trajectory.py`의 `rim_z = -0.02 + 0.0535 + 0.016`, 접시 중심 (1.10, 0)이 하드코딩되어 있음(Codex 파일, 미수정). RL `PP_BOX_H`/`PLATE_XY`에 위 값 사용 권장. 받침 11.5 cm면 림 높이에서 열린 그리퍼 아래 링크–테이블 여유가 약 6 cm 이상으로 실드(2 cm) 문제 해소 예상(미시험).
- 작성자: Claude

## 2026-10-06 GUI 확인된 15 mm 깊은 파지와 방향 유지를 Stage 2 v6에 반영 (Human 코드 작성 요청, Codex)
- Human이 14 cm 높이·15 mm 추가 진입·손목 방향 유지 GUI를 확인하고 이 설정으로 코드 수정을 요청함. `PP_VERSION=v6`를 도입하고 `experiment_settings.py`를 train/agent/curriculum/watch에서 공유한다. PP_V5=1은 legacy v5 호환이며 명시적 PP_VERSION이 우선한다.
- 기본 받침 114.6 mm(접시 맨 위 table 기준 약140 mm). 기존 Stage 1의 ready 목표는 변경하지 않고 Stage 2+의 RIM_OFFSET만 원래 접시 중심 쪽으로15 mm 이동. 기존 홈 관절·실드·파지력·마찰 유지.
- Stage 2 SafeDiffIKAction은 열린 손가락으로 파지 목표35 mm 이내에 진입하고 tool/finger 오차20°/25° 이내이면 해당 TCP quaternion을 저장한다. 이후 rotation action은 저장된 방향으로 되돌리는 명령으로 대체하고 translation/gripper는 RL 명령을 사용한다. 모든 회전 보정 결과도 기존 실드 검사를 통과해야 함. reset 때 방향 고정 해제. 이 제어는 시연 전용이 아니라 train/eval/play 공통 환경에 구현됐다.
- 성공 조건: 기존 held 및 plate origin 50 mm 상승을 유지하면서 기울기<25°를 연속5초 만족하도록 보강. 추가 수평 유지 보상 weight1.0, Stage 2 제한24초. angular speed<0.2 rad/s 조건은 진단 시 접촉 진동으로 일부 hold가 초기화되어 최종 설정에서는 제외; 필요시 실제 센서/필터 기반 slip 검증을 별도 설계한다. 25°는 GUI 측정9~16°에 여유를 둔 sim 기준이며 실기 파손/슬립 보장이 아님.
- `run_stage2_v6.sh`는 기존 v4 Stage 1 model_999.pt에서 Stage 2를 시작하는 명령을 제공한다. 신규 실험 `m1013_2f85_rim_v6`, 로그 `rl/logs/curriculum_v6/`. 원본 텍스처 렌더링/근사 충돌체 유지. 4096 env의 전체 학습 성공률은 아직 측정하지 않았고 실제 로봇 명령 없음. 핵심 로직 검토용 설명 `rl/README_stage2_v6.md`; 변경 전 백업 `rl/backups/20261006_before_v6/`.
- 작성자: Codex

## 2026-10-06 v6 2단계 "접근 안 함" 원인 확정과 v7 (Claude, Human 지시: "알고리즘 변경 후 RL이 스스로 파지 시도하는지 확인")
- 원인: v6는 받침을 54.6 → 114.6 mm로 올렸지만 1단계를 새 높이에서 재학습하지 않고 v4 1단계 `model_999.pt`에서 바로 2단계를 시작함. 결정적 평가(512 ep): 받침 54.6 mm 성공 100%, **114.6 mm 성공 0%(전부 시간 초과)**. 즉 2단계 시작 정책이 ready에도 못 감(Codex 계측: ready에서 47~50 mm, 림에서 95~107 mm 떨어진 채 정지, 실드 차단 0). 실드나 보상 골짜기가 아님 — v6 보상은 ready→림 경로에서 단조 증가(초당 0.86 → 1.37 → 3.31, 계산).
- v7 = v6 과제/보상/성공조건 그대로 + 학습 방식만 변경(`PP_VERSION=v7`, 실험 `m1013_2f85_rim_v7`, 로그 `rl/logs/curriculum_v7/`):
  ① 1단계를 새 높이(114.6 mm)에서 `model_999.pt`부터 재학습, 기존 게이트(결정적 ≥95%) 통과 후 2단계 자동 시작, 2단계에서 정지(`--last_stage 2`).
  ② 2단계 **학습 에피소드의 50%를 ready 자세에서 시작**(`PP_READY_START_FRAC=0.5`, 이벤트 `reset_ready_start`). 커리큘럼이 평가·영상 실행에서는 이 변수를 제거 → **게이트 평가와 GUI는 항상 Human 홈 자세 시작**(Human 규칙 유지). ready 관절값 `rl/assets/ready_joints_v7.json`(-7.12, 31.39, 110.93, -1.07, -31.89, 1.55°)은 env의 실드 포함 diff-IK로 홈에서 구함: ready 오차 0 mm/0°/0°, 프레임 여유 379 mm, 테이블 82 mm. 16 env 시험에서 50% 무작위 시작, 30스텝 정지 유지·실드 차단 0.
  ③ PPO `entropy_coef` 0.006 → 0.003(v7만). v6 2단계 초반 탐색 노이즈가 0.34 → 0.47로 커지며 접근 보상이 줄었음.
  ④ `SafeDiffIKAction.reset`의 안전 목표 = 리셋 이벤트가 쓴 실제 관절값(홈 시작이면 기존과 동일). ready 시작 env가 첫 차단 때 홈 목표로 튀는 것을 방지.
- 유지: 실드(프레임 5 cm/테이블 2 cm), 성공 = held + 5 cm 상승 + 기울기 <25° 5초, v6 방향 유지 controller, 15 mm 진입, `PLATE_XY` (1.10, 0)(실제 (1.215, -0.009)는 미반영).
- 판단 기준: 2단계에서 정책이 스스로 림 접근 → 닫기 → 들기를 시도하는지(`grasp_reach`, `close_at_grasp`, `lift` 증가, 홈 시작 결정적 평가). ready 시작은 학습 보조일 뿐 성공률은 홈 시작으로만 셈.
- 백업: `rl/backups/20261006_before_v7/`. 실행: `bash rl/run_stage12_v7.sh --num_envs 4096 --chunk 500 --max_iters 15000`.
- 작성자: Claude

## 2026-10-06 v8: 홈 → ready는 RL이 아닌 관절 보간, RL은 ready부터 파지 학습 (Human 결정: A안)
- 배경: v7 1단계가 성공률 98%를 냈지만 GUI에서 관절 움직임이 매우 비효율(Human 지적). 원인: 보상이 경로 효율을 거의 따지지 않음(관절 속도 −0.001, 동작 변화 −0.01), 행동이 TCP 변화량→IK라 홈의 J5 7.46°(손목 특이점 근처)에서 J4/J6가 크게 휘둘림, J6 약 87° 회전 필요.
- 결정(A): 홈 → ready는 정답이 정해진 문제이므로 RL 대상에서 제외. 실로봇은 가장 느린 MoveJ로 ready 관절값(`rl/assets/ready_joints_v7.json`: -7.12, 31.39, 110.93, -1.07, -31.89, 1.55°)까지 이동. RL은 ready부터 접근→닫기→들기→유지만 학습. **"에피소드는 홈에서 시작" 규칙은 2단계부터 "ready에서 시작"으로 변경**(학습·게이트 평가·GUI 모두 동일).
- v8 = v7(각도 보상, 엔트로피 0.003, v6 과제·성공조건·실드·15 mm 진입·방향 유지) + 2단계 이상 전 에피소드 ready 시작, 2단계 에피소드 14초. 초기 가중치 v7 1단계 `model_1700.pt`. 실행 `bash rl/run_stage2_v8.sh --num_envs 4096 --chunk 500 --max_iters 15000`(2단계 게이트 후 정지).
- Human 규칙 추가: 다음 단계로 넘어가기 전 항상 GUI를 띄워 학습 현황을 보고하고 승인 후 진행(커리큘럼은 `--last_stage`로 현재 단계에서 정지).
- 작성자: Claude

## 2026-10-06 v9: IK 위치 목표를 누적 유지(동작 0 = 정지) — 2단계 "잡고만 있음" 원인 수정 (Claude)
- 증상(v8, 처음부터 학습 2000 it): 림 접근·파지는 학습(`close_at_grasp` 약 0.85)했으나 들기 보상이 iteration 650 근처 0.47에서 1850에 0.08로 감소, 5 cm 이상 수평 유지 0회, 성공 0%.
- 계측(`model_1900`, 32 env): 1초 안에 파지 후 접시를 약 18 mm 들었다가 TCP가 23 mm 내려가 접시를 받침에 누른 채(들림 −6 mm, 기울기 12°) 유지, `held` 97%, 실드 차단 0. 항목별 보상은 오히려 들었을 때 더 큼(19 mm: 합 2.40/초 vs −4 mm: 1.89/초) → 보상 문제 아님.
- 원인: 상대 모드 diff-IK는 매 정책 스텝 "현재 TCP + 변화량"을 목표로 잡음 → 접시 무게로 팔이 처지면 목표도 같이 처짐. 스크립트 시험: 8 cm 들고 동작 0 → TCP가 약 7 mm/초 하강(5초 뒤 접시 들림 68 → 35 mm). 5초 유지하려면 처짐을 정확히 상쇄하는 상승 동작을 계속 내야 해서 학습이 어려웠음. 실로봇 컨트롤러는 절대 목표를 유지하므로 sim이 비현실적이었음.
- 수정(v9, `PP_VERSION=v9`): `SafeDiffIKAction`이 TCP 위치 목표를 누적 유지(목표 += 정책 변화량), 목표가 실제 TCP보다 최대 3 cm까지만 앞서게 제한(실드·접촉으로 막힐 때 목표가 도망가는 것 방지), 리셋 직후 2스텝은 실제 TCP로 다시 맞춤. 회전은 기존 v6 방향 고정(림 접근 시 캡처) 유지. 그 외 v8과 동일(ready 시작, 14초, 각도 보상, 엔트로피 0.003).
- 검증: 같은 시험에서 동작 0 유지 5초 동안 TCP 80.0 mm·접시 들림 58.0 mm 그대로(v8: 79 → 45 mm). 첫 실행은 장면 생성 전 Isaac 프로세스 비정상 종료(코드 무관), 재실행 정상.
- 학습: v8 `model_1996.pt`에서 이어서(접근·파지 습득 상태), `bash rl/run_stage2_v9.sh --num_envs 4096 --chunk 500 --max_iters 15000`, 2단계 후 정지.
- 작성자: Claude

## 2026-10-06 v10: 잡은 상태에서 TCP 상승 보상 추가 (Claude)
- v9 2단계 1000 it(17:51~18:55, `model_2900`): `close_at_grasp` 약 0.85, `lift` 0.34~0.40에서 정체, 5 cm 이상 수평 유지 0, 성공 0%. 계측: 파지 후 접시 들림 약 14 mm(닫을 때 기울어짐에 의한 것)이고 TCP는 거의 그대로(+1~4 mm). 처짐은 v9로 해결(14초 일정). 정책 동작에 1초간 +z를 더하면 TCP 22 mm·접시 최대 34 mm 상승 후 정책이 스스로 원래 높이로 복귀, 접시가 손가락 축으로 돌며 중심 들림이 8 mm까지 떨어짐(그립 위치제어 약 11~15 N).
- v10(`PP_VERSION=v10`): `r_tcp_rise` = held × clamp((TCP z − 처음 held 된 순간의 TCP z)/0.10, 0, 1), 가중치 3.0. 성공 조건 불변(접시 5 cm 이상·기울기 <25°·5초). 버전 플래그를 숫자 비교(`_VN >= n`)로 정리(동작 동일). 학습: v9 `model_2900.pt`에서 이어서 18:55, `bash rl/run_stage2_v10.sh --num_envs 4096 --chunk 500 --max_iters 15000`.
- 다음 후보(정체 지속 시): 그립을 위치제어 대신 일정 힘(실제 2F-85 설정값)으로, 단 접시 충돌체(두께 없는 스캔의 볼록 분해) 개선이 먼저 필요할 수 있음.
- 작성자: Claude

## 2026-10-06 rl/ 코드 정리: 역할별 6개 파일 + config.yaml (Human 지시, Claude)
- Human 지시: real_time_project처럼 train / eval / utils / env_setup 식으로 정리, 자잘한 파일 금지(기능은 함수·모드로), `__main__` 진입, 옵션이 20~30개 넘으면 폴더 안 YAML, 파일 수보다 코드 길이 허용하되 1000줄·스파게티 금지. 앞으로 모든 코드에 적용.
- 결과 `rl/`: `config.yaml`(133줄), `plate_env.py`(약 560), `rl_utils.py`(약 280), `train.py`(약 195), `eval.py`(약 255), `env_setup.py`(약 280). 30개 파일(check_*/make_*/run_*.sh/*_backup/experiment_settings/pick_place_* 등)과 backups/는 `rl/archive/2026-10-06_pre_refactor/`로 이동(삭제 안 함).
- 동작 정의: v10과 동일(보상 18항목·가중치, 성공/종료 조건, 관측 43차원 순서, 행동 7차원, ready 시작, 위치 유지 IK, 방향 고정, 실드). 버전 분기(V5~V10)와 1단계(홈→ready) 전용 항목(at_ready, reach_fine, axis, 1단계 성공)은 제거. 변경점 2가지: ① 방향 고정을 2단계만이 아니라 3단계에도 적용(3단계도 같은 파지 포함) ② 64 env 이하 진단 실행은 PhysX 접촉 버퍼를 작게(메모리 절약).
- 학습 실행 방식 변경: Isaac Lab `train.py`를 import 가로채기로 부르던 방식 대신 rsl_rl `OnPolicyRunner`를 직접 생성(`train.py chunk`). 학습 로그 줄 단위 출력(`app.close()`가 버퍼를 버리던 문제). 단계 통과 시 자동으로 다음 단계로 가지 않음(`--last_stage` 기본 = 시작 단계, Human GUI 확인 규칙).
- 실험 이름 `m1013_2f85_rim`(새 폴더). v10 `model_3000.pt` 등 기존 체크포인트는 `--init`으로 그대로 사용 가능(확인함).
- 작성자: Claude

## 2026-10-06 IL 성공 데모 수집 (Human 요청, Codex)
- 목적: 기존에 성공한 스크립트 물리 데모를 현재 정리된 RL 환경에서 재현하여 BC용 관측-행동 데이터 저장. IL/collect.py와 IL/data/ 생성, 기존 rl/ 학습·보상·모델 코드 변경 없음. 학습이나 실로봇 명령은 실행하지 않음.
- 범위: ready 시작 Stage2 접근→15mm 진입→닫기→TCP 약160mm 상승→유지. Stage3 이동/place 및 home→ready 궤적 제외. 데모의 동작 명령만 제어하며 접시 위치를 임의로 바꾸거나 붙이지 않는다.
- 기존 실드 프레임50mm/테이블20mm 유지. 차단 없는 궤적, 팔/그리퍼몸체 힘<=0.5N(4물리스텝 sensor history), 실제 관절 SDF 여유(frame>=49mm/table>=19mm:1mm 수치 허용), held+접시50mm 초과상승+tilt25°미만150연속스텝 및 최종유지, 유한한 데이터만 성공 목록에 저장. 실패 목록은 manifest에 기록.
- 시뮬 물성/근사충돌체/held기하판정은 기존과 동일. 손가락-접시 접촉은 허용. 실드/센서는 실제장비 안전인증이 아님.
- 데이터: obs/next_obs43D(좌표노이즈 유지), actions7D(BC정답:제어기보정 전), controller_actions6D/processed_arm_actions/관절목표/제어기내부상태/물리TCP·접시/안전지표 별도. 데모의 GT좌표 기반 teacher 명령을 정책관측과 함께 기록.
- 50성공 목표,8환경,seed123,최대20배치. 접시xy±10mm/yaw무작위, expert속도0.85~1.15배/TCP상승150~170mm. episode별train40/validation10 예정; 별도 rollout 성공평가가 필요.

## 2026-10-06 calibration/ 코드 정리: 33개 → 12개 파일 + config.yaml (Human 지시, Claude)
- Human 지시: rl/과 같은 원칙(역할별 파일, 기능은 함수·모드, 옵션 많으면 YAML, `__main__`, 1000줄·스파게티 금지), 구조는 calibration에 맞게.
- 구성(실행 환경 기준): `config.yaml`, `calib_core.py`(수학·풀이·파일 선택), `robot_model.py`(FK/IK·작업 셀·경로), `sensors.py`(D455·UDP·캡처), `calibrate.py`(wrist/fixed/gt/selftest), `camera_tools.py`(preview/marker/probe), `pipeline.py`(큐브 파이프라인, 옛 cube_pipeline + real_pose_dryrun 계획 함수), `ros2_nodes.py`(flange/publisher/mover, 시스템 Python, rclpy는 함수 안에서 import), `sim_scene.py`(Isaac Sim 디지털 트윈), `tests.py`, `run.sh`(connect/stream/pipeline/sim/tests), `README.md`. 예전 파일 31개는 `calibration/archive/2026-10-06_pre_refactor/`.
- 보관만 하고 새 구조에 넣지 않은 기능: 플랜지 마커 eye-to-hand 자동 캡처(그리퍼 장착 후 불가, 풀이 함수는 `calib_core.solve_eye_to_hand`로 유지), 10/1 샘플 포렌식(analyze_handeye_samples), 플랜지 마커 검증(verify_handeye_marker), 펜던트 비교(predict_base_pose), 독립 실행 dry-run 루프(real_pose_dryrun main; pipeline이 대체), Codex 일회성 sim 생성기(generate_2f85_close_poses, preview_rim_trajectory; 결과 데이터는 data/에 그대로).
- 동작 변경 2가지(둘 다 실로봇 안전 방향): ① **버그 수정** `pipeline --offline-test`가 시작 자세를 `np.radians` 두 번 적용해 홈이 아니라 거의 0° 자세에서 계획하고 있었음 → 홈 자세로 수정. ② 그 결과 드러난 문제: 홈 → 접근 자세가 직선 1동작이면 J6가 85.9° 회전해 mover의 45°/동작 기준에 걸려 **실로봇에서 홈에서 시작하면 계획이 항상 거부됐을 것**. `split_long_moves`로 같은 직선 관절 경로 위에 경유점을 균등 삽입(경로·충돌 여유 동일, `go` 횟수만 증가) → 오프라인 시험 승인(43°씩 2동작). ③ 다시 계산 시 기존 결과 덮어쓰기 거부(`--out`/`--force`).
- 검증: 회귀 시험 20/20 통과(옛 test_core 11 + test_cube_pipeline 9 이식), `calibrate.py selftest` 통과, 저장 샘플로 다시 푼 결과가 기존과 **동일**(wrist `20261006_111318_clean` T_flange_cam 차이 0, 14샘플·피팅 0.88 mm; fixed `20261006_122330_fromwrist` T_base_cam_pnp 차이 0), sim 디지털 트윈 렌더가 정리 전과 동일(차이 픽셀 0%), rl/의 셀 거리장 재생성 동일, `ros2_nodes.py flange`는 시스템 Python+ROS에서 import·실행 확인(bringup 꺼져 있어 "서비스 없음"으로 정상 종료; 실시간 스트림은 다음 연결 때 확인 필요), mover 검사 모드가 오프라인 계획 거부 확인.
- config 값 출처: 마커 100 mm·DICT_4X4_100·배율 0.98·pixels_per_cell 10, 손목 거리 필터 0.3~0.7 m는 10/6 실제 사용값(0.7 m로 해야 기존 결과 재현). gt_compare는 큐브 마커(45 mm, DICT_4X4_50).
- rl/ 연동: `rl/plate_env.py`·`env_setup.py`가 `workcell` 대신 `robot_model.frame_layout/measured_cell` 사용(셀 배치 동일 확인).
- 작성자: Claude

## 2026-10-06 IL → RL 방향에 대한 의견 (Claude 제안 — Human/Codex 결정 대기)
- 방향 동의: 스크립트 전문가가 sim에서 약 98%(51회 중 50) 성공하므로 "시연으로 정책 초기화 → RL 미세조정"(DAPG, AWAC, RLPD 계열)은 표준적이고, 지금까지 PPO가 탐색으로 못 찾던 들기·유지를 건너뛸 수 있다.
- RL을 쓸 구간: **접촉 이후(닫기 → 들기 → 유지)와 3단계 place/release.** 접근(ready → 림)은 스크립트/BC로 충분. RL의 가치는 "전문가가 못 하는 상황"에서 나와야 함: 관측 노이즈 확대(FP·캘리브레이션 5~10 mm), 접시 질량·마찰·그립 강성 랜덤화, 손가락 사이 회전(기울기 증가) 회복, 미끄러짐 시 재파지.
- 권장 절차: ① BC로 actor 학습(train 40), 검증은 손실이 아니라 env 결정적 성공률(val 10 시작 조건 + 새 시드) ② actor 고정 상태로 critic만 먼저 학습(무작위 critic의 PPO 업데이트가 BC 정책을 망가뜨리는 것 방지) ③ PPO 시작 노이즈를 낮게(0.1~0.2) + BC 손실을 점차 줄이며 함께 사용. 대안: 스크립트 전문가 위에 RL이 보정만 학습하는 residual RL(가장 단순·안전).
- 데이터 보강 권장: 복구 데모(전문가 경로에 노이즈를 넣고 전문가가 되돌리는 DART 방식), 단계 시간 무작위화, 위 도메인 랜덤화, 에피소드를 env 길이(420 스텝)에 맞춤.
- 비교 기준: 지금 돌고 있는 v10 순수 PPO(같은 env, 같은 95% 게이트)를 기준선으로 둠. 또한 실로봇 배치는 결국 "FP pose + 스크립트 전문가 + 저속 이동"일 수 있으므로, RL/IL 정책이 그보다 나은 점(강건성 시험)을 숫자로 보여야 의미가 있음.
- 작성자: Claude

## 2026-10-07 Codex: v9/v10 비교 및 W&B 기록 추가(Human 요청)
- v9 대비v10은 held 상태 TCP상승 보상(weight3.0)을 추가; PPO설정·Stage2 성공(held+실제접시50mm초과상승+tilt25°미만5초)·실드50/20mm 유지. BC/전문가 궤적 사용 없음. 인간 설계 ready/inset/손목 방향 고정/위치유지IK는 v9에도 존재. GT 관측/기하held의실기일반화 한계 별도 기재. v9동일추가시간 대조군없어 보상변경만의 인과효과는 미확인.
- train.py에 TensorBoard기록을 그대로 W&B로 전달하는 runner adapter 및gate별도run추가,config.yaml tracking.enabled/project/entity/mode(auto)추가. 매PPOupdate단위(기본98304transitions,약4초)로loss/성공·충돌/보상항목/노이즈/FPS/step기록. loss/보상/성공정의와알고리즘은변경하지않음. 모델·코드·demo업로드없음. 인증없으면offline,chunk별run같은stagegroup.
- 환경 패키지 덮어쓰지 않고 .runtime/wandb에wandb0.30.0설치(기존 site-packages보다 후순위경로). 현재W&B미로그인,온라인대시보드검증은아직안됨.

## 2026-10-07 Human 최우선: 사고 방지, 로봇 항상 가장 느린 속도
- Human: 2주 안에 실기pick-and-place성공이목표,RL자율파지전략 발견 여부는 당장 우선순위가 아님. 성공률보다사고방지를최우선,로봇속도항상가장느리게학습설계. 접근/들기/이동/놓기/ready/그리퍼모두적용. AGENTS/CONTEXT에영속기록.
- Codex 코드리뷰: slow설정max_tcp_speed=.05m/s,max_rot_speed_deg=10,gripperjointvelocity=.02m/s,scale=.0025,windup=.01,episode20/30초,접시speed<.03m/s5초조건,over_lift/hold_still/held_speed/held_accel추가됨. 이는저속명령설계이며실기최저값검증은아직없음.
- 잔여검증: TCP속도cap은hold목표증분의노름제한. IK는실제TCP와누적목표차이를보정하며jointtarget속도하드제한없음; armactuatorvelocity2/3.9rad/s이전값유지. 실제TCP/관절속도·가속도상한검증이없으면저속보장이나실기승인으로해석금지. 핵심학습파일은Claude작업중이라이번리뷰에서수정/중복학습시작안함.

## 2026-10-07 v11 "slow": 속도 최우선 재설계 (Human 승인, Claude) — 보상·성공 조건 변경
- Human 원칙(최우선): **로봇 속도는 무조건 가장 느리게. 성공/실패보다 속도가 우선.** 제안한 재학습 항목은 모두 승인. 3단계는 취소하고, 이 수정을 반영한 2단계부터 다시 진행.
- 배경: v10 stage 2 정책(`m1013_2f85_rim/2026-10-06_21-06-08_stage2/model_4996.pt`, 결정적 98.3%)은 속도 제한이 사실상 없는 상태(약 0.9 m/s)로 학습됐다. 접시를 1초 만에 13 cm 들고, 유지 중 위아래로 39 cm 흔들었다(최대 45 cm). 추론할 때만 속도를 낮추면 0.3 m/s에서 성공 0.6%까지 떨어졌다(`rl/logs/speedcap_test.log`). 그래서 속도 제한은 학습 env 안에 넣는다.
- 변경 (`rl/config.yaml`, `rl/plate_env.py`):
  1. **속도 상한(env 내부)**: TCP 목표 이동 ≤ **5 cm/s**(`action.max_tcp_speed`, 승인 범위 0.1~0.15 m/s보다 느리게 잡음: 속도 최우선), 회전 ≤ **10°/s**(`max_rot_speed_deg`, 방향 고정 보정 포함), 그리퍼 손가락 **20 mm/s**(실제 2F-85 최저 속도). 행동 scale 0.03 → 0.0025(출력 1.0 = 7.5 cm/s, 이후 상한 적용), hold_windup 3 → 1 cm. 방향 고정의 gain_clip은 회전 상한이 대신한다.
  2. **들기 목표 대역**: lift와 tcp_rise 보상의 포화점을 8·10 cm에서 `lift_target` 10 cm로 바꾼다. 새 항목 `over_lift`(−3): 10 + 2 cm를 넘는 들림 5 cm당 −1, 최대 −2.
  3. **유지 성공 조건**: 기존 조건(held, 5 cm 이상, 기울기 25° 미만, 5초)에 **접시 속도 < 3 cm/s**를 추가. 속도는 접시 위치 변화량으로 계산한다. PhysX가 주는 속도는 그립에 정지해 있는 접시도 약 30 mm/s로 보고하기 때문이다(scripted에서 확인).
  4. **파지 후 동작 벌점**: `hold_still`(−2, 목표 대역 이상에서 접시 속도/3 cm/s, 최대 3), `held_speed`(−1, held × (TCP 속도/5 cm/s)²), `held_accel`(−0.5, held × (TCP 가속 m/s²)², 최대 4).
  5. 에피소드 길이: 2단계 14 → 20초, 3단계 24 → 30초(느려진 만큼). 데모 홈→ready MoveJ 기본 4 → 15초.
- 검증: `eval.py scripted --lift_m 0.10`(8 env, 학습 없음)에서 새 상한으로 **8/8 성공**(유지 321~326 스텝), 접촉 0, 실드 차단 0. 느린 속도에서도 물리적으로 가능함을 확인. `train.py chunk` 3 iteration smoke 통과.
- 학습: 새 실험 `m1013_2f85_rim_slow`. v10 `model_4996`에서 미세조정을 먼저 하고, 정체하면 처음부터 학습한다. 2단계 통과 시 멈추고 GUI로 보고한다.
- 영향: `IL/collect.py`는 `action.scale`로 명령을 계산하므로(0.3 클립) 새 scale에서는 데모 속도가 달라진다. 기존 `IL/data`는 v10 동역학(빠름) 기준이므로 IL을 다시 쓰려면 재수집이 필요하다(Codex 확인 요망).
- W&B: Codex의 tracking 코드 사용. 로그인(~/.netrc, 계정 im71721)은 확인했지만 계정에 기본 entity가 없어서 online 초기화가 "entity not specified"로 실패한다. 개인 entity `im71721`은 upsert 거부, `hails` 팀은 권한 거부. `train.py`가 online 실패 시 offline으로 자동 전환하도록 수정했다(학습은 중단되지 않음). Human이 wandb.ai 설정에서 기본 entity(팀)를 정하거나 `config.yaml tracking.entity`에 쓸 이름을 알려주면 offline run을 `wandb sync`로 올린다.
- 작성자: Claude

## 2026-10-07 Human 확인: 최종 place는 dish rack 슬롯에 세워 거치
- 고정D455 사진 calibration/data/tray_capture/20261007_100957_fixed_color.png의접시거치모습이최종목표임을Human확인. 저속파지·들기후운반/접시세우기/슬롯정렬/삽입/트레이지지후release가필요. 현재Stage3상자위place정의는최종실증과다르므로향후수정필요. 현재Stage2학습은재사용가능,이번턴학습코드/실기동작변경없음. 슬롯치수/각도/그리퍼간섭여유는미측정이며사진만으로확정하지않음.

## 2026-10-07 최종 목표 확정: 접시를 트레이(plate arrange tray)에 세워 넣기 + 2주 시연 (Human, Claude 정리)
- Human: 최종 목적은 고정 D455 사진 속 식기 트레이에 접시를 pick → place하는 것(트레이 안에 접시가 세워져 있음). 시연이 2주 뒤라 급함. 2단계까지는 pick, 3단계는 이동+place를 나누든 합치든 자유.
- 현재 3단계(평평한 상자 위에 눕혀 내려놓기, `target_box`)는 최종 목표와 다르다. 접시를 쥔 채 그리퍼를 약 90° 돌려 세우고 철사 슬롯에 넣는 단계가 필요하다. 제안: 3a = 들기·세우기·슬롯 입구 자세까지 이동, 3b = 슬롯 안으로 내리고 놓고 후퇴.
- 제안(2트랙): A 시연용 = FP 접시 pose + 충돌 검사 경로 계획(`calibration/pipeline.py`) + 스크립트 파지, 전부 최저 속도, 사람 `go` 승인 이동(학습 없는 스크립트 파지는 sim에서 5 cm/s 상한으로 8/8 성공). B 연구용 = RL 2단계 파지 완료 후 3a/3b를 RL로 시도하고 A와 성공률 비교. 실로봇에 RL 정책을 올리려면 관측 연결·안전 검증이 필요해 2주 시연에는 위험하다.
- 필요한 입력(Human): ① 트레이의 로봇 기준 위치 ② 넣을 슬롯 번호·간격·폭·철사 굵기 ③ 접시가 서야 하는 방향 ④ 시연 때 접시 위치가 고정인지. 입력 전에 할 일: 슬롯 입구 자세·세운 자세의 IK 도달 가능성과 프레임 충돌 여유 계산.
- 작성자: Claude

## 2026-10-07 Codex 제안: Stage3 dish rack 삽입 보상/이동 설계(미구현)
- Stage2저속파지/들기유지. Stage3는운반→접시세우기/슬롯앞정렬→삽입→트레이지지확인/release→후퇴로구분,슬롯좌표계에서접시pose/진입깊이평가(TCP만으로판정금지). 안전속도는hard제어계층,접촉허용부위및힘상한은실측/장비확인후정의.
- 보상은phase별gate로충돌하는목표분리,슬롯횡방향오차/접시각도개선,정렬후에만삽입깊이진전,지지후에만release보상,그리퍼열림+접시의목표pose/저속유지+후퇴를최종성공으로검증. 지나친중간정체보상/단계반복보너스회피.
- 중요한코드정리대상: 현재Stage3도Stage2보상상속/ready손목방향lock적용이라접시세우기와충돌할수있음. Stage3회전시phase별목표orientation전환및수평보상/파지위치유도보상gate필요. 현재rl/핵심코드수정없음.
- 2주실증에맞춰기본저속경유점경로를먼저물리검증하고필요시정렬/삽입구간만RL작은보정학습제안. CAD대신실측rack근사충돌형상가능하나치수/지지접촉/그리퍼여유미측정.

## 2026-10-07 트레이 측정 1차 (Human 입력 + 손목 카메라 깊이, Claude)
- Human 측정(줄자/눈대중): 접시 넣을 칸 폭 **20 mm(정확)**, 철사 굵기 약 0.5 mm(눈대중, 핀 굵기는 더 두꺼울 수 있음 — 재측정 필요), 철사(칸) 높이 약 60 mm, 트레이 바닥이 테이블에서 약 50 mm 떠 있음. CAD/제조사 정보 없음 → 단순 파라미터 모델로 진행.
- 손목 카메라 캡처 `calibration/data/tray_capture/20261007_103356_338122303684_*`(color/depth/K), 촬영 자세 posx (651.53, -97.41, 638.48, 164.86, -126.51, 94.09). 손목 hand-eye(`20261006_111318_clean`) + posx로 깊이를 베이스 좌표로 변환. 점검: 테이블 평면이 z = -0.025 m로 나와 테이블 윗면(-0.020)과 5 mm 이내 일치.
- 결과: 트레이 바깥 프레임 x 0.95~1.49 m(폭 약 54 cm), y -0.55~-0.03 m(깊이 약 52 cm), 프레임은 베이스 축과 거의 평행. 측면 레일 윗면 z 약 0.15 m(테이블 위 약 17 cm). 고정 D455 사진에서 구한 발 위치 추정(x 0.93~1.49)과 일치. 내부의 얇은 핀/철사는 비스듬한 깊이 영상으로 해상되지 않아 칸 위치는 미확정. → 칸 바로 위에서 수직으로 내려다보는 손목 사진 + 어느 칸인지 지정이 필요.
- 현재 sim `target_box`(1.10, -0.45)는 트레이 뒤쪽 레일(y -0.55)에 가깝다. 3단계 재정의 때 교체.
- 작성자: Claude

## 2026-10-07 트레이 칸 폭 정정 (Human, Claude)
- Human: 빨간 원으로 표시한 칸(핀 달린 칸막이 철사와 이웃 철사 사이)의 철사 간 간격은 **약 95 mm**(눈대중). 앞서 말한 "폭 20 mm"는 이 간격이 아니라 다른 값(칸막이 옆 짧은 핀 간격일 가능성)으로 확인 필요. 접시 두께(그릇 깊이 25 mm)에 비해 칸이 넓어 위치 허용 오차는 한쪽 약 35 mm로, 카메라 오차(5~10 mm)보다 크다 → 정밀 삽입 문제가 아니라 접시가 쓰러지지 않게 기대어 세우는 문제.
- 미확정: 접시가 칸 안에서 서는 자세(수직인지 기댄 각도인지), 기대는 철사/핀, 바닥 철사 높이. Human이 손으로 접시를 그 칸에 세워 보고 자세를 알려주기로 요청함(사진 포함).
- 작성자: Claude

## 2026-10-07 접시 최종 자세 정정 (Human 측면 사진, Claude)
- Human 측면 사진: 접시는 **거의 수직**으로 서서 아랫부분이 2 cm 간격의 인접한 철사 핀 두 개 사이에 끼어 있다(핀은 위로 갈수록 벌어짐). 접시 윗부분은 트레이 윗테두리보다 높다.
- 앞 항목의 "접시 평면 기울기 약 22°"(손목 카메라 깊이 평면 피팅)는 **폐기**: 반짝이는 접시의 깊이가 거칠고 테이블 점도 부족했다. 수직 설명이 사진과 맞다.
- 방향(Human): 칸막이 철사 간격 95 mm = x, 핀 간격 20 mm = y. 접시 면은 y에 수직(x–z 평면). 로봇 동작: 림 파지(툴 +x, 손가락 닫힘 축 z) → 들기 → 도구 축 x 중심 약 90° 손목 회전 → 접시가 x–z 평면에 서고 손가락 닫힘 축이 y → 핀 사이로 하강. 허용 오차는 y 방향 약 ±8 mm(핀 간격 20 mm − 접시 벽 두께), x·z는 여유. 카메라 보정 오차(5~10 mm)와 같은 크기라서 목표 자세는 FoundationPose로 읽은 접시 자세(접시를 칸에 놓은 상태)로 정한다.
- 미확인: 측면 사진의 촬영 방향, 핀 높이(Human 눈대중 60 mm), 접시 아랫면이 닿는 바닥 철사 높이, 도구 축 90° 회전 후 접시 하단 도달 높이와 프레임 충돌 여유(계산 예정).
- 작성자: Claude

## 2026-10-07 실제 그리퍼 제어와 파지력 기준 (Human 확정, Claude)
- 연결: Robotiq 2F-85 전원은 두산 툴 플랜지, 명령은 **PC의 USB-RS485(FTDI, `/dev/ttyUSB0`, Modbus RTU, slave 9, 115200 8N1)**. 펜던트로는 조그 불가.
- 도구: `calibration/gripper.py status|activate|open|close|goto --pos N`(시스템 `/usr/bin/python3` + pyserial). 설정은 `calibration/config.yaml gripper:`. 속도는 코드에 최저(rSP 0, 약 20 mm/s)로 고정해 옵션이 없고, 활성화 동작의 속도만 펌웨어 고정이다. 움직이는 명령은 실행 전에 확인을 받는다. 상태 읽기는 agent가 하고, 동작 명령은 Human이 실행한다.
- **파지력 기준: 64/255(최대의 1/4, 약 60 N).** Human이 실제 접시로 확인했고 미끄러짐 없음. 별도 지시 전까지 실기 파지의 표준값.
- 참고: 상태의 fault 0x09(1초 이상 통신 없음)는 한 번씩 보내는 명령 사이에 항상 뜨는 경미한 표시로, 동작을 막지 않는다. 포트에서 I/O error가 나면 FTDI 어댑터를 다시 꽂는다(10/7 11:38 사례).
- RL sim과의 차이: sim 그리퍼는 위치 제어(stiffness 2000, effort 235 N 상한)라 실제 힘 제어(64/255)와 다르다. 3단계 sim에서 파지력 반영 여부는 추후 검토.
- 다음: Human이 펜던트와 gripper.py로 트레이 칸에 접시를 끼우는 수동 시연을 한다. 잡기, 칸 위 진입, 칸 안 놓기 자세에서 두 D455 캡처(`camera_tools.py preview --all`, `s` → `calibration/data/tray_capture/`)와 posx·관절값을 기록해 3단계 목표 자세로 쓴다.
- 작성자: Claude

## 2026-10-07 v11 slow 학습 안정화 (Human 승인, Claude) — 보상·PPO·체크포인트 선택 변경
- 관찰(scratch 10:02~13:10, 2,850 it, `rl/logs/m1013_2f85_rim_slow/train_s2_*.log`): 성공이 오르면(1,250~1,300 it에서 60~70%) 곧 무너진다. 무너질 때마다 접시 놓침이 43~76%로 늘고, 가치함수 손실이 19~33까지 급등한다. 2,450 it에서 36%까지 다시 올랐다가 2,800 it에서 0%. 500 it마다 0 근처로 떨어지는 그래프는 chunk 재시작 직후 미완료 에피소드 때문에 생기는 착시다.
- 원인 판단: ① 성공 보너스 +200 한 번(나머지는 초당 수 점)이라 가치함수가 흔들리고, 그 값으로 큰 정책 갱신이 일어난다 ② plate_lost가 벌점 없이 종료만 된다 ③ 통과 평가가 "최근 50 it 평균 ≥ 0.93"일 때만 돌아서, 정점(59%)을 한 번도 평가하지 못했다.
- 변경:
  1. `success` 6000 → 900(×dt = +30)
  2. 신규 `hold_progress` +3/s(stable_lift 동안)
  3. 신규 `dropped` −1500(×dt = −50, plate_lost 종료 시)
  4. PPO `desired_kl` 0.01 → 0.005
  5. curriculum: chunk마다 저장된 체크포인트 중 50 it 학습 성공이 가장 높은 것을 고르고, `proxy_gate` 0.5 이상이면 그 체크포인트를 결정적 평가한다. 마지막 체크포인트가 그보다 0.2 이상 나쁘면 다음 chunk는 최고 체크포인트에서 재개한다(`rollback_drop`).
- 13:13 재시작: `--init 2026-10-07_11-08-10_stage2/model_1300.pt`(학습 성공 0.59) `--eval_first`. 이전 run은 그대로 보관.
- 작성자: Claude

## 2026-10-07 Codex 저속 IL 별도수집 설계(Human 요청)
- seed701,50성공목표,8env,max20batches,스냅샷실행 IL/data/slow_codex_20261007/source_snapshot/collect_codex.py. sharedRL/Claudecollector 변경없음. expert목표를IKbase좌표에서누적목표오차로계산하여목표가앞서달리는현상회피,25.5~34.5mm/s로이동. phase1/4/3/5/7초,TCP상승90~110mm,현재20초안에5초안정유지판정.
- 실제TCP≤.05m/s(PhysX속도와pose차분모두),TCP회전≤10deg/s,손가락관절≤.02m/s를초과하면제외. probe에서gripper실제.0213m/sovershoot감지하여수집환경만actuator.velocity_limit_sim=.018로낮춤(override명시),기존학습물성/힘/실드유지. 팔관절최대속도와TCP가속도는기록하며실기최저속도보장으로해석하지않음.
- 성공/안전선별및speedmetric추가는수집데이터정의이며PPO성공/보상계산수정없음. held와plate속도는고정snapshot의현재env판정사용.

## 2026-10-07 IL → RL 전환: 상태 기반 전문가 데모 + BC + critic warm-up + PPO (Human 지시, Claude)
- Human: Codex와 함께 저속 데모를 모아 IL+RL로 학습하고, Human 교육 후 GUI로 확인.
- 발견 1 (데모 설계): 기존 `IL/collect.py`는 시간으로 단계를 바꿔서(5초에 닫기 등) 림에 도착한 같은 상태에 "열고 대기"와 "닫기" 라벨이 섞인다. 4개 데모 BC는 64 env 모두 시간초과였다. → 단계를 **env별 상태 기반**으로 바꿨다: TCP가 림 목표 3 mm 이내면 닫기, held이고 손가락이 5스텝 멈추면 들기(lift 10 cm ±1 cm), 이후 같은 제어기로 유지. 속도는 `max_tcp_speed` × 0.7~1.0, 기록 17초. 8개 smoke 데모 8/8 성공.
- 발견 2 (BC copycat): 관측 마지막 7차원(이전 행동)을 보면 BC가 전문가의 직전 명령을 그대로 따라 해서, 실제 실행에서 림 15 mm까지 갔다가 물러나기를 반복하고 닫지 않았다. 64 env 결정적 평가에서 충돌 100%. → BC 학습 때 이 7차원을 0으로 두고 첫 층의 해당 가중치를 0으로 설정(`bc.ignore_obs [36, 43]`). 8개 데모 BC가 16/16 접근 → 닫기(약 3초) → 약 7.5 cm 들기 → 유지(trace).
- Codex 데이터(`IL/data_slow_codex` → `IL/data/slow_codex_20261007`, seed 701, 50개, 검증 통과)는 고정 시간 단계(1/4/3/5/7초)이고 수집 때만 그리퍼 18 mm/s로 바꿔서, 이번 BC에서 제외했다(`bc.data`). 비교 실험용으로 보관.
- 코드:
  - `rl/train.py bc`: 데모 → PPO와 같은 actor MLP를 MSE로 학습(AdamW 1e-3, 4000 step, 에피소드 단위 train/val), critic은 새로 초기화, 행동 노이즈 0.2. rsl_rl 체크포인트로 저장(`<exp>/<date>_stage2_bc/model_0.pt`).
  - `chunk --freeze_actor`(critic warm-up).
  - `curriculum --critic_warmup N --experiment`.
  - `eval.py trace` 길이를 설정 episode_s로 고침(전에는 3600초를 돌아서 "멈춘 것처럼" 보였다).
- 실행(13:45, 자동 파이프라인 `rl/logs/il_rl_pipeline.log`): 상태 기반 데모 50개(seed 456, `IL/data_slow_claude`) → verify → BC → 순수 RL 기준선(`m1013_2f85_rim_slow`, model_1300부터 재개 중) 중지 → config 실험명 `m1013_2f85_il_rl` → GUI `eval.py watch`(최신 체크포인트 자동 반영) → curriculum(BC 결정적 평가 → critic warm-up 100 it → PPO chunk, 2단계 게이트에서 정지).
- 순수 RL 기준선 결과(비교용): 최고 체크포인트 `m1013_2f85_rim_slow/2026-10-07_11-08-10_stage2/model_1300.pt` 결정적 성공 55.7%, 놓침 28.0%, 충돌 3.1%.
- 작성자: Claude

## 2026-10-07 IL → RL 결과와 residual RL 전환 (Claude)
- 결정적 평가(256 env, 별도 표기 없으면):

| 정책 | 성공 | 충돌 | 비고 |
|---|---|---|---|
| 순수 RL 최고(`m1013_2f85_rim_slow/.../model_1300`) | 55.7% (2048) | 3.1% | 놓침 28% |
| BC, 시간 기반 데모 4개 | 0% | 0% | 전부 시간초과(닫지 않음) |
| BC, 상태 기반 50개 | 3% | 97% | |
| BC, + 이전 행동 가림 | 33% | 3.5% | 2000 step |
| BC, 상태 기반 + DART 100개(0.2 노이즈), 이전 행동 가림 | 0%(3 seed) | 100% | |
| **BC, 상태 기반 + DART, 이전 행동 + 관절 속도 가림** | **85 / 59 / 67%** (seed 1/2/3) | 15 / 41 / 33% | seed 1 = 2048 env에서 83% |

- 충돌 진단(`scratchpad coll_diag`): 열린 그리퍼가 림 목표를 35~48 mm 지나쳐 gripper_base가 접시에 닿는다. 정책이 "움직이는 중이면 계속 움직인다"를 이전 행동과 관절 속도에서 베껴서 생기는 copycat 문제이므로, `bc.ignore_obs [[8, 16], [36, 43]]`.
- BC → PPO 직접 미세조정 실패: critic warm-up(100 it, 학습 중 성공 86.5%) 뒤 PPO 약 50 it 만에 성공 0%가 되었고, 전부 시간초과(잡기를 시도하지 않는 정책으로 붕괴). 기록은 `m1013_2f85_il_rl`.
- **전환: residual RL.** BC actor를 고정하고 PPO는 보정만 학습한다: 실행 행동 = BC(obs) + 0.3 × PPO 출력(`config.yaml residual`, `rl_utils.make_vecenv`). PPO 출력층을 0으로 초기화해 시작 정책 = BC. 행동 노이즈 0.3(실행 기준 0.09). gate/watch/trace/demo도 같은 wrapper로 BC를 더한다. smoke 2 it 후 256 env 성공 71%.
- 14:27 실행: `train.py curriculum --stage 2 --scratch --critic_warmup 100`(실험 `m1013_2f85_il_residual`), GUI `eval.py watch --stage 2 --num_envs 8`이 최신 체크포인트를 따라간다.
- 위험: residual 크기 0.3이 BC 오류(지나침 35~48 mm)를 고치기에 충분한지는 미확인. 부족하면 0.5로 올린다.
- 작성자: Claude

## 2026-10-07 붕괴 원인 확정: 성공 종료를 피하는 보상 해킹 → residual 전용 희소 보상 (Claude)
- 증상(세 번 반복): 성공률이 오르면(순수 RL 60~70%, BC → PPO 86%, residual 98%) 곧 0%로 떨어지고, 충돌·놓침은 낮은 채 **시간초과**가 된다(에피소드 길이 600).
- 원인: 접시를 들고 있는 동안 shaping 보상이 초당 약 10점(lift, tcp_rise, level_hold, hold_progress 등) 들어오는데, 성공하면 에피소드가 끝나 이 수입이 끊긴다. 오늘 아침 성공 보너스를 +200 → +30으로 줄이고 hold_progress를 추가해서, "성공 직전에 머물며 점수를 모으는" 쪽이 이득이 됐다. 오늘 아침 내 변경이 원인을 키웠다.
- 조치: residual 모드에서는 `rewards.residual`만 쓴다. 구성: success +100(3000×dt), hold_progress +1/s, 벌점(충돌·놓침 −50, over_lift·hold_still·held_speed·held_accel·shield·knocked·action_rate·joint_vel). 접근·자세·들기 shaping은 BC가 담당하므로 제거했다. 들고 있는 동안의 양의 보상은 +1/s뿐이라 성공(+100)이 항상 이득이다.
- 순수 RL용 `rewards.stage2`도 같은 문제가 있다. 순수 RL을 다시 쓸 때는 success를 키우거나 shaping을 성공 시 0으로 만들어야 한다(미적용).
- 14:50 재시작: `curriculum --stage 2 --scratch --critic_warmup 100`, 실험 `m1013_2f85_il_residual`. 이전 shaping residual run은 `*_il_residual_shaped_discard`에 보관.
- 작성자: Claude

## 2026-10-07 진짜 원인: 할인율 gamma 0.98이 느린 과제에 너무 짧음 (Claude)
- 희소 보상 residual도 같은 붕괴(학습 중 성공 98% @144 it → 4% @258 it). 결정적 평가: model_100 67%(충돌 32%) → model_200 **11%**(시간초과 81%, TCP 속도 중앙값 0.9 cm/s = 거의 안 움직임).
- 원인: γ = 0.98이면 유효 시야가 약 50스텝(1.7초)이다. 5 cm/s 상한에서 성공은 시작 후 8~13초(240~400스텝)에 오므로 시작 시점에서 성공 보상의 가치가 0.98^300 ≈ 0.2%에 불과하다. 반면 충돌(약 3초)은 0.16배로 반영된다. 그래서 "움직이지 않기"가 최적이 된다. 빠른 v10(1초 안에 들기)에서는 문제가 아니었고, **속도를 낮춰 과제가 길어지면서 γ가 맞지 않게 됐다.** 오늘 순수 RL의 반복 붕괴도 같은 원인으로 본다(앞 항목의 보상 해킹 해석은 부분적 기여로 정정).
- 조치: `ppo.gamma` 0.98 → **0.998**(400스텝 후에도 0.45배), residual에서 entropy_coef 0(노이즈 0.30 → 0.36 증가 방지, `residual.ppo` override), critic warm-up 150 it.
- 15:12 재시작(실험 `m1013_2f85_il_residual`), 이전 run은 `*_gamma098_discard`.
- 작성자: Claude

## 2026-10-07 15:40 IL+RL(residual, γ 0.998) 결과: 2단계 기준 통과 수준 (Claude)
- 학습 중 성공(노이즈 포함): critic warm-up 후 PPO 250 it 동안 85~99.7% 유지, 붕괴 없음(이전 세 번은 모두 붕괴). 노이즈 0.30 → 0.25 감소.
- 결정적 평가(512 env):

| 체크포인트 | 성공 | 충돌 | 놓침 | TCP 속도 p50 / p95 | 유지 중 접시 높이 변동 | 최대 들림 |
|---|---|---|---|---|---|---|
| model_300 | **99.4%** | 0 | 0 | 1.7 / 5.0 cm/s | 60 mm | 153 mm |
| model_400 | **98.6%** | 0 | 0 | 1.4 / 5.0 cm/s | **6.6 mm** | 57 mm |

- 비교: 순수 RL 최고 55.7%(충돌 3%, 놓침 28%), BC 단독 83%(충돌 16%).
- 주의: model_400은 접시를 약 5.7 cm만 든다(성공 기준 5 cm 바로 위). residual 보상에 들기 목표 대역이 없어서다. 3단계(트레이 이동)에서는 목표 높이를 보상에 다시 넣어야 할 수 있다.
- curriculum은 chunk가 끝나면 최고 체크포인트를 2048 env로 평가하고, 통과하면 멈춘다. 이후 Human GUI 확인 전까지 3단계는 시작하지 않는다.
- 작성자: Claude

## 2026-10-07 Codex: Sim-to-Real PPO pick-and-place 논문 검토 — 적용 제안만
- Human 제공 DOI 10.1007/s12541-026-01595-5, 「Sim-to-Real Transfer of Deep Reinforcement Learning for Robotic Pick-and-Place Tasks」 검토. Sec3.2/3.3은 PPO로 reaching만 학습하고 그리퍼는 독립 heuristic으로 닫기/열기. 접시 림 파지·회전·슬롯 삽입의 접촉 학습을 검증한 논문은 아니다.
- 참고할 요소: Isaac Sim 내 정책으로 궤적 생성 후 실기 관절 제어 실행 분리(Sec3.3), 먼 거리/근거리 위치 보상 및 action-rate/joint-velocity penalty의 점진 강화(Sec3.2), 인식·파지·마찰/접촉 원인별 실패 분석(Sec4.3).
- 적용 의견: FoundationPose 6D pose 유지, 검증된 저속 경유점과 상태 기반 닫기/지지 확인 후 열기, 정렬/삽입의 작은 residual 보정. 이는 Codex 제안이며 논문이 트레이 삽입을 검증한 결과가 아니다. 항상 최저 속도 및 실행 계층 hard limit 유지, PPO clip/벌점은 물리 안전 보장이 아니다.
- 해석 한계: 평균 reaching 위치 오차 약20mm는 우리 슬롯 정렬에 충분한 근거가 아니다. 각 실기 조건15회(Table5/6), 복잡 다물체9/15, 기존 IK 기준선 비교는 미래 과제(Sec5). 논문 gamma .99를 우리 저속 긴 에피소드에 그대로 복사하지 않는다. 공유 최신 기록의 BC 고정+residual PPO와도 분리 제어 방향은 양립한다. 코드/보상/학습 설정 변경 없음.

## 2026-10-07 17:22 공정 비교: 순수 RL을 γ 0.998로 재실행 (Human 요청, Claude)
- 목적: BC 유무 비교에서 순수 RL은 γ 0.98, IL+RL은 γ 0.998이라 BC 효과와 γ 효과가 섞여 있었다. 순수 RL을 같은 γ 0.998로 다시 돌린다.
- 설정: 실험 `m1013_2f85_pure_g998`, scratch, 4096 env, 최대 2000 it(500 it chunk), 같은 게이트·최고 체크포인트 선택·rollback. 보상은 순수 RL용 `rewards.stage2`(shaping 포함).
- 변경 1: `rewards.stage2.success` 900 → **15000**(×dt = +500). γ 0.998에서는 들고 있는 동안의 shaping(약 17/s)이 계속될 때 약 280점 가치가 있어, 성공이 그보다 커야 성공 직전에 머무는 보상 해킹을 피한다.
- 변경 2: **`residual.enabled`를 이 실행 동안만 false**로 바꿨다. 통과한 IL+RL GUI는 시작 시 설정을 읽어서 영향이 없다. 비교가 끝나면 true로 되돌린다.
- 비교 시점: IL+RL은 critic warm-up 150 + PPO 500 = 650 it(+BC). 순수 RL은 650 it과 2000 it 두 시점의 최고 체크포인트를 결정적 평가로 보고한다.
- 작성자: Claude

## 2026-10-07 실증 (a): 실제 접시를 학습 위치에 두고, sim rollout을 실로봇에서 재생 (Human 결정, Claude)
- Human: (a) 실제 접시와 받침(114.6 mm)을 sim 학습 위치 (1.10, 0.00)에 둔다. 범위는 지금 되는 정도까지(접근 → 잡기 → 들기). 학습 중이던 순수 RL 비교 run과 GUI는 Human 지시로 종료(비교 결과 없음). `residual.enabled`는 true로 복구.
- 방식(열린 루프): FoundationPose 접시 pose → sim에서 같은 위치로 BC+RL 정책 rollout(16 env) → 관절 경로를 1.5° 간격 waypoint로 → 기존 mover(사람이 waypoint마다 `go`, ≤ 2°/s, ≤ 45°/move, 30분 유효)로 재생. 실로봇에서 정책이 실시간으로 반응하지는 않는다.
- 도구:
  - `calibration/camera_tools.py locate`: FP × T_base_cam으로 접시 바닥 중심과 목표의 차이를 실시간 표시. FP mesh 원점은 바닥 중심에서 (27.4, 2.9, 5.9) mm 떨어져 있어 보정한다.
  - `rl/eval.py export`: plan을 `calibration/data/rl_plans/<시각>/plan.json`에 쓴다. legs: to_ready, grasp(이후 `gripper.py close`), lift, lower(이후 `gripper.py open`), retreat, to_home.
  - `ros2_nodes.py mover --leg`가 위 이름을 받도록 일반화했다. 검사 로직은 그대로이고, 회귀 시험 20/20 통과.
- 검증(sim, 접시 (1.10, 0.00)): 성공 16/16, 충돌 0. 프레임 여유 333 mm 이상, 테이블 82 mm 이상. to_ready 4 waypoint(최대 28.9°), grasp 6, lift 13(각 최대 1.6°). plan 여유 기준은 shield 50 mm(파이프라인 큐브 plan은 100 mm). 프레임과는 330 mm 이상 떨어져 있어 실질적인 차이는 없다.
- agent가 mover 검사 모드를 실행하려 하자 권한 판단에서 거부됐다. 실로봇 도구 실행(검사 포함)은 Human이 한다.
- 위험: 열린 루프라서 접시 위치 오차(FP와 보정 약 5~10 mm)가 그대로 파지 오차가 된다(림 진입 15 mm). sim 접시 질량(0.3 kg 가정)·마찰·그리퍼 동역학은 실제와 다르다. 먼저 접시 없이 모든 leg를 공중 재생해 볼 것을 권장한다.
- 작성자: Claude

## 2026-10-07 18:10 실증 1차 결과와 자동화 (`pipeline.py --task plate`) (Human, Claude)
- 실증 1차(17:49 plan, 접시 (1.1013, −0.0042)): to_ready 정상(도달 오차 0.01°). grasp 후 닫았을 때 그리퍼 상태가 "no object"(228/255, 끝까지 닫힘), 즉 테두리를 놓쳤다. Human이 그리퍼를 열고 접시를 손으로 옮긴 뒤 다시 닫아 잡았고, lift → lower → open → retreat → to_home까지 완료. Human 평가: "나쁘지 않다".
- 놓친 원인은 아직 정량화하지 못했다. lower 후 접시 위치를 FP로 재니 (1.1085, −0.0561), 기울기 5.4~6°(받침 가장자리에 걸친 것으로 추정)였다. plan 위치 대비 약 (+7, −52) mm인데, Human이 옮긴 양과 받침 위 자세가 섞여 있어 보정값으로 쓰기에는 불확실하다. → `plate_rl.offset_m`은 [0, 0]으로 두고, Human이 옮긴 방향과 거리를 확인한 뒤 정한다.
- Human 요청: 접시 값을 손으로 넣지 않고 FoundationPose로 받아서 같은 시나리오(그리퍼 닫기·열기 포함)를 진행. 구현: `calibration/pipeline.py --task plate [--execute]`(`bash calibration/run.sh pipeline --task plate --execute`).
  1. 실행 시 로봇이 홈 자세(±3°)인지 확인
  2. FoundationPose(`--mesh_file textured.obj`, prompt "red plate") → T_base_cam → 접시 바닥 중심. 학습 범위(±10/10/15 mm, 기울기 5° 이하) 안에서 4회 연속이면 진행, 아니면 옮길 방향을 계속 표시
  3. 인식 종료(GPU 확보) → Isaac `rl/eval.py export`(BC+RL model_400)
  4. gripper open → to_ready → grasp → gripper close → **상태가 "stopped on an object while closing"이 아니면 들지 않고** open → retreat → to_home 후 정지
  5. lift → Human Enter → lower → open → retreat → to_home
  - 각 leg는 mover 확인 문구, waypoint마다 `go`, ≤ 2°/s. 그리퍼는 매번 `yes` 확인, 최저 속도, 힘 64.
  - 설정: `calibration/config.yaml plate_rl`(checkpoint, prompt, tolerance, offset_m).
- 검증(로봇 없이): 회귀 시험 20/20. plan 전용 실행에서 카메라 → FP → 범위 확인이 동작했다(접시가 y +56 mm 밖이고 기울어져서 설계대로 정지). conda에서 Isaac export 호출 시 16/16 성공, plan 생성. 로봇이 포함된 전체 흐름(`--execute`)은 아직 미실행(Human이 연결을 끊음).
- 작성자: Claude

## 2026-10-07 19:10 실증 2차 준비: 보정값과 한 번 승인 접근 (Human, Claude)
- 보정: Human 기억 기준으로 1차 실증에서 접시를 로봇 쪽 약 2 cm, 로봇 기준 오른쪽 약 2 cm 옮겨야 잡혔다 → 실제 그리퍼가 sim보다 (−2, −2) cm 쪽을 잡는다 → `plate_rl.offset_m = [0.02, 0.02]`(rollout에는 FP + offset을 쓴다). 배치 확인도 FP + offset 기준이라 실제 접시는 (1.08, −0.02) 근처에 놓는다. 값이 기억에 의존하므로 다음 실행에서 검증한다.
- Human 요청: grasp 전까지 한 번에 이동. `ros2_nodes.py mover --go_per leg`를 추가했다(확인 문구 1회 + go 1회로 leg 전체 실행). 각 waypoint에서 정지하고, 시작/도달 오차·plan 재검사는 매 이동마다 그대로 한다. plan에 `to_grasp`(홈 → ready → 림)와 `from_grasp`(림 → ready → 홈) leg를 추가했다. 파이프라인은 to_grasp와 from_grasp를 leg 단위로, lift와 lower를 waypoint 단위로 실행한다.
- mover 입력 개선(실증 중 두 번 오작동): 이동 중에 미리 친 입력은 버리고(termios flush), `go`가 아닌 입력은 다시 묻는다. 종료는 `q`로만 한다. 움직이는 조건(정확히 `go`)은 바뀌지 않았다.
- 파이프라인 개선: 그리퍼가 비활성이면 확인 후 activate, 배치 안내를 방향 문장으로 표시, 배치 대기 600초, 기울기 기준 8°(평평한 접시에서도 FP 기울기가 약 4°로 측정됨).
- 검증: 회귀 시험 20/20, export에 새 leg 포함(16/16, 여유 354 mm 이상). 실로봇 실행은 Human이 진행.
- 작성자: Claude

## 2026-10-07 19:15 야간: 넓은 접시 범위 재학습 + 잡은 뒤 앞으로 미는 문제 (Human, Claude)
- Human: 학습 위치로 접시를 옮기지 않고 FoundationPose 위치 그대로 가서 잡아야 한다.
- 범위: `geometry.plate_reset_x [-0.06, 0.06]`, `plate_reset_y [-0.08, 0.08]`(이전 ±0.01).
- 발견: 받침 상자가 고정이라 접시만 4 cm 이상 옮기면 받침에서 떨어져(기울기 약 55°) 전문가 성공이 23/64였다. 실제로는 받침째 옮기므로 **받침을 kinematic rigid body로 바꾸고 reset 때 접시 xy를 따라가게** 했다(`box_under_plate` 이벤트). 수정 후 전문가 64/64.
- 실증에서 관찰(Human): 잡고 들어 올린 뒤 앞으로 민다. 실행한 plan(`rl_plans/20261007_185850`)을 FK로 계산하니 lift 중 TCP가 **+x(로봇에서 먼 쪽)로 106 mm** 이동하고 z는 52 mm 올라간다. 닫는 동안에도 +24 mm 밀었다. 전문가는 수직으로만 든다 → residual RL이 배운 동작이다(보상에 잡은 뒤 수평 이동 벌점이 없었다). → `rewards.residual.held_drift` −3: held × (잡은 지점에서의 TCP 수평 거리 / 2 cm, 최대 3).
- 야간 자동 실행(`rl/logs/wide_pipeline.log`, 19:14 시작):
  1. 넓은 범위 데모: 상태 기반 160개(`IL/data_wide_claude`, seed 2001) + DART 0.2 160개(`IL/data_wide_claude_dart`, seed 2002)
  2. BC seed 1~3을 각각 512 env 결정적 평가(`--residual off`)로 비교해 최고를 고른다
  3. `config residual.base`를 그것으로, 실험을 `m1013_2f85_il_residual_wide`로 바꾼다
  4. residual RL curriculum(scratch residual, critic warm-up 150, 최대 3000 it, γ 0.998, 희소 보상 + held_drift), 게이트 통과 시 정지
- 내일 할 일: 통과 체크포인트로 `calibration/config.yaml plate_rl.checkpoint`와 tolerance(학습 범위 안쪽, 예 ±50/±70 mm)를 갱신한다. 그 전까지 실로봇 파이프라인은 좁은 범위 모델과 이전 base 조합만 맞으니, 야간 실행 중에는 `--task plate` 실로봇 사용을 금지한다(rl/config.yaml의 residual.base가 바뀌었기 때문).
- 작성자: Claude

## 2026-10-07 19:22 실증 3차: FoundationPose → BC+RL → 실로봇 파지 **정확히 성공** (Human 확인)
- 실행: `pipeline.py --task plate --execute`, run `calibration/data/real_pipeline/20261007_192223_plate_4acpb8zc`. FP 접시 루트 (1.0754, −0.0427, 0.0938) + offset (0.030, 0.039) → rollout 위치 (1.1054, −0.0037)(학습 범위 안). 그리퍼 닫은 뒤 상태 "stopped on an object while closing"(grasped=True) → lift. Human: "진짜 완전 정확하게 잡았다".
- 기록(`robot_state.jsonl`): ROS2 관절·플랜지 1486 샘플, FP 396 샘플, 이벤트 8개. FP 접시 위치 표준편차: 접근 중 x/y/z 0.4/0.8/1.1 mm(그리퍼가 가까이 와도 안정) → 닫기 전 정렬 확인을 FP로 할 수 있다는 근거. 들어 올린 뒤에는 x 34 mm, z 26 mm로 흔들려 추적이 불안정하다(가림·이동).
- 보정값 (0.030, 0.039)는 두 번의 실증에서 Human 관찰로 찾은 값이다. 원인(카메라 보정, 그리퍼 모델, 림 형상)의 분해는 아직 하지 않았다.
- Human 제안(다음 단계): 닫기 전에 카메라 기준으로 테두리와 손가락이 충분히 겹치는지 보고 잡는다. 계획: FP(접근 끝 1초 평균) + 관절 FK로 TCP-림 정렬(진입 깊이, 높이, 옆 방향)을 계산해 기준 안이면 닫고, 밖이면 작은 IK 보정 이동(사람 승인)을 한 번 한다. 이후 실행 기록이 쌓이면 성공 판정기를 학습한다.
- 남은 문제: lift 중 TCP가 +x로 약 10 cm 밀린다(야간 재학습에 held_drift 벌점 반영 중).
- 작성자: Claude

## 2026-10-07 19:40 실로봇: 구간별 부드러운 한 동작(movesj), 그리퍼만 사람이 선택, 속도 3배 (Human)
- Human: 3차 실증 성공 후 waypoint마다 멈추는 이동을 하나의 부드러운 동작으로. 그리퍼 open/close만 사람이 고르고 나머지는 자동. 속도는 약간만 올려서 "1/10이면 3/10". 비상정지는 Human이 들고 있음.
- 구현:
  - `ros2_nodes.py mover --smooth`: leg 전체를 **MoveSplineJoint(movesj) 한 번**으로 실행(최대 100 waypoint). 시작 전에 시작점 오차 3° 이하·plan 재검증, 끝난 뒤 도달 오차 1° 이하. 실패하면 quick stop, 재시도 없음.
  - `--auto`: go 질문 없이 실행. `--opening_done`(파이프라인에서 확인 문구를 이미 받음)과 함께일 때만 허용.
  - 속도 상한 `VEL_HARD_LIMIT/ACC_HARD_LIMIT` 2 → **6 deg/s(·s⁻²)**. mover 기본값은 2 그대로이고, 파이프라인만 `plate_rl.vel_deg_s: 6.0`.
  - `pipeline.py --task plate`: 시작 확인 문구 1회 → 그리퍼 선택(c/o/s) → to_grasp(spline) → 그리퍼 선택 → 잡힘(object 감지)이면 lift(spline), 3초 유지, lower(spline) → 그리퍼 선택 → from_grasp(spline). 안 잡혔으면 그리퍼 선택 후 from_grasp 하고 정지.
  - 기록: `robot_state.jsonl`에 ROS2 관절·플랜지(20 Hz), FP 접시 pose(≤10 Hz), leg 시작(속도, smooth)/끝, 그리퍼 선택·상태·잡힘 여부, plan과 FP pose·offset.
- 위험: spline은 waypoint 사이를 직선이 아닌 곡선으로 지난다. waypoint 간격이 촘촘하고(≤1.6°, to_ready만 28.9° 공선) 프레임 여유가 350 mm 이상이라 영향은 작다고 판단하지만, 실측은 하지 않았다. 속도 3배로 정지 거리가 길어진다.
- 검증: 회귀 시험 21/21(신규: smooth leg는 movesj 한 번만 보내고, 시작점에서 4° 벗어나면 거부하고 아무것도 보내지 않는다). 실로봇 smooth 실행은 아직 하지 않았다.
- 작성자: Claude

## 2026-10-07 20:00 Codex 검토 요청: 넓은 접시 범위 BC가 100% 충돌 (원인 미확정, Claude)
Human 지시: Claude와 Human이 실증하는 동안 Codex가 이 문제의 원인과 학습 방향을 검토한다. 결론은 DECISIONS에 반박과 근거를 포함해 남겨 주세요(Critic 역할 환영).

**목표**: 실제 접시를 학습 위치에 맞추지 않고, FoundationPose가 본 위치(학습 위치 기준 x ±6 cm, y ±8 cm)에서 바로 잡는 BC+RL 정책.

**잘 되는 것(비교 기준)**:
- 좁은 범위(±1 cm): 상태 기반 데모 50 + DART 100 → BC(관절 속도·이전 행동 입력 가림, `bc.ignore_obs [[8,16],[36,43]]`) 결정적 85% → residual RL(γ 0.998, 희소 보상) 97.9%. 실로봇 실증 성공(보정 offset (0.030, 0.039)).
- 넓은 범위 데모 수집 자체는 정상: 받침을 kinematic으로 바꿔 접시를 따라가게 한 뒤(`box_under_plate`) 상태 기반 전문가 64/64, 96/96 성공, 충돌 0.

**안 되는 것**: 넓은 범위 데모로 학습한 BC(같은 MLP 512-256-128, 같은 가림)가 256 env 결정적 평가에서 **성공 0%, 충돌 100%**. seed 3개, 데이터 320개(`IL/data_wide_claude`, `_dart`)와 192개(scratchpad w46*) 모두 같다.
- 충돌 양상(`scratchpad/coll_diag.py`): t≈1.1~2.1초, 열린 그리퍼(fingers 0)의 **gripper_base**가 접시에 닿는다. TCP는 림 목표를 **+x로 10~15 mm 지나쳤는데**(목표를 넘었으니 rim−TCP의 x 성분은 음수) 행동 x는 여전히 +0.33~0.39다. 정책이 목표를 넘은 것을 반영하지 않는다.
- BC 학습 MSE: 좁은 범위 0.003 → 넓은 범위 0.011~0.012(약 4배). grip 부호 정확도는 99.6%.

**시도했지만 효과 없음**:
1. 목표 상대 관측 추가: `obs_rim_rel = rim − tcp`(3차원, 46차원 obs, `task.obs_rim_rel`). 43차원 체크포인트는 `eval.build_env`가 actor 입력 차원으로 감지한다.
2. 접시 관측 노이즈 ±5 mm → ±1 mm(`task.plate_obs_noise 0.001`; 실측 FP 흔들림은 접근 중 0.4~1.1 mm std).

**Claude의 다음 가설(검증 전)**:
- (a) 수집할 때 기록한 obs와 평가할 때 정책이 받는 obs가 다르다. 예: `collect.py`의 `before["obs"]`는 env.step 이전 obs인데, 관측 그룹 순서나 noise 적용이 RslRlVecEnvWrapper 경로와 다를 수 있다. 같은 상태에서 두 경로의 obs를 직접 비교해 확인한다.
- (b) 넓은 범위에서는 접근 행동이 타이밍 의존적이다. 전문가는 속도 상한(0.47~0.67 단위)으로 포화된 명령을 내다가 3 mm에서 바로 닫는다. 감속 구간이 없어서 "언제 멈추나"가 입력의 작은 차이에 걸려 있다. → 전문가에 감속 구간(P 제어, 예 목표 2 cm 안에서 비례 감속)을 넣어 라벨을 부드럽게 한다.
- (c) 비례 단위 문제: 상태(절대 좌표 m 단위)가 표준화 없이 MLP에 들어간다. 넓은 범위에서 rim_rel 같은 작은 신호의 상대적 크기가 작다 → obs 정규화(평균/표준편차)를 넣는다.
- (d) 받침이 kinematic이 되면서 gripper_base와 받침·접시의 접촉 조건이 달라졌을 수 있다(전문가는 충돌 0이므로 가능성 낮음).

**제약(바꾸지 말 것)**: 속도 상한(TCP 5 cm/s, 회전 10°/s, 그리퍼 20 mm/s), 실드 50/20 mm, 실로봇은 사람 승인과 비상정지. 실로봇 파이프라인은 `calibration/config.yaml plate_rl`의 checkpoint와 residual_base 쌍(오후 43차원 정책)에 고정되어 있으니 그 경로를 깨지 말 것.
- 작성자: Claude

## 2026-10-07 Codex Critic: 20:00 넓은 BC 충돌 검토 — 진단 결과와 제안
- 범위: read-only 코드/NPZ/체크포인트 CPU 검토. 시뮬레이터·GPU 학습·실로봇 명령 실행 및 RL/calibration 코드·설정·고정 policy pair 변경 없음. 근거 JSON: rl/reports/20261007_wide_bc_review/.
- **비교 수정**: 실제 성공 residual의 BC base(14:05:09)는 clean50+DART100, metadata val_mse0.01265633. wide43(19:20:55)는0.01052223, wide46(19:42:42)는0.01133083. 0.00351은13:49의이전clean50모델이다. 따라서 "좁은 성공BC 대비오차4배"는현재성공base에대한matched비교가아니다. 평균MSE로닫힌루프성공/안전성을판단할수없음.
- **관측경로 검토**: collect는 이전step/reset에서받은obs와현재action을저장한뒤step의next_obs기록. RslRlVecEnvWrapper는순서변경없음. 4개데이터폴더각12ep(24480frames)CPU검사에서 next_obs[:-1]==obs[1:],TCP/base/joint_vel/previous executed action 일치,46D rim_rel noise±1mm범위. trainmask8:16/36:43및저장first-layer해당열0 확인. 현재자료에서1stepshift/관측순서오류근거없음. 초기get_observations재계산은noise를다시뽑으므로별도compute끼리exact비교는오해가능.
- **전환부 실패 근거**: wide43을clean held-out32ep의첫close상태에CPU추론하면닫기출력0/32, x전진32/32,평균a_x+0.430; 전문가xyz는0. 전체grip부호99.626%지만첫close구간오류가가려짐. 좁은base도첫close10ep에서닫기0/10,+x평균.341이므로이현상단독이확장실패를증명하지는않음. wide첫10closeframes닫기80.94%,좁은80%. closed-loop overshoot분석을함께봐야함.
- **강한 가설**: expert err/scale(.0025)명령은3mmclose전환전까지포화되어유의미한감속영역없음(속도명령.47~.67이면1.2~1.7mm에서야비포화). 내부phase/close target/hold_pos/stall/random lift goal은BC입력에없고,GT3mm전환을noisy관측으로모방. hold_pos lead는firstclosewide평균x2.809mm,maxnorm4.203mm;좁은도유사3.551/4.194mm. 구조적취약점이며유일원인으로확정하지않음.
- **손실/표현**: wide프레임75.4%가raise/hold,접근14.6%,close9.9%;좁은도유사. uniform7DMSE는정지·닫기전환을과소대표. 마지막3~6mm희귀구간/overshoot open상태가부족하며,작은m단위relative벡터추가만으로해결안됨.
- **우선 제안(미구현)**: (1)phase·거리bin별XYZ오차/첫close오류/overshoot행동평가 (2)원본고정후별도작은실험에서relative입력스케일과전환부sampling개선 (3)필요하면최종약20mm에연속감속하는controller-consistent전문가데모및전환/targetlead정보추가 (4)작은BC닫힌루프평가로접근→멈춤→닫기가검증된뒤residualRL. 단순PPO추가학습/실드완화/전체prev-action·속도unmask는권하지않음. 정규화시추론에도같은통계필수,기존43D실기정책쌍보존.
- 기타: rim_rel은world vector,기존tcp/plate는base position. 현재robot root회전identity라숫자상같고100%실패원인은아님. 향후base회전시표현일관성수정필요.

## 2026-10-07 20:00 넓은 범위 BC 해결: 94.1% (Claude, Codex 검토 반영)
- Codex Critic 검토(위 항목) 수용: 관측 불일치 아님, 내 "MSE 4배" 비교는 잘못(성공한 좁은 BC base도 val 0.0127), 전환부와 내부 상태가 취약, 입력 스케일·전환부 sampling 개선 제안.
- 실험(256 env 결정적, 넓은 범위 x ±6/y ±8 cm, BC 단독 `--residual off`, 데이터 각 96+96):

| 변경 | 성공 | 충돌 |
|---|---|---|
| 기존 계단 라벨, rim_rel 추가 | 0% | 100% |
| + 접시 관측 노이즈 ±1 mm | 0% | 100% |
| + 비례 감속 전문가(`collect.py --slow_zone_m 0.02`), MSE 0.012 → 0.003 | 0% | 100% |
| + 상대 입력만(절대 좌표 가림) | 0% | 100% |
| **+ rim_rel ×20 스케일(학습 시 곱하고 첫 층에 접어 넣음) + 접근·닫기 프레임 3배 샘플링** | **94.1%** | **0.8%** |

- 결론: 넓은 범위 실패의 직접 원인은 **m 단위의 작은 목표 상대 신호(±2 cm)가 정규화 없는 MLP에서 거의 쓰이지 않은 것**과 **데이터의 75%가 유지 구간이라 접근·닫기가 과소 학습된 것**이다(Codex 제안 (2)와 같은 방향). 실행 중 진단에서 정책은 목표를 15 mm 지나치고 5 mm 낮은데도 +x·−z 행동을 냈다. 비례 감속 라벨은 같이 유지한다(MSE 개선, 자기 교정 라벨).
- 코드: `train.py bc`에 `bc.obs_scale [[43, 46, 20.0]]`와 `bc.phase_weight 3.0` 추가(데모 `phase` 사용). 관측은 46차원(`obs_rim_rel`). 43차원 체크포인트는 자동 감지하므로 실로봇 고정 정책 쌍은 영향 없음.
- 야간 v2 자동 실행(`rl/logs/wide_pipeline_v2.log`, 19:58 시작): 비례 전문가 데모 200 + DART 200(`IL/data_wide2_claude*`) → BC 3 seed 중 512 env 평가 최고 → `residual.base`와 실험명 `m1013_2f85_il_residual_wide` → residual RL(γ 0.998, 희소 보상 + **held_drift**, critic warm-up 150, 최대 3000 it, 게이트 통과 시 정지).
- Codex에게: 위 야간 실행 중에는 `rl/config.yaml`(residual.base, experiment)과 `IL/collect.py`, `rl/train.py`를 수정하지 마세요(실행 중 프로세스가 매 chunk 다시 읽음). 검토와 제안, 별도 경로의 read-only 분석은 환영. 남은 확인 거리: ① residual RL이 넓은 범위에서도 붕괴 없이 오르는지 ② held_drift가 들어 올릴 때의 +x 이동(실기 106 mm)을 없애는지(통과 체크포인트로 `eval.py export` 후 FK 확인) ③ 넓은 범위 정책을 실로봇에 쓸 때 `calibration/config.yaml plate_rl`의 checkpoint·residual_base·tolerance 갱신.
- 작성자: Claude

**finish codex now your turn**

## 2026-10-07 20:04 Codex: 야간 v2 코드 검토 — 자동 중단 조건 보완 필요
- Human 요청: Claude의 최신 수정과 야간 학습 구성을 확인하고 퇴근 전 문제 여부 판단. 수집 → BC → residual PPO → 평가 → 실로봇 export 연결을 read-only 검토했다. 실행 중 코드·config·프로세스·실로봇 상태는 변경하지 않았다.
- **정상 확인**: `rim_rel` 43:46 입력 ×20와 저장 시 첫 층 가중치 ×20 folding은 정상이며 추론에서 중복 적용되지 않는다. CPU 동등성 오차 <1e-5. 접근·닫기 phase 3배 sampling의 데이터 인덱스도 맞다. 기존 43D 모델은 추가 scaling을 건너뛰며 실로봇 checkpoint/base 쌍은 `calibration/config.yaml:94–95`에 고정, export가 base를 명시적으로 전달한다. 속도 상한과 안전 여유의 완화는 확인되지 않았다.
- **[P1] 수집/검증 실패 후 진행 가능**: `/tmp/claude-1000/-home-pan-pan/4ecf17ff-b84a-41fe-b581-b1eb38240569/scratchpad/wide_pipeline.sh:5,15–19`는 `set -u`만 사용하며 명령 반환값과 검증 결과를 강제 검사하지 않는다. 수집 `status=incomplete`도 BC 로더에서 거부하지 않는다. 수집 complete, 요구 episode 수, verify all_passed 및 실행 성공 여부를 확인하고 실패 시 즉시 정지해야 한다.
- **[P1] BC 전부 실패해도 RL 시작 가능**: 같은 스크립트 :22–41은 성공률 최고 모델만 선택한다. 최소 성공률·최대 충돌률 기준이 없어 세 seed가 모두 success 0/collision 1이어도 checkpoint가 있으면 best로 선택해 3000 PPO iteration을 시작한다. 수치 기준을 만족한 후보만 선택하고 후보가 없으면 종료하는 조건이 필요하다. 현재 수정은 구현하지 않았다.
- **오늘 밤 목표 평가 누락**: `train.py:365`의 통과 조건은 success ≥0.95뿐이다. `held_drift`는 첫 held 시점 XY를 올바르게 기록해 벌점을 주지만 수평 이동을 금지하지 않고 60 mm 이상에서 벌점이 포화된다. gate 통과만으로 '파지 후 전방 밀림 제거'를 확인할 수 없다. 통과 모델의 held 이후 TCP/접시 XY 최대 이동량·충돌률·실제 export FK를 확인한 뒤 실로봇 정책 교체를 검토해야 한다.
- **[P2, 현재 미발생] 검증 데이터가 없을 때 BC scaling 중복**: `train.py:190`이 `xva=xtr`로 동일 tensor를 공유하고 :211–212에서 두 번 ×20하여 학습 입력은 ×400, 저장 folding은 ×20가 된다. fallback clone 등으로 이후 수정 필요. 현재 v2 clean 데이터는 validation 40개로 정상이라 이번 실행에는 해당하지 않는다.
- 실행 상태(20:02 로그): clean 200개 수집 완료, rejected 2, 102000 frames, 186.99 s. verify 200개/102000 frames/train160/validation40/all_passed=true. DART 수집 진행 중이며 BC 3 seed와 residual PPO 결과는 아직 없다. 로그: `rl/logs/wide_pipeline_v2.log`.
- 야간 실행은 stage2만, critic warm-up150 + PPO 최대3000 iteration, chunk 재시도3회/15분 무출력 watchdog. 실로봇 자동 실행은 포함하지 않는다. 결론: 핵심 BC 수정은 타당하나 위 두 P1 자동 중단 조건을 보완하기 전에는 '문제 없음'으로 승인하지 않는다.

## 2026-10-07 20:21 Codex — Human 승인: 야간 중단 조건 수정 및 재시작
- Human은 기존 야간 실행 종료, 지적한 두 P1 수정, 재시작, W&B 기록을 명시적으로 승인했다. 이전 파이프라인과 Isaac/BC/RL 프로세스가 이미 종료된 것을 host 조회로 확인하고 중복 없이 새 실행을 시작했다.
- 유지: `train.py`, `plate_env.py`, `rl_utils.py`, `IL/collect.py`, `calibration/config.yaml`, `calibration/pipeline.py` SHA256 동일. 보상·PPO·전문가·속도 상한·실드·실로봇 정책 쌍 변경 없음. 백업과 hash: `rl/backups/20261007_night_codex/`.
- 구현: `rl/run_wide_night.py` + `rl/night_checks.py`. 기존 scratchpad `wide_pipeline.sh`는 새 checked driver를 exec한다. 수집/verify/BC/eval 반환값을 확인한다. 완전한 데이터 수·실제46D·source SHA·split을 검사하고 새 verify 결과가 all_passed여야 진행한다. 실패 시 config 변경·PPO 진입 전에 nonzero 종료한다. 불완전한 기존 데이터를 덮어쓰지 않는다.
- BC training-entry qualification: 512 env 결정적 평가에서 success≥0.85, collision≤0.02, unfinished=0, 결과 identity·rate 유한성·합계 검사. 통과 후보끼리 최고 성공률(동률이면 낮은 collision)을 선택한다. 전부 탈락하면 종료한다. **실로봇 배치 승인 기준이 아니다.**
- 새 curriculum state가 있으면 scratch 재시작을 거부하고, 파일 lock으로 중복 실행을 막는다. 기존 config와 보호 파일이 동시에 바뀌었으면 중단한다. 두 P1 검사 통과 후에만 `residual.base`·`paths.experiment` 두 항목을 원자적으로 갱신한다. stage2만, critic warm-up150 + PPO최대3000, 기존 chunk watchdog/retry 유지.
- 종료 처리: job token을 모든 child에 전달해 별도 PGID의 Isaac chunk 및 고아 프로세스도 SIGINT→TERM→KILL로 정리한다. GPU 없이 SIGINT 무시·별도 세션·부모 종료 조건을 재현해 종료 성공(15.3s). qualification/데이터 실패 회귀 테스트9개 통과.
- W&B: 온라인 preflight가 성공해야 시작. project `plate_place_KIMM`, run ID `25e685bc1f63`, URL https://wandb.ai/klng12345-dankook-university/plate_place_KIMM/runs/25e685bc1f63 . SDK online 및 실제 서버 history에서 Data 200+200, BC seed1/2 success·collision·val_mse·qualification 기록 확인. 기존 PPO update-wise scalars와 gate Eval/*는 같은 run ID로 이어진다. 네트워크 문제가 생기면 기존 tracking의 offline/TensorBoard 보존 경로가 남는다.
- 관측 추가만: `eval.py run_gate`에 held 첫 유효 위치 기준 TCP/접시 XY 이동 p95/max(mm), +x 전방 이동 p95(mm), 측정 episode 수를 추가했다. 행동·보상·성공/종료 판정 변경 없음. 자동 reset된 terminal sample은 제외하므로 종료 직전 한 step 이동을 누락할 수 있다. fake-env CPU 재현과 구문 검사 통과.
- 재사용 데이터: clean200 + DART200, 각102000frames/train160/validation40. 이번 실행에서 새로 verify를 실행해 통과. seed1 BC97.4609%/collision0, seed2 BC96.2891%/collision0 확인, seed3 평가 진행 중(20:21).
- 새 driver PID1117275(wrapper1117270), directory `/home/pan/pan/6D_pose/rl/logs/night_codex_20261007_201842`. 상위 로그 `rl/logs/night_codex_launcher.log`, latest pointer `rl/logs/night_codex_latest.json`, 상태 `/home/pan/pan/6D_pose/rl/logs/night_codex_20261007_201842/status.json`. 기존 v2 로그와 섞이지 않는다.

### 내일 W&B 그래프 확인 순서
1. `Eval/success`와 `Eval/collision`: 넓은 범위에서 결정적 성공≥95%인지, 충돌이 있는지 확인. 학습 중 `Episode_Termination/success`·`collision`도 함께 보고 붕괴 여부를 확인한다.
2. `Eval/held_tcp_forward_p95_mm`, `Eval/held_plate_forward_p95_mm`: 파지 후 앞으로 밀리는지 mm 단위로 확인. `Eval/held_*_xy_drift_p95_mm`와 `*_max_mm`를 함께 보고 드문 큰 이동도 확인한다. 측정 episode 수를 함께 확인한다.
3. `Eval/tcp_speed_p95`, `Eval/tcp_speed_max`와 `Eval/lift_range_during_hold_mm`: 실제 측정 속도와 들고 있는 동안의 흔들림 확인. 목표 변화량 상한이나 reward 벌점을 실제 속도·거리로 해석하지 않는다.
4. `Loss/surrogate`, `Loss/value_function`, `Loss/entropy`, `Policy/mean_noise_std`, `Loss/learning_rate`: 손실 폭증/NaN·불필요한 탐색 증가 확인. loss가 낮다는 이유만으로 성공/안전을 판정하지 않는다.
5. `BC/success`, `BC/collision`, `BC/val_mse` (x축 `BC/seed`): RL 이전 baseline과 비교. PPO/Eval 그래프의 x축은 `Progress/iteration`(PPO update), BC는 seed 기준이다. held_drift reward는 원시 거리 그래프가 아니다.
- 실기 교체는 내일 성공 checkpoint의 수평 이동·속도·export FK/GUI 확인 후 Human과 검토한다. 이번 야간 실행은 실로봇 자동 동작이나 policy pair 교체를 포함하지 않는다.

## 2026-10-07 20:23 Codex — 야간 재시작 실제 PPO 진입 확인
- BC seed3 success97.8515625%, collision0%, lost0%, timeout2.1484375%로 최종 선택(512 env). TCP speed p95 4.400cm/s, max 4.998cm/s. held TCP forward p95 2.21mm, plate forward p95 4.91mm. 이는 BC baseline이고 최종 residual RL 결과는 아직 아님.
- residual PPO run `/home/pan/IsaacLab/logs/rsl_rl/m1013_2f85_il_residual_wide/2026-10-07_20-21-50_stage2`에 실제 진입, critic warm-up150 중 iteration21 확인. zero residual/base 정확히선택, actor/noise freeze 로그 확인. W&B online 동일 run URL 확인.
- config diff는 residual.base·paths.experiment 두 항목뿐임을 YAML 구조 비교로 재확인. 보호 파일 SHA 모두 동일. Human 실로봇 정책 쌍 유지.

## 2026-10-08 08:40 Codex — 야간 RL 통과 실패
- Human 요청으로 로그·curriculum state·BC/gate 계측·체크포인트·W&B 서버를 read-only 확인. 시뮬레이션 재평가, 재학습, 로봇 실행/정책 교체는 하지 않았다. 결과 JSON: `/home/pan/pan/6D_pose/rl/reports/20261008_overnight_result/result.json`.
- 10/7 23:35:18 KST, critic warm-up150 + PPO3000을 마치고 95% gate 미통과로 종료. passed_checkpoint=null. host에 야간 학습/Isaac/W&B child 프로세스 없음. W&B 서버 상태 finished, 온라인 기록과 마지막 native success0/timeout1 및 Eval83.4473%를 대조했다.
- 위치 범위는 학습·평가 snapshot 모두 x±6 cm/y±8 cm. sim GT 위치에 ±1 mm 관측 noise이며 FoundationPose의 새 넓은 실기 검증은 아직 아니다.
- 선택 BC seed3(512ep): success97.8516%, collision/lost0%, timeout2.1484%. held TCP +x p95 2.208mm, 접시+x4.907mm, TCPXYp953.913/max16.948mm. TCP speed p95 4.400/max4.998cm/s. 유지 중 높이 변화 중앙값13.400mm.
- 유일한 RL gate(2048ep, first run model300): success83.4473%, collision/lost0%, timeout16.5527%. held TCP+x p9587.054mm, 접시+x89.626mm, TCPXYp95107.990/max201.702mm. TCP speed p954.997/max5.461cm/s. 유지 중 높이 변화 중앙값53.297mm. 평가 표본 수는 다르지만 RL의 전방 이동 악화와 95% 미통과는 명확하다.
- 첫 PPO chunk saved model300의 최근 학습 success proxy는98.3%였으나 결정적 평가83.45%. 이후 chunk에서 rollback 후에도 붕괴하여 말기 최근50it 학습 성공0%, timeout100%. 마지막model2396의 결정적 평가는 없으므로 '최종 평가 성공0%'로 표기하지 않는다.
- NaN/Inf scalar0건 및 검사한 BC/model300/finalmodel2396 가중치 finite. 소프트웨어 검증 오류/프로세스 crash가 아니라 현재 RL 구성이 좋은 BC를 악화시킨 결과다. 원인 단서: actor freeze critic warm-up 동안 실제 adaptive LR0.01(설정1e-4의100배). 스케줄·optimizer resume 확인 후 원인 확정 필요.
- 이번 RL은 실기 채택 근거가 없고 전방 밀림 제거 목표 미달. BC baseline을 보존하고 matched 평가·GUI/FK를 우선, residual 정책 변화 및 LR/전역 best 보존/수평 이동 제약을 검토해야 한다. 새 학습은 이 요청에서 시작하지 않았다. 기존 실로봇 policy pair와 안전/속도 설정 및 보호 파일 SHA는 유지.
- W&B: https://wandb.ai/klng12345-dankook-university/plate_place_KIMM/runs/25e685bc1f63 . success curve 붕괴와 held forward/actual speed를 함께 확인해야 한다. loss 감소만으로 성과를 판정하지 않는다.

## 2026-10-08 Codex — residual PPO 붕괴 재발 방지 수정 (Human 검토 필요)
- 근거: BC seed3 결정적 success 97.85%, collision/lost 0%. 첫 RL gate model300은 training proxy 98.3%인데 결정적 success 83.45%, held TCP/plate forward p95 약 87/90 mm. 후반 training proxy 0%/timeout100%였고, model2396의 결정적 평가는 없으므로 최종 모델 실패율은 미확정.
- 원인 가설을 하나로 단정하지 않음. 코드상 확인된 위험은 (1) RSL-RL adaptive schedule이 작은 KL에서 minibatch마다 LR×1.5, max 1e-2까지 올릴 수 있음(설정 1e-4 대비 100배; warm-up에서 KL이 작아 특히 취약), (2) held_drift가 2cm 기준·clip3여서 6cm에서 수평이동 벌점 포화, (3) deterministic regression/safety 결과에 따른 조기중단이 없어 proxy가 좋은 중간 checkpoint 이후에도 나쁜 구간을 계속 학습 가능. 학습 중 성공 proxy는 결정적 gate 성능을 보장하지 않음.
- 코드 수정: residual PPO `schedule: fixed`, LR `2e-5`; resume 시 optimizer group LR을 설정값으로 재고정. held_drift는 1cm 단위 기울기를 최대 40cm까지 유지. curriculum은 BC 시작 checkpoint부터 결정적 baseline을 기록하고, 매 chunk 결정적 평가에서 baseline 대비 5%p 이상 하락 또는 success<90%, collision/lost>0이면 그 자리에서 정지(통과 checkpoint 미기록). proxy는 평가 생략 조건으로 사용하지 않음. 이전 넓은 실험과 분리한 `m1013_2f85_il_residual_wide_stable` 사용.
- 야간 자동 driver 기본 실험명도 stable로 맞춤. 추후 qualified BC만 갱신하고 안정화 설정은 덮어쓰지 않음.
- 보호: `calibration/config.yaml`, 실기 정책 쌍, 실제 속도·실드·로봇 명령 변경 없음. 새 학습 미실행. YAML parse 및 Python syntax compile만 통과; Isaac simulator에서 새 reward/LR/rollback 동작은 아직 검증 전이며 재학습 결과를 보장하지 않음.
- 백업: `rl/backups/20261008_collapse_fix/`. 다음 실행 전 Human은 보상 크기(held drift) 및 90% regression floor를 확인하고, 첫 chunk deterministic gate와 W&B `Loss/learning_rate`, `Eval/success`, `Eval/held_tcp_forward_p95_m`, `Eval/held_plate_forward_p95_m`, collision/lost를 확인한다.

## 2026-10-08 Codex — W&B run 식별 규칙과 기존 이름 정리
- Human 요청: run 이름이 `wandb_eval_s2_resume`처럼 반복되어 실험 구분이 불가능함. W&B API에서 project `plate_place_KIMM`의 16개 run 설정·요약을 읽어 알고리즘·experiment·stage·update/checkpoint 역할을 식별했다.
- 기존 run 16개를 W&B에서 이름 변경했다: `robot_PPO__PPOonly__...`, `robot_PPO__BCinit__...`, `robot_PPO__BCresidual__...`, BC gate는 `robot_BC__...`. 서로 다른 run ID와 metrics는 유지했고, 각 run Notes에 이전 표시명을 기록했다. 예: overnight wide `robot_PPO__BCresidual__wide__S2__u2396_gate_m0300_834pct`; 오후 통과 run `robot_PPO__BCresidual__il_residual__S2__train_u0648_gate_m0400_979pct` 및 별도 gate run.
- 코드: `train.py::start_tracking`은 train ID를 experiment/stage에 대해 유지하되 gate 평가에는 각 tag별 독립 run ID를 사용한다. 이름은 `robot_PPO__{PPOonly|BCresidual}__{short experiment}__stageN__{train|gate__tag}`. 야간 driver의 BC qualification/preflight run도 독립 ID와 `preflight` 역할을 사용한다. 따라서 eval metric은 train run을 덮거나 합치지 않는다.
- 검증: API에서 전체 16개 목록을 다시 조회해 새 이름 확인, `py_compile` 통과. 변경 전 소스 백업 `rl/backups/20261008_wandb_names/`. 향후 새 이름 규칙은 아직 새 run 생성으로 검증 전.

## 2026-10-08 Codex — M1013 Base→flange 및 2F-85 TCP 기하 검토
- Human 확인: `calibration/config.yaml` 홈 관절값 `[-5.98, -14.75, 117.74, -3.92, 7.46, -85.07]°`는 실제 시작 자세이며 현재 로봇도 같은 자세, 축 방향 설정도 확인됨. M1013 URDF FK `T_base_flange`: 위치 `(500.948, -18.880, 584.040) mm`; ZYZ `(−6.551, 110.443, −89.163)°`. 홈 자세 기준이며 이 턴에 실시간 ROS pose를 새로 읽은 값은 아님.
- `rl/assets/m1013_2f85.urdf`의 모델상 flange→gripper base 20 mm + Robotiq base→pinch TCP 155.8 mm, link_6 +Z 방향, 총 translation 175.8 mm, 회전 Rz(−90°). 이를 홈 자세 FK에 합성한 **시뮬레이션 가정** `T_base_TCP` 위치 `(664.600, −37.674, 522.636) mm`; ZYZ `(−6.551, 110.443, −179.163)°`. 프린트 부품·실제 체결 방향·TCP 기준점에 대한 실측값이 아니며 실물 TCP 보정 완료로 취급하지 않음.
- 제공 GrabCAD 링크와 PickNik 문서는 UR5용 RealSense wrist-camera adapter(카메라 장착대)를 가리킴. 따라서 D455가 이 부품으로 손목에 장착된 경우 flange→camera adapter→D455 optical frame은 별도 사슬이며 Robotiq pinch TCP와 자동으로 직렬 연결되지 않음. D455 adapter CAD/치수 파일은 현재 프로젝트에서 찾지 못함. 실제로 adapter가 flange와 그리퍼 사이의 기계 하중 경로에 들어가는 구성이라면 그 실물 변환을 추가 계측해야 함.
- 실물 TCP는 실제 체결 조립체(AGC-DOO-KIT-85 coupling + 2F-85 및 장착 adapter가 있다면 포함)를 기준으로 flange→pinch point를 pivot/다점 보정하고, 여러 손목 방향에서 고정 기준점에 대해 검증할 것. CAD만으로 프린트 공차와 실제 장착 방향은 확인되지 않음.
- 보호: 로봇 실행, 학습, calibration 산출물 및 config 변경 없음. 모델 가정값과 미확정 측정 항목만 기록.

## 2026-10-08 Human 보충 — wrist-camera adapter의 실제 장착 위치 확인
- 정정: Human 확인상 출력한 PickNik 기반 adapter는 **실제로 M1013 flange와 Robotiq 2F-85 사이에 장착되어 있고, D455도 그 adapter에 고정**되어 있다. 이전 Codex 기록의 “실제 하중 경로에 들어가는지 확인 필요”는 더 이상 불확실한 사항이 아님.
- 따라서 실제 변환 사슬은 `T_base_TCP = T_base_flange · T_flange_adapter_stack · T_adapter_to_Robotiq_base · T_Robotiq_base_TCP`이며, 실제 장착된 TCP 수치는 앞서 계산한 단순 simulator 모델값과 동일하다고 볼 수 없다. 기존 20 mm `AGC-DOO-KIT-85` coupling과 출력 adapter가 별도 부품이면 두 변환을 모두 포함해야 함.
- 현재 저장소에는 수정 출력한 D455 adapter CAD/URDF와 M1013 flange 기준 장착 원점·회전 정보가 없음. GrabCAD 링크의 기본 모델은 UR5용이며, PickNik 예시 URDF의 tool-changer/UR adapter 체인 치수는 우리 M1013의 체결 변환으로 그대로 전용할 수 없음. 따라서 물리적으로 장착되었다는 사실은 확정되었으나, CAD/실측 변환 수치가 없는 상태에서 adapter 포함 TCP 좌표의 정확한 재계산은 아직 불가.
- wrist D455의 `T_flange_cam` hand-eye 보정은 카메라가 동일하게 장착된 상태로 수행됐다면 adapter를 포함한 실측 flange→camera 변환이다. 카메라나 bracket을 재장착/이동했다면 재보정 필요. gripper pinch TCP는 별도 보정 대상.
- 다음 계산 입력: Human 수정본 adapter STEP/STL(또는 출력 STL)와 그리퍼/커플링 체결 방향이 식별되는 assembly CAD/사진. CAD 프레임을 flange 기준으로 정합한 뒤 FK 홈값으로 Base→adapter, Base→D455 mount/optical, Base→pinch TCP를 각각 계산하고, pivot 접촉으로 실측 검증한다. 실측 전까지 simulator TCP는 실물 기준값으로 사용하지 않음.

## 2026-10-08 Codex — 기존 wrist hand-eye로 adapter 장착 D455 pose 재계산
- 저장된 `wrist_handeye/20261006_111318_clean/result.json`의 `T_flange_cam`(14 sample; pos residual mean 0.884 mm, rotation mean 0.368°)를 Human 확인 홈 관절값의 URDF FK `T_base_flange`와 곱함. 결과 `T_base_wrist_cam` 위치 `(548.944, −29.901, 644.471) mm`, ZYZ `(−6.937, 118.473, −90.703)°`.
- 이 계산은 hand-eye 당시 flange↔D455 optical frame 변환이 현재와 같을 경우 adapter와 카메라 장착을 이미 포함한다. 결과 날짜는 2026-10-06이며 이후 bracket/카메라를 분리·이동했는지 아직 기록으로 확인되지 않았으므로 현재 실물 카메라 pose로 사용하기 전 재현성 검증 필요.
- 이 hand-eye는 flange→camera를 정함; flange→Robotiq pinch TCP를 주지 않음. 실물 TCP는 현재 장착 stack 상태에서 별도로 pivot/tool calibration 해야 함.

## 2026-10-08 14:00 목표 1·2 계획과 학습 계획 (Human 목표 → Claude 초안, Codex 검토 요청)
목표 원문 요지는 CONTEXT "Human 목표 (2026-10-08)"에 있다. 기한은 다음 주(약 10/15), 시연은 10/21경.

### 현황
- 목표 1(파지): IK 직선 + BC, 반경별 경로 라이브러리 + 1축 회전. 실기 5회, 높이 오차 말고는 양호(Human).
  - 범위: 반경 1.04~1.32 m, 방향 ±60°, 프레임 간섭이면 거부.
- 목표 2(바구니): 미착수. 트레이 실측 일부만 있다(메모리/CONTEXT 10/7: 와이어 간격 95 mm, 핀 피치 20 mm, 핀 높이 60 mm). CAD 없음.

### 원칙
- 학습하는 것과 하지 않는 것을 나눈다.
  - 인식(접시는 FP, 바구니는 pose)과 접근 경로(계획·최적화)는 학습하지 않거나 오프라인 최적화로 푼다.
  - **접촉이 있는 동작(파지, 슬롯 삽입)만 IL(BC)로 학습**하고, RL은 BC를 넘길 때만 채택한다.
- RL 보상은 단계별 [−1, 1], 종료 |r| ≤ 10. 매 chunk 결정적 평가를 하고 BC보다 5%p 떨어지면 중단한다(10/8 붕괴 교훈).
- 속도는 항상 최저(실기 6°/s 상한, Human 승인). 실기 명령은 Human만 실행한다.

### A. 목표 1 마무리 (10/9~10/12)
1. **파지 높이 보정.** 5회 실기 기록(`real_pipeline/*/robot_state.jsonl`: 관절 20 Hz + FP)에서 닫는 순간 TCP(FK)와 FP 접시 높이의 차를 구해 편향을 측정한다. 라이브러리에 접시 높이 축(dz −10~+10 mm, 2~5 단계)을 추가하고 FP z로 고른다. 고정 편향이면 offset z 하나로 처리한다.
2. **범위를 테이블 전체로.**
   - 반경 < 1.04 m와 프레임 근처는 그리퍼 접근 방향을 바꿔야 한다.
   - 전문가에 접근 방위 φ(반경 방향 대비 ±40°)와 pitch 파라미터를 추가해 데모를 모은다. BC는 tool axes 입력이 있으니 같은 정책으로 학습한다.
   - 라이브러리를 (r, φ, dz) 격자로 만들고, 실행 시 간섭 여유가 가장 큰 φ를 고른다.
   - 기준: sim 결정적 ≥95%, 실기 위치 5곳 연속 성공.
3. **경로 최적화.** 지금은 "홈 → 접시 앞 10 cm" 관절 1회 이동이다.
   - 1차: 라이브러리 생성 때 오프라인 최적화(관절 이동량·시간 최소, 간섭 여유 ≥ margin, 부드러운 spline, 경유점 수 감소).
   - 2차(선택): 경로 생성 정책을 GRPO 방식으로 학습. 입력은 (r, φ, 장애물), 출력은 경유점, 보상은 0~1(도달·짧음·부드러움·여유), critic 없음.
4. **가려짐 기반 자동 파지.**
   - FK로 손가락 위치를 알고 T_base_cam으로 고정 카메라에 투영한다. FP 접시 테두리와의 거리, 깊이·마스크 가림 비율이 기준을 넘으면 닫기를 제안한다.
   - 처음에는 "제안 + Human c 확인", 검증되면 자동으로 바꾼다.

### B. 목표 2 데모 = 티칭 (10/9~10/13, 시연 보험)
1. `pipeline.py`에 teach 모드를 추가한다. Human이 펜던트·직접 교시로 자세를 잡고 키를 누르면 관절·posx를 기록한다(읽기 전용).
   - 슬롯 k마다 기록한다: 운반 자세 → 슬롯 위 → 접시 수직 회전 → 삽입 하강 → (그리퍼 열기) → 후퇴.
2. 파지 위치(방위)는 매번 달라도 손 안의 접시 자세는 같다(항상 로봇 쪽 테두리를 같은 tool 방향으로 잡음). 그래서 lift 뒤 **공통 운반 자세**(관절 고정)로 이동한 다음 슬롯 k 경로를 재생한다.
3. 실행 순서: 첫 접시는 슬롯 1, 두 번째는 슬롯 2, …(카운터 파일). 각 구간은 6°/s 한 동작이고 그리퍼는 Human 선택.
4. 성공 확인(초안): 열기 후 FP나 깊이로 접시가 그대로 있는지 확인한다. 아래 C4의 반력 확인도 한다.

### C. 목표 2 학습 = 바구니 pose가 바뀌어도 (10/13~10/20)
1. **바구니 모델.** 실측으로 파라메트릭 CAD를 만든다(외곽, 와이어, 핀 배열). 또는 D455로 TSDF 스캔한다(접시와 같은 방식).
2. **바구니 6D pose.** 얇은 와이어는 FP에 어렵다(깊이 노이즈). **1안: 바구니 프레임에 ArUco/AprilTag 2~4개**(우리 캘리브레이션 도구 재사용). 2안: CAD + FP(시험만).
   - 슬롯 pose = T_base_tray × T_tray_slot_k. 티칭 데이터로 슬롯 오프셋을 보정한다.
3. **삽입 스킬 학습.**
   - Isaac에 바구니(핀은 얇은 원기둥)를 넣고, pose를 무작위로 둔다(±10 cm, yaw ±30°).
   - 전문가: 슬롯 좌표계 기준 스크립트(슬롯 위 정렬 → 수직 회전 → 저속 하강, 필요하면 작은 좌우 탐색).
   - BC: **슬롯 상대 좌표 입력**(파지와 같은 평행이동 불변 설계, copycat 입력 제외), DART를 쓴다. 실기 직접 교시 데모를 추가 데이터로 쓴다.
   - 선택: residual RL(보상 [−1, 1], 성공 +10, 충돌·떨어뜨림 −10, 중단 조건 포함).
   - 기준: sim ≥95%, 실기 바구니 위치 3곳 × 연속 3회.
4. **삽입 확인.** M1013은 관절 토크 센서가 있어 외력 추정이 된다(별도 F/T 없이). 읽기 전용 API(외력·토크 조회)부터 확인한다. 열기 전에 ±3 mm 좌우로 조금 움직여 반력이 생기면 갇혀 있다고 판정한다(아주 저속, Human 승인 후).

### D. 일정 (초안)
| 날짜 | 작업 |
|---|---|
| 10/9 | A1 높이 분석·보정, B1 teach 모드, Human이 슬롯 1~3 티칭 |
| 10/10 | B2~B3 슬롯 경로 실행(목표 2 데모 1차), A4 자동 파지 제안 모드 |
| 10/12 | A2 범위 확장(φ, pitch 데모 + 라이브러리 격자), A3 오프라인 경로 최적화 |
| 10/13~14 | C1~C2 바구니 모델 + tag pose, 슬롯 pose 검증 |
| 10/15~19 | C3 sim 삽입 학습, 실기 시험, C4 반력 확인 |
| 10/20~21 | 통합 리허설 |

### Codex 검토 요청
1. 바구니 pose 1안(태그)과 2안(CAD+FP)의 위험.
2. 삽입 BC 입력 설계(슬롯 상대 좌표, copycat 제외 목록).
3. 관절 토크 기반 반력 판정의 신뢰도와 두산 API.
4. 공통 운반 자세 가정(손 안 접시 자세 일정)이 깨지는 경우.

## 2026-10-08 Codex 검토 — 목표 달성 및 학습 실행안

### 판단 기준
- 최종 실증은 임의의 접시 pose를 인식해 파지·들기 한 뒤, 바구니의 pose/슬롯 기준으로 접시를 세워 넣고 안정적으로 놓는 것이다. 단순히 접시를 들어 올리거나 고정 바구니에 재생하는 것만으로 목표 완료라 하지 않는다.
- 현재는 **BC 단독 + 기하/IK 경로**를 기본으로 유지한다. PPO/residual RL은 다시 켜지 않는다. 어젯밤 BC 성능 저하 및 붕괴가 확인됐고, 10/8 로그의 넓은 범위 IK→BC 전체 성공률은 92.7~93.0%, 가까운 구간은 62~74%이며 충돌이 주요 실패다. 98~100%는 가까운 구간을 제외한 특정 영역 또는 성공 library 구간의 값으로 한정해 표현한다.
- “임의 위치”는 로봇·작업셀의 실제 안전 reachable set 안에서만 의미한다. 도달 불가/충돌 여유 부족 pose는 안전하게 거부하는 동작도 올바른 결과로 정의한다.
- 속도는 성공률과 독립적인 필수 gate다. 실기 실제 최저 속도 설정과 제어 계층 제한, 실제 TCP/관절 속도·가속도·파지 후 흔들림을 계측하기 전에는 어떤 학습/시뮬레이션 결과도 실기 안전 승인으로 간주하지 않는다.

### 우선순위와 통과 조건
1. **안전 전제부터 해소:** 10/8에 기록된 `SOS violation (group 5, index 7056)`의 펜던트 원인/복구 상태를 확인한다. 해소 전에는 실기 경로를 재실행하지 않는다. 사람 승인 없는 로봇 이동은 하지 않는다.
2. **실물 조립체를 모델에 반영:** flange–출력 adapter–Robotiq–D455 실제 구성의 CAD/실측 변환과 pinch TCP를 확보한다. 현 library의 0.165 m tool / 0.06 m 반경 및 프레임·테이블 여유 검사가 이 adapter와 카메라까지 포함하는지 확인하고, 누락이면 안전 포락선과 경로를 재생성한다. TCP/hand-eye 오차가 있는 상태에서 library를 실기 기준으로 쓰지 않는다.
3. **목표 1의 평가를 위치별로 닫기:** 현 checkpoint를 고정 기준선으로 보존하고, 별도 후보에서만 학습한다. 반경·방위각·높이·가림 조건별 고정 평가 집합과 실패 지도를 만든다. 성공/충돌/미파지/timeout 및 파지 후 접시 상대 이동량을 각각 보고한다. 가장 가까운 충돌 구간은 우선 자세·접근 방향/작업공간 경계를 바꿔 해결하고, 충돌 사례를 그대로 학습에 넣어 성공률만 올리지 않는다.
   - Gate A(sim): 안전 reachable set 안의 격자별 결정적 평가 ≥95%, 충돌 0, 열린 검증 위치에서 성능 유지. 평가 seed/pose는 학습 데이터와 분리한다.
   - Gate B(실기): SOS 해소, TCP/충돌 모델 검증, 최저 속도 계측 완료 후 사람 승인하에 안전한 내부 위치부터 단일 시도씩 검증한다. 여러 위치 연속 성공과 접시 안정성 확인 전에는 범위를 넓히지 않는다.
4. **목표 2는 기하 정의 후 시작:** rack/slot CAD 또는 실측 치수·슬롯 좌표를 먼저 만들고, 바구니 기준 pose를 산출한다. 얇은 철선 인식에만 의존하지 말고 우선 tag 기반 바구니 pose를 고정 D455로 검증한다. tag→slot 변환은 실측으로 맞춘다. 초기 실증은 한 바구니·한 슬롯부터 시작하고 이후 위치/yaw 변화를 늘린다.
5. **삽입 동작은 티칭/BC 우선:** Human이 저속으로 운반→접시 회전→slot 위 정렬→삽입→지지 확인→그리퍼 개방→후퇴를 시연하고, slot 상대 좌표 입력으로 BC를 학습한다. 먼저 단일 고정 슬롯에서 경로의 간섭·접시 손상·안정성을 검증한 뒤, 바구니 위치와 방향 변화를 단계적으로 늘린다. 토크 기반 접촉 판정은 API와 영점/재현성을 확인하기 전 성공 신호로 사용하지 않는다. 좌우 흔들기 검증도 검증된 접촉력 한계가 없으면 넣지 않는다.

### 학습·모델 운영 규칙
- 데이터는 실제 동작 단계별로 분리한다: 접근은 검증된 IK/경로 계획, 접촉 파지·들기 및 슬롯 삽입은 BC 시연. 필요하면 이후에만 BC 고정/제한 residual 학습을 별도 실험으로 검토한다.
- 새 데이터·checkpoint는 버전/설정/시드/평가 pose와 함께 분리 저장한다. 기존 기준 checkpoint는 덮어쓰지 않는다. 후보는 고정 검증 집합에서 기준선보다 나빠지거나 충돌이 발생하면 승격하지 않는다.
- W&B에는 run 이름에 날짜·stage·알고리즘·범위·seed를 넣고, train/validation loss, 성공·충돌·미파지·timeout 비율, 단계별 성공, 위치별 성공 지도, 파지 후 drift, 명령/실측 속도를 기록한다. 성공률 하나만으로 승격하지 않는다.
- 사람 시연은 처음부터 많이 모으기보다 평가 지도에서 실패하는 구간/접근 방위/높이를 겨냥해 보충한다. 시작·검증·테스트 pose는 섞지 않는다.
- 범용성은 먼저 대상(접시/트레이)과 작업셀 범위를 명시해 제한적으로 입증한다. 새 접시·새 바구니·새 카메라 위치는 각각 분포 변화로 보고 별도 검증한다.

### 확인된 blocker / Claude 초안 보완
- CONTEXT에는 `98~100% (반경 1.04~1.32m)`라고 요약됐지만 같은 날 LOG의 넓은 범위 전체 측정은 92.7~93.0%, 가장 가까운 구간은 62~74%다. 분모/평가 범위가 다른 값이므로 하나의 전체 성공률처럼 쓰지 않는다.
- LOG에 실기 SOS 알람의 해소 기록이 없다. 더 최신의 “실기 5회 양호” 서술과 시간상 충돌하므로, 알람 이후 실제 재시도/상태와 안전 확인 여부를 Human/Claude 기록으로 확인해야 한다.
- flange와 2F-85 사이 adapter가 실제 장착된 사실은 Human이 확정했다. 그러나 실제 adapter 포함 TCP/충돌 외형은 미계측이며, 현 library의 단순 tool 모델이 adapter/D455를 포함한다고 검증되지 않았다. 경로 안전성 및 오프라인 accepted 결과를 실기 유효성으로 해석하지 않는다.
- Claude 초안의 10/9~10/21 일정은 측정·안전 gate 통과에 따라 재산정해야 한다. 목표 날짜를 맞추려고 검증을 생략하거나 속도를 올리지 않는다.
- 별도 이슈: Claude 초안은 내부 F/T가 있는 것처럼 서술하지만, 실시간 외력/토크 API와 안전하게 사용할 수 있는 임계값은 확인되지 않았다. 확인 전 force-confirmation 기능은 미구현 상태로 취급한다.

## 2026-10-08 Codex — A1 편향 판정 및 opt-in phi/pitch 실험
- 높이 보정은 고정 평균 offset 대신 별도 dz library 후보를 검증한다. 실기 preclose FP 높이 변화, 당시 policy/path 차이, adapter 미보정이 섞여 있어 −5 mm 평균을 물리 TCP 오차로 해석할 근거가 없음.
- opt-in candidate bank에서만 rim_offset, tool_z/finger_y, 시작 IK 자세를 per-env로 일치시킨다. obs 차원·기존 보상·hold 판정·실드 여유·속도 상한은 유지한다. 새로운 phi/pitch에서 BC를 다시 평가하며 기존 정책 성능을 전용하지 않는다.
- library joint sweep의 smoothstep peak derivative(1.5)를 반영해 6°/s 이하가 되도록 sweep 시간을 늘렸다. 접촉 구간은 기존 5cm/s·10°/s 제한 유지, export_hold_s=5로 검증. 실기 config는 아직 갱신하지 않음.
- 실험/보고용 metric은 executed plan과 candidate library를 구분하고 FP drift도 별도 기록. Human 검토 대상이며 실기 적용은 10/12 확인 후.


## 2026-10-08 Codex — 목표 1 후보 평가/선택 규칙 (Human 검토 대상)
- BC 학습 입력 숨김/손실/성공 판정은 기존 설계를 유지. 신설 데이터/phi·pitch goal frame만 opt-in으로 추가; 기존 1200개 + 400개. 새 실험은 BC만, residual/PPO 없음.
- 격자 library는 각 방향(phi,pitch)에서 최근접 **성공** 반경을 선택한 뒤 전체 FK 경로 여유가 큰 방향을 고름. 기존 단일 방향 library는 최근접 반경 선택 그대로. 높이 축은 검증한 root z와 tolerance 안에서만 선택, 실제 config에는 연결하지 않음.
- 작업영역 평가는 --bank_fixed로 1024개 고유 xy를 모두 1회 시험. 방향 선택 후 같은 xy 재시험은 validation이며 최종 주장은 독립 seed gate로 확인해야 함. FK workspace map은 보간 반경의 물리 성공 지도가 아님.
- 속도 평가에 에피소드별 max_tcp_speed_mps와 speed_over_cap_episodes 추가. 성공률 목표 충족만으로 배치하지 않음. 실패에서의 속도 초과도 숨기지 않음.
- clearance 계산 최적화는 원본 voxel query의 모든 geometry/sample을 동일하게 batch 처리(원본값 일치 시험). map은 부분 자세의 최소거리 upper bound가 이미 전체 검증된 최선보다 작거나 같을 때만 그 후보를 생략; 장애물/margin/성공 threshold를 바꾸지 않음.
- 실물 TCP/adapter/운반 접시 간섭이 미확인이라 새 후보는 candidate_only. Human의 실제 조립체 계측과 저속 확인 전 기존 config 변경/실로봇 실행 금지.


### Codex 추가 기록: 영역 시연 보충 후 입력 스케일/속도 데이터 gate 후보 (Human 검토 대상)
- 프레임 주변 성공 clean300 + DART300 추가 수집/재검증. 검증한 5초 hold/실드/충돌 조건 유지. 기존 시연 삭제 없음.
- 원본 workspace 시연 속도 audit에서 clean4회(.05018m/s 이하) / DART1회(.06133m/s) 상한 초과 발견. 물리 파지 성공만으로 저속 데이터 자격을 인정하지 않음.
- BC 옵션 max_demo_tcp_speed_mps=.05 및 tolerance=1e-5 신설: 실제 현재→다음 TCP/기록 dt로 전체 episode 최고를 계산, 넘으면 후보 학습에서 제외하고 demo_selection.json에 기록. 데이터 자체는 보존. 기존 BC는 옵션 없으면 그대로.
- retained 입력 중 TCP z[18:19], plate z[27:28]를 rim_rel처럼 ×20 스케일하는 별도 후보. 작은 높이 신호의 학습 조건을 개선하려는 실험이며 차원/관측 종류/phase 입력을 추가하지 않음. 입력 마스킹과 phase_weight=1 그대로, 저장 시 첫 층 가중치로 스케일을 접어 runtime raw 관측의 의미 유지. 기존 config/model 변화 없음.
- 위 손실 데이터 선별 및 scaling 변경은 후보 gate 결과와 함께 Human이 검토해야 하며, 후보 sim 성공만으로 실제 설정을 변경하지 않음.


### Codex 추가 기록: 2단계 그리퍼 닫힘 유지 후보 (Human 검토 대상)
- BC 3-seed 보충 후 최대 93.16%, 높이 입력 scaling+속도 시연 선별 후보 최대93.95%(같은 tuning set, 독립 최종 gate 아님). 실제 config는 유지.
- `eval.py --latch_gripper` opt-in: 2단계에서 BC가 림 10mm 안에서 처음 닫기를 명령한 뒤 reset까지 닫힘 유지. 접근/팔의 들기/성공/held 조건은 기존 BC/물리 판정 그대로. 고정 lift 명령, 위치 강제 부착, 마찰/힘/실드 완화 없음. 3단계에는 사용 거부(삽입 후 release가 필요).
- 가설: 단계2는 release가 없는 grasp→lift→hold이고 Human 실기도 별도 close 후 유지한다. BC가 파지 후 실수로 open을 내는 것을 제어 계층에서 막는 인터록을 비교한다. 새 결과는 **BC + gripper latch**로 명시해야 하며 순수 BC가 독립적으로 모두 해결한 결과로 보고하지 않는다.
- 속도 gate는 commanded cap와 기존 허용 상한(.05m/s)을 모두 기록. 나중에 명령 cap를 낮추더라도 기존 5cm/s 실측 상한은 올리지 않음.


### Codex 추가 기록: controller 실제 속도/저속 시간 여유
- 리셋 직후 첫 2스텝은 capped step를 계산하고도 raw xyz action을 super에 보내던 코드 오류 수정. 첫 두 스텝도 같은 번역 cap 적용. 실제 controller method를 mock IK base로 실행하여 fresh/reset/rotation/target lead cap 시험 통과.
- opt-in `--hold_windup_m .002 --speed_cap .04`: held TCP 목표가 실제 EE보다 앞서는 누적오차 상한을 10→2mm로 축소, 명령 속도 5→4cm/s로 축소. 기존 yaml/config 상한은 그대로. 이 시험에서 실제 성공 episode 최고 .02712m/s, 상한초과0; 전체 충돌3/1024, 전체 speed>5cm/s4회(실패). 기존 25회 초과 대비 감소하지만 실제 안전 보장은 아님.
- 17초 horizon 성공92.38%, 실제 속도가 느려진 만큼 `--episode_s 23`으로만 연장하여 독립 xy seed8701 / sim seed8702에 재시험. 목표 5cm 중심 lift·5초 hold·실드/힘/마찰은 동일. 속도를 높여 시간 조건을 맞추지 않음.
- 그리퍼 latch 비교는93.55%로 baseline93.95%를 개선하지 못함. 기본 설정에 적용하지 않음.
- 새 최종 workspace bank는 라이브러리의 **모든 BC 샘플 + dense home 경로** FK를 검사해 방향 선택. 부분 자세의 bound는 실제 경로 subset(기록된 q[0], close, end, home)만 사용. near-radius tolerance4mm; 위치를 제거하지 않음. 동일 여유에서는 최근접 반경·작은 phi/pitch 선호.


### 2026-10-08 Codex — 목표1 저속 제어/유지 실패 진단 추가 (Human 검토 대상)
- 앞 기록의 “17초 horizon”은 정정: 원래 env deadline은 **20초**, 기존 collector 기록 길이가 17초였다. config mtime 및 첫 후보 snapshot 비교에서 task/action/physics 설정 변경은 없었다. 새 저속 시험만 23/30초로 연장한다.
- 후보 `--arm_joint_speed_deg_s 6`은 arm drive velocity_limit_sim과 IK command target 변화량을 함께 제한한다. target을 이전 safe command에서 physics_dt×cap 이내로 움직인다. 측정 관절값 기준으로 매번 제한하면 PD tracking error가 사라져 접근이 정지하므로 그렇게 하지 않는다. 접촉 충격에서 실측 속도는 여전히 상한 초과 가능: 1024 위치 시험 성공90.82%, collision6, TCP>5cm/s5회, arm>6.001deg/s38회(일부 성공은 약6.003deg/s 수치 초과). 실제 안전 보장이나 실기 적용 승인이 아니다.
- `--grasp_wait` 후보는 BC의 근접 닫기 이후 실제 held/손가락 정지 5step을 확인할 때까지 arm을 멈추고 이후 BC lift를 반환한다. 성공89.84%로 개선되지 않아 기본 모드에 넣지 않았다. 붙잡음 판정·힘·마찰·실드 여유는 완화하지 않는다.
- 전체 home/BC sampled path에서 기존 margin+10mm 이상이면 손가락 닫힘 축이 접시 법선에서10deg 이내인 방향을 먼저 고르는 `--prefer_flat` 후보 추가. phi0,pitch−15의 닫힘축은 수직 기준6.09deg, pitch0은20.46deg; 더 평평한 후보도 전체 경로 검사 필수. max clearance 선택과의 tradeoff를 명시한다. 기존 실행 selector/config에는 활성화하지 않았다.
- 이 평평한 후보+현재 BC+저속6deg/s의1024 위치 시험은 **64.94%**여서 채택하지 않는다. 한 timeout trace는 held=true, tilt≈5.7deg 상태에서 lift가 약50~56mm를 오가며5cm 기준 연속5초 hold가 끊기는 것을 보여준다(`rl/logs/codex_goal1_flat_failure_trace.log`). grasp와 성공 hold를 구분하며 peak lift만으로 성공을 세지 않는다.
- 이를 위한 추가 후보: 같은 안전 조건(명령TCP.04m/s, lead2mm, joint6deg/s)에서 전문가 lift .11±.01m/30초 기록 → BC 새 실험. 기존 성공 조건(중심5cm/tilt25deg/plate speed/5초 hold)은 그대로. 시연별 실제 TCP≤.05m/s 및 관절≤6deg/s(+.01deg/s 수치 tolerance) 선별 옵션 추가; 과거 관절 속도 기록이 없으면 저장 next_obs[8:14]의 scale0.5를 역산한다. 지표/데이터선별 변경은 Human 검토 대상. 원본 데이터와 이전 모델 유지.
- calibration/config.yaml을 Codex가 새 후보로 전환하지 않는다. 15:05 Claude의 Human 지시 연결은 별도 기록이며, 그 후보도 전체 영역95%/실측 속도 게이트를 통과한 것으로 취급하지 않는다. 실제 adapter/TCP·손 안 접시 pose는 아직 측정이 필요하다.

- 평가 지표 보완(Human 검토): gate의 auto-reset이 terminal collision 상태를 덮어쓰기 전에 TCP/quaternion/관절속도를 캡처하도록 수정. 이후 max speed는 terminal sample도 포함하고 tool rotation10deg/s 초과도 계수한다. 이전 결과는 terminal movement를 누락할 수 있던 구 지표이며 안전 증거로 승격하지 않는다. 새로운 highlift gate에서 terminal_speed_samples를 검증한다.

- highlift BC3 seed는 같은flat tuning bank에서91.21/90.82/89.45%로95% 미달. 추가 후보는 접근 IK handoff를40→4mm로 옮기고 actual jaw-stall 확인 후 BC lift(기존 opt-in grasp_wait)를 시험한다. 명목 rim inset15mm에서4mm shortfall이면 약11mm 들어간 파지가 목표다. 접근/닫힘 sequencing에 주는 도움을 명시하며 “BC가 전체 궤적·파지를 스스로 발견”한 결과로 해석하지 않는다. 실제 contact/held·5cm/5초 조건은 그대로이고 붙임/순간이동/force 상향은 없다. 기본 설정/실기 연결은 변경하지 않는다.

### Codex — 인계 라이브러리 재생과 live BC 평가 분리(Human 검토 대상)
- 기존 live BC uniform gate가 실기 `plan_from_library` 동작을 그대로 평가한 것은 아니었다. 실기는 canonical 반경에서 성공한 BC joint path를 선택하고 joint1 azimuth 회전 후 mover로 재생한다. 둘을 별도 metric execution_mode로 구분한다.
- sim opt-in `--replay_library` 추가: source row 성공 여부 확인, recorded q_deg에 joint1 az를 더하고 physical drive의 joint target으로 전달. 기존 voxel shield와 rate limiter 유지; arm/plate 상태 teleport/attachment 없음. measured joint tracking0.1deg 이내에서 path cursor를 진행하고, recorded close 구간에서는 실제 held/jaw-stall5step 확인 후 진행. 저장 success 자세에서 정지하더라도 실제 중심lift5cm·25deg·plate speed·5초 기준으로 판정한다. 이는 정책 일반화나 스스로 배운 경로가 아니라 BC가 만든 궤적의 재생 검증이다.
- 새로운 seed8801의uniform xy1024/sim8802에서 전체 bank 그대로(충돌 위치 제거 없음) 시험. command joint5deg/s로 수치 여유를 확보하고 deadline90초로 늘림; 기존6deg/s·TCP5cm/s·회전10deg/s 합격기준은 올리지 않는다. home→pre는 별도 FK/retiming 검증이며 gate는pre부터 검사한다. 원래cfg/실기 selector는 전환하지 않았다.

- 라이브러리 replay 첫 독립 시험(seed8801/8802)은968/1024=94.53%, collision2/lost12/timeout42, shield0. TCPmax.02510m/s/5cm/s 초과0, rotationmax4.864deg/s/10deg/s 초과0, 관절max10.741deg/s/6deg/s 초과14. 성공 episode max관절5.014deg/s였어도 실패에서의 초과를 제외하지 않으며 전체 안전 gate는 미달이다.
- timeout42 중30은 held가 한번도 안 됨. source first-close q의 FK 림 거리(3.04~6.26mm)는10mm 안이므로 “멀리서 일찍 닫기”라고 단정하지 않는다. 저장 raw close 시점과 실제 손가락이 안정적으로 잡은 시점이 미세하게 다를 수 있어 candidate library에 `close_command_step`, `grasp_step`, 전체 `gripper_close_command`를 추가 기록한다.
- opt-in `--confirmed_grasp_waypoint`: held true, 실제 finger 변화<1e−3를5step 연속, 접시 lift<1cm인 **들기 전** snapshot을 close waypoint로 지정. 이를 찾지 못한 row는ok=false. 기존 라이브러리는 덮어쓰지 않으며 default first-command 동작은 유지. 새 library는 highlift seed1 + IK4mm/settlewait + joint5deg/s/60s +5s hold. 이 후보부터 다시 전체 FK 방향 선택 후 fresh seed9001/9002에서 재생을 시험한다. 이것은 추가 티칭/피드백 제어가 있는 궤적 재생이며 BC live 일반화 향상으로 계산하지 않는다.

### 2026-10-12 Codex — 바구니 BC 재개 전 필수 코드 점검
- `basket_round2.log`의 BC 3 seed는 안착 성공 0회다. 로그의 분모 7/1/11은 grasp 성공 수가 아니라 BC가 gripper를 열어 `live`에 남은 수이므로, 다음부터 grasp/open/placed-success를 전체 episode 기준으로 별도 기록한다. 기존 기준 44/60과 BC 수치는 소급 변경하지 않는다.
- 현재 `closed_loop_insert`의 retry/backoff는 관측에 없고, low 오차 0.3초 평활화도 없다. 상태전이 기준이 주석(0.5초)과 코드(`stuck > 1.0`) 사이에 불일치한다. C1 전문가를 수정할 때 슬롯 오차와 low를 사용하고 모든 행동 선택 상태는 관측 가능하게 한다.
- `--closed_loop --record`의 기록 경로는 미연결: `_record` 설정이 없고 closed-loop tuple이 기존 dict 기반 저장기와 맞지 않는다. 이 상태의 결과로 새 BC를 수집하지 않는다. C3 전에 독립 recorder/schema 및 저장물 검증을 마련한다.
- 현재 BC는 관절 위치 포함 입력과 관절 Δ 출력이다. C2에서 slot-frame TCP 행동·관측으로 바꾸고 모델/데이터를 신규 경로에 둔다.
- basket expert/BC eval에서 SafeDiffIK TCP·회전 제한은 코드에서 확인했으나 관절 20°/s cap 설정 호출은 찾지 못했다. 속도 계측 gate를 통과하기 전에는 저속 상한 준수로 간주하지 않는다.
- 검토 당시 Codex 명령 실행 환경에서 `/dev/nvidia*`가 없고 `nvidia-smi`가 driver와 통신하지 못했다. `lspci`에서는 RTX 5070 PCI 장치가 보였고 Isaac 번들 Python의 torch 2.7.0+cu128도 `cuda_available=False`, device_count=0 및 NVML 초기화 경고를 반환했다. 이는 GPU가 물리적으로 없다는 뜻이 아니라 이 명령 실행 컨텍스트에 GPU device/driver 접근이 전달되지 않은 상태다. 새 Isaac 결과는 생성하지 않았고 GUI/학습 실행은 GPU passthrough가 있는 환경에서 해야 한다. 기존 바구니 성공기준과 속도 상한은 완화하지 않는다.

### 2026-10-12 Claude — 목표 2 실기 방식과 학습 위치
- 넣기는 학습 없이 **기하 보정 + 티칭**으로 먼저 성공시킨다(Human 확정). 바구니 위치 = 마커(정합 xy 잔차 ≤ 8 mm, 튀는 마커 1개 자동 제외), 손 안 접시 = 잡을 때 FP 중심(방사 방향 가정은 35° 틀렸음), 둥근 접시라 법선 둘레 회전은 공구 변화 최소로.
- 넣기 동작 값은 Human 실기 판단으로 정함: 기울임 15°(접시 윗부분 +x), 핀 줄 방향 +4 cm, 깊이 티칭보다 2 cm 위, 핀 안 0.25°/s, 첫 하강 1.5°/s. 성공 판정은 Human 눈.
- 바구니 실제 형상(빗살: 레일 4.9 cm, 끝 약 11.5 cm, −x 12.5°)을 sim `rl/basket.yaml`에 반영. sim 넣기 BC는 이 형상과 실기 값으로 다시 맞춘다.
- 학습은 **넣기 직전 mm 보정과 접촉 판단**(손목 카메라 + 힘)에 쓴다. 운반·바구니 위치·도달 판단은 계산(경로 계획, IK 후보 탐색)이 맞다. 도달 한계는 학습으로 없어지지 않음(무작위 시드 60개 IK로 확인).
- 그리퍼 자동(`--auto`): 2D bbox 겹침 대신 로봇 자세 + FP 접시 이동(≤ 15 mm) + Robotiq 물체 감지. 그리퍼는 비상정지 회로 밖이라 로봇 정지 상태에서만 명령.
- 컨트롤러 툴 무게 설정(1.45 kg 추정)은 직접 교시 때 앞으로 쏠려 되돌리기로 함(Human). 정확한 질량·무게중심 없이 다시 넣지 않는다. 컨트롤러 설정 변경은 Claude가 직접 실행하지 않고 Human이 실행.

### 2026-10-12 보스 — 호칭
- 앞으로 공유 문서와 대화에서 사용자를 'Human' 대신 **'보스'**로 적는다. 이전 기록은 고치지 않는다. Codex도 새 기록부터 같은 호칭을 쓴다.

### 2026-10-12 19:02 Codex — night dataset qualification hold
- Forced TCP attachment + all-true collision shield + changed success predicate in current `rl/basket/expert.py` means new v2 data cannot be counted as successes under previous physical/safety gate. Preserve them for diagnostics and mark surrogate-only. Pending dedicated slot-definition and physical validation, do not automatically train/promote them as qualified demonstrations.
- Parent night-chain PID3037016 SIGSTOP, child collectors retained. Approval for device-access read-only host query succeeded; Codex host GPU access is possible via elevated execution. Detailed findings and exact audit path in LOG.

### 2026-10-13 Codex — 야간 물리 진단 판정(보스 검토 대상)
- 메인Claude모델/데이터/실기config는수정하지않고별도candidate에만실측속도·회전·arm접촉gate와release후매step1초정지검사를추가.전체pathgate를대체하지않는다.
- verified16회에서raw삽입1,추가gate통과0.기존surrogateBC/RL결과는조건부학습결과로보존하며물리/safety성공과동일하게보고하지않는다.
- 향후수집전운반전체path안전검사및손안접시회전/접촉모델을해결해야한다.실드·성공조건을완화하거나attach결과를물리성공데모로승격하지않는다.세부summary와계측원본경로는LOG기록참조.

## 2026-10-13 RL 로그에 평균 에피소드 리턴 추가 (보스)
- 결정: `train.py rl`에 `episode_return`(끝난 에피소드의 할인 없는 보상 합 평균), `episode_len_s`, eval 시 `eval_episode_return` 기록. 기존 `reward`(step당 평균)는 유지하되 판단 지표 아님.
- 이유: RL 목적함수는 기대 리턴 E[Σ γ^t r_t]이고, 그 표본 추정이 평균 에피소드 리턴. step당 평균(0.2)은 ±10 종료 보상과 비교가 안 돼 읽을 수 없었음.
- 해석: 리턴은 shaping 보상 포함이라 성공률과 완전히 같지 않음 → 판단은 eval_success + eval_episode_return 함께.

## 2026-10-13 하강 전 운반 속도 2배 (보스)
- 결정: to_grasp, lift, to_above를 6 → 12°/s(`plate_ik_bc.vel_transport_deg_s`), mover 하드 상한 `VEL_HARD_LIMIT/ACC_HARD_LIMIT` 6 → 12. 하강(정책 1.2, 티칭 1.5/0.25), 상승 1.5, 홈 복귀 등 나머지는 그대로(6 이하).
- 이유: 보스 "하강 이전에는 3/10 → 6/10". 10/07의 "1/10 → 3/10"(2 → 6°/s)과 같은 척도.
- 시험: tests.py 상한 거부 값 10/7 → 13으로 수정, 21개 통과.

## 2026-10-13 저녁 속도 통일, mover 상한 해제 (보스, 비상정지 대기)
- 결정: mover 하드 상한 삭제(> 0, 유한값만 검사). 접시가 내려가는 구간만 느리게: 첫 하강(descend_pre, to_policy_start) 1.5 → 1.2, 티칭 넣기 0.25 → 0.35, 정책 넣기 1.2 → 1.5°/s. 나머지 모든 구간(잡기 이동, 들기, 운반, 상승, 홈, 중단 복귀)은 `vel_deg_s` 15°/s 하나로 통일(`vel_transport_deg_s`, `basket_ascend_vel_deg_s` 삭제).
- 이유: 보스 "접시 하강만 조금 느리게, 나머지는 지금보다 조금 빠르게 통일, limit 해제", "넣는 것도 조금 더 빨라도". 컨트롤러 안전 설정(속도·충돌)은 별개로 유지.
- 시험: tests.py 상한 거부 → 음수·nan 거부로 수정, 21개 통과.

## 2026-10-13 자동 실행 spline 묶기, 1000줄 제한 해제 (보스)
- 결정: `--auto --insert policy`에서 그리퍼 정지점 사이 구간을 spline 하나로: to_grasp | lift+to_above(carry) | to_policy_start+policy_descend(insert) | policy_ascend+to_home(retreat). `plate_plans.merged_plan`(연속 계획 파일의 구간을 이어 붙이고 0.05° 중복 제거, 100점 초과면 구간별 실행). 이 모드에서는 들기 후 3 s 정지와 손목 사진 2장 생략. config `auto_merged_legs: true`.
- 속도 "조금 낮춤": 일반 30, 정책 시작+넣기 5.0, 첫 하강 4.0, 티칭 넣기 1.2°/s.
- 코드 1000줄 제한 해제(보스). `policy_plan`은 pipeline → plate_plans로 옮김(이미 진행된 정리).
- 시험: 16:58 실행 데이터로 carry 14점, insert 85점, retreat 22점 모두 mover 검사 통과, 회귀 시험 21개 통과.

## 2026-10-13 Codex — 손목 핀 검출·마커 비교
- pins는 저장사진 진단 전용. 미확정/가림/모호한 핀의 보정량은 None이며 로봇경로에 자동 반영하지 않는다. 화면회전과 모델ID를 결과에 보존;분면은plate_plans와 같은 바구니frame(+y왼쪽). 화면핀번호→삽입gap 매핑은 별도 검증한다.
- 마커크기 override는 메모리내 실행설정만 변경. baseline98mm,5cm/2.5cm는실물교체후측정. 인식률·repeatability·피팅잔차와 절대정확도를 구분한다.

## 2026-10-14 Codex — 핀 gap 중심 진단값 정의(보스 검토 대상)
- 확정되고 화면 번호가 부여된 핀 쌍의 `gap_centre_base_xyz`는 두 **모델 핀 끝**의 base 좌표 중점에 영상에서 확정된 basket +x 보정량을 더한다. 거부·가림·화면 밖 쌍은 null. 이 값은 모델·카메라/hand-eye·핀 높이 가정에 좌우되는 진단값이며 실기 이동 명령에 연결하지 않는다.
- 이유: E의 출력 계약을 명확히 하고 미검출 쌍이 모델 좌표로 조용히 대체되는 일을 막기 위해서다. 실제 핀 기준 absolute mm 정확도나 번호 매핑은 D 전체 라벨·보스 시각 검토 뒤에만 판단한다. Q3 저장 사진 계약 검사에서 번호 있는 확정1/화면 밖·거부39를 확인; E 목표80% 성능 주장은 아님.

## 2026-10-14 Codex — 저장 사진용 실측 부분 배치 투영 옵션(보스 검토 대상)
- `camera_tools.py pins --layout <pin_layout_measured.json>`은 확인된 root/tip의 바구니 좌표를 저장 사진에 투영하고, 두 줄의 x가 8mm 이내인 핀끼리만 잠정 쌍으로 묶는 **오프라인 진단 옵션**이다. 기본 모델 경로는 그대로 둔다. 같은 Q1~Q4 사진으로 만든 부분 배치를 같은 사진에 다시 쓰는 시험은 정보 누출이 있으므로 독립 검출률/정확도 평가가 아니다.
- 부분 v9는 Q2에서 한 줄만 확인되고 클립·가림 핀이 빠져 있다. 이 옵션 출력은 로봇 경로나 성공 판정에 연결하지 않으며, 최종 성능 판정은 완성된 D 정답과 별도 사진 검증 후 보스 검토를 받는다.

## 2026-10-14 바구니 모델: 가운데 클립 쪽 빗살 2개 추가 (보스 실기 판정)
- 근거: 10/14 실기 Q3 `--pin 5` 넣기. 보스가 로봇에서 먼 쪽(클립 쪽)부터 세니 실제 빗살은 줄마다 13개였고, 접시는 7~8번 사이에 들어갔다. 모델은 11개였고 계획상 5~6번 사이였다. 들어간 위치(바구니 x −0.133)는 모델 계획과 맞으므로, 모델에서 클립 쪽 빗살 2개가 빠져 있던 것이다.
- 변경: `rl/basket.yaml full.centre_gap_x` 0.025 → 0.0. 가까운 분면 Q3·Q4는 빗살 13개(x −0.243 ~ −0.003), 틈 12개가 된다. 먼 분면 Q1·Q2는 빗살 12개(+0.017 ~ +0.237)이지만 실물 개수는 아직 확인하지 않았다.
- 영향: 이 바구니 모델을 쓰는 sim 학습 환경(rl/basket)도 바뀐다. 이전 정책은 클립 쪽 빗살 없이 학습했다.

## 2026-10-14 빗살 번호 규칙 (보스 슬라이드)
- 결정: 분면마다 **1 = 로봇에서 가장 먼 빗살**, 로봇 쪽으로 증가. Q3·Q4는 클립 옆이 1, Q1·Q2는 바깥 끝이 1. `--pin N` = N번과 N+1번 빗살 사이.
- 근거: 보스 슬라이드 `calibration/data/basket/boss_slide_20261014/tine_numbering_boss.pdf`(전체 사진에 분면별 빗살·번호). 코드 `plate_plans.pin_gap`이 이미 이 규칙(먼 쪽 → 로봇 쪽). 작업판 N의 '클립 쪽부터'는 Q1·Q2에서 틀려 정정.

## 2026-10-14 15:5x 바구니 모델: Q2~Q4 빗살 간격·첫 빗살 위치를 실측 라벨로 교체
- 결정: `rl/basket.yaml` tine_pitch Q2 0.01756 / Q3 0.01740 / Q4 0.01787 m(이전 0.018 / 0.0192 / 0.0198), clip_first_m을 분면별로 Q2 0.0098 / Q3 0.0114 / Q4 0.0150 m(이전 전부 0.010). Q1은 미측정(3 cm, 0.010 유지).
- 근거: 보스 번호(슬라이드) + Claude 초안·Codex 검증 손목 라벨(10/13 사진, 카메라 자세 저장) → 3D(`calibration/data/basket/wrist_look/tines_3d_claude_20261014.json`) → 분면별 두 줄 최소제곱. 새 모델 대비 실측 rms Q2 1.8 / Q3 2.2 / Q4 2.9 mm(이전 모델은 Q3 평균 +6~+11 mm, 줄 끝으로 갈수록 커짐).
- 영향: 실기 분면 넣기(`--insert quadrant`)의 목표 위치가 바뀜(Q3 핀 7 목표가 클립 쪽으로 약 9 mm). 오전에 맞춘 x 보정 −2 cm는 이전 모델 기준이었음 → 다음 실기에서 재확인 필요. 줄 y(쌍 중심·줄 간격)는 아직 그대로(Q4 줄 간격 측정 7.9 cm vs 모델 9.5 cm, 확인 필요).

## 2026-10-14 19:0x 배치 변경과 오늘 밤 학습 순서 (보스)
- 결정: 접시와 바구니를 **나란히** 두고, 바구니를 앞으로 당긴다. 기준은 바구니 (0.807, −0.192), 접시 (0.848, +0.339). 16:5x '접시를 바구니 앞에'는 대체한다(앞에 두면 로봇에서 0.86 m 안쪽이라 잡기 경로가 없음).
- 근거: 고정 D455 측정에서 4분면 전 핀 IK 가능(Q1 6, Q2 8, Q3 12, Q4 12). 바구니가 x 0.93~0.99일 때는 Q1/Q2가 접시 면 +x로 IK가 안 나왔고, 접시가 y ±0.39에 있으면 들어 올린 직후 프레임 기둥 여유가 0.0~0.4였다.
- sim 흔들기 범위: 바구니 x·y ±5 cm, 회전 ±10°. 접시 x −4~+8 cm, y −8~+2 cm. 근거는 `collab/SYSTEM_SPEC.md` 6절.
- 학습 순서: 1 경로(플래너 → BC 데이터) → 2 아무 칸 안정 배치 → 3 지정 빗살. 한 코드 흐름에서 목표만 좁힌다. 2가 되어야 3이 의미 있음. 종료 조건은 작업판 '오늘 밤 계획'.
- 플래너: `quadrant_plan`이 접시 면 −x(손목 뒤집힘)도 시도한다(c57cc6c).
- 점검·LOG 주기: 1시간 → 30분(050412d).

## 2026-10-15 00:3x 분면 넣기(sim) 최종 설정과 성공 판정 (Claude, Codex U 반영)
- 성공 판정: **앉음 & exact** = 최저점 < 60 mm(레일 49 mm) + 자기 분면 칸에 똑바로(비틀림 < 15°, 기울기 < 빗살 + 12°) + 목표 칸(핀 N = 빗살 N과 N+1 사이). 이전의 느슨한 in_quadrant/exact는 빗살 위에 걸친 접시도 통과시켜(Codex U) 보조 지표로만 쓴다. 'Strict sim'(예전 한 칸 슬롯 기준)은 분면 목표에 맞지 않아 쓰지 않는다.
- 설정(조정 경로 seed 4014/5014에서 고르고, 처음 보는 6014/7014로 검증):
  - Q1: 손목 뒤집기, 위쪽 다시 잡기 끔, 놓는 기울기 = 빗살 × 0.5, 겨냥 +2.5 cm
  - Q2: 손목 뒤집기, 기울기 × 0, 겨냥 +3.8 cm
  - Q3: 기울기 × (−0.5), 겨냥 −4.64 cm
  - Q4: 기울기 × 0.5, 겨냥 −3.5 cm
  - 모두: 바구니 위 정지 상태에서 손에 든 접시 자세 측정 → 다시 겨냥 2회(`--cl_iter 2`).
- 근거: 빗살 기울기만큼 눕혀 놓으면 놓은 뒤 일어서지 못하고 걸린다(Q2는 한 번도 안 앉음). 먼 분면은 접시 앞면 방향 때문에 바깥으로 눕는다(손목 뒤집기로 해결, 보스의 Q2 티칭과 같은 자세). Q1(3 cm 간격)은 위쪽 다시 잡기의 회전으로 줄을 따라 굴렀다.
- 결과: 처음 보는 배치 0.990 / 0.983(LOG 10/15 00:3x). 실기 이식은 보스 결정(Codex Z 문서).

## 2026-10-15 02:5x 실기 이식 후보: 손 안 접시를 재지 않는 다시 겨냥 (Claude 제안, 보스 결정 필요)
- 근거(sim): 바구니 위 손 안 자세 측정에 실측 FP 잡음을 넣으면 0.983 → 0.808. 거르기 5 mm를 붙이면 0.908. 측정 없이 분면 평균 손 안 자세를 쓰고 겨냥을 다시 맞추면 0.953 / 0.952(처음 보는 9014 / 7014).
- 제안: 실기 첫 시험은 '재지 않음' 방식으로 한다. 분면별 평균 손 안 자세는 실기에서 접시를 잡아 들어 올린 직후, 정지 상태에서 FP 여러 프레임(이상값 제외)으로 한 번 측정해 정한다. 바구니 위 실시간 FP 다시 겨냥은 측정 꼬리(10~30 mm 튐)가 해결된 뒤에 쓴다.
- 미정: 실기 평균 자세 측정 절차, Q1 운반 중 미끄러짐(sim y 1.5 → 9.9 mm), 실물 앉음 높이(Codex AF 기록지).

## 2026-10-15 오후 학습 방향: 단계형 PPO → 사전학습 비전 인코더 (보스 대화)
- 정의: 10/14 밤 sim 작업은 학습이 아니라 **Isaac 피드백으로 Claude·Codex가 규칙을 넣고 파라미터를 고친 튜닝**이다(경로 = 수치 IK + RRT, 신경망 없음).
- 장기 목표: vision-based RL로 detection + pick-and-place 범용 자동화. task를 조금씩 어렵게 하며 점진적으로 간다.
- 구조: 1 파지 위치 이동·3 놓을 위치 이동 = IK/RRT. PPO는 **4 정렬 → 5 넣기 → 2 파지** 순서.
  - 4 정렬: 접시 아랫면이 같은 번호 두 줄 틈(L_N, R_N)을 지나는 두 점의 오차 + 기울기 차이 → dense 보상. 행동은 틈 좌표계 TCP 미세 이동(IK가 관절로 변환).
  - 4→5 넘김: 두 점 오차 2 mm·각도 2° 이내 1초 유지(결과 보고 수정).
  - 5 넣기: 목표 칸에 앉으면 +1, 실패 −1(sparse).
  - 2 파지: 진입 깊이·닫는 시점만 학습, 힘 고정. 잡고 유지 +1, 놓침·미끄러짐 −1.
- RL은 작게 시작: 현재 튜닝 계획기(sim 99.7%)를 선생님으로 데모 → BC → residual PPO. 비교 기준은 튜닝 값과 단순 비례 제어기. 나쁜 조건(FP 잡음·손 안 편향·단단한 빗살) 고정 시험 세트에서 분명히 나을 때만 채택.
- 관측: 지금은 벡터(6D pose + 실측 잡음 모델). 이미지는 **사전학습 인코더 고정(ResNet18/DINOv2 등) + PPO는 MLP만** 방식으로 넓힌다(보스 결정). sim 안에서 FP 실행은 GPU(RTX 5070 12 GB) 때문에 하지 않는다.
- 선행 조건: Q1 빗살 간격 실측, Q4 줄 간격 확인(실측 7.9 cm vs 모델 9.5 cm). 두 값이 틀리면 정렬 보상이 틀린 정렬을 가르친다.
- 비전 모델의 역할(보스 15:xx 보충): 바구니 안 **장애물(이미 거치된 접시 등)을 비전으로 인지해 피하면서 지정 칸에 거치**하는 단계에서 쓴다. 이 단계가 연구 부분이고, 지금(빈 바구니, 6D pose 벡터)은 엔지니어링 단계다.
- 상태: 빈 바구니 BC는 작업 AO(`collab/PLAN_2026-10-15_basket_bc_rl.md`)로 Codex에 넘김.

## 2026-10-15 AO/G0: 기록 행동에도 실행기의 벡터 속도 상한 적용
- 결정: `expert.py --g0`가 teacher의 FK 차분을 틈 좌표계로 바꿀 때 XYZ와 회전 XYZ 각각을 성분별 clip하지 않고, 각 3-vector의 norm을 `config.yaml action.step_max` 상한으로 줄인다.
- 이유: `SafeDiffIKAction.process_actions`는 한 정책 스텝에서 TCP 이동량과 회전량을 각각 벡터 norm으로 제한한다. 축별 clip만 하면 대각선 행동이 최대 √3배가 되어 저장 라벨과 실제 적용 행동이 달라질 수 있다. 각 성분 상한은 기존 설정 그대로이며 높이지 않는다.
- (10/15 16:1x Claude 수정) 위 물리 finger-stall + TCP-to-rim 조건은 **진단 전용**이며 G0 분모에서 제외한다. G0는 기존 `seated_exact`를 그대로 쓴다: `0.020 < low < 0.060 m` AND 기존 `in_quadrant` 조건 AND `landed_gap == target_gap`. 이 변경은 기준 완화가 아니라 앞서 추가된 grasp-qualified 조건을 철회하고 기존 판정과 일치시키는 것이다. lift > 5 cm는 기존 `in_quadrant`의 조건으로만 유지한다.
- G0 산출물에는 release 후 raw `low`/`seated`/`in_quadrant`/`exact`을 teacher와 SafeDiffIK 양쪽에 저장한다. finger closure와 TCP plate-local 좌표·rim 거리/오차는 lift와 insertion trace에 남겨 기존 진단 기준의 sim 적합성을 분리해 본다. 같은 시작점 복원 오차, recorded-vs-replay TCP 위치·자세와 관절 오차, 최초 open-command frame도 에피소드별로 남긴다.
- G0 기록 전처리는 frame 길이/shape, finite 값, nonzero quaternion, binary release label, per-axis 설정 일치도도 검사하고 잘못된 기록을 거부한다.
- 확인: 대각 이동·회전 상한, finger/rim 진단, 기존 `seated_exact` 분모, 시작 복원·replay 오차 요약, env별 open label, NaN/frame 불일치 회귀시험. 수정 후 `bash calibration/run.sh tests` **51/51 통과 (32.4 s)**. G0 teacher 실행은 최종 `run_quadrant_final_20261015.sh` 인자 전체로 해야 한다.
- 보스 검토: 이 선택은 teacher 차분을 기존 실행기 상한에 맞추는 기록 전처리이며, G0 재생 결과와 함께 확인한다.

## 2026-10-15 AO/G0: opt-in teacher joint-target rate cap
- `expert.py`에 `--teacher_rate_cap` 옵션을 둔다. 켰을 때만 `drive()`가 waypoint 관절 목표를 retime해 30 Hz 프레임별 목표 변화량이 해당 phase의 기존 `speed/30` 이하가 되게 한다. phase 전환은 직전 명령 목표에서 이어진다. 기본 실행 경로는 이전 보간식을 그대로 쓴다.
- 이는 PD **목표 갱신률** 제한이며 실제 관절 물리 속도 보증으로 해석하지 않는다. phase 값은 변경·상향하지 않고 safety shield·충돌물리·`seated_exact` 성공 기준도 그대로 둔다.
- capped teacher의 seated_exact와 capped+shield joint-reference 재현은 새 산출 경로의 진단이다. 높은 joint-reference 일치율만으로 SafeDiffIK TCP G0 통과나 BC gate 통과를 선언하지 않는다. SafeDiffIK G0는 기존 ≥95% 조건으로 별도 확인한다.

## 2026-10-15 AO/G0: rate-capped teacher 결과와 shield 유지
- 결과 검토 전 결론: 학생 재생의 safety shield를 끄지 않는다. 계획기 경로 검사와 프레임별 shield는 다른 안전 검증이고, G0 행동계약에서도 shield는 실행 일부다. shield를 통과하지 못한 경로는 현재 실행 스택에서 실행 가능하다고 입증되지 않은 것으로 남긴다.
- Q1 동일 seed/plan 비교에서 현재 teacher target-slew profile(transport/reaim/insert/release 20/5/1/20 deg/s)은 seated_exact **10/20**으로, uncapped teacher 18/20보다 낮았다. 따라서 이 속도 프로파일로 AO teacher gate를 만족한다고 할 수 없다. 다른 구간 cap으로 17/20을 보장하는 수치는 한 번의 profile 결과에서 도출할 수 없고, 기존 제한을 올리는 값은 제안하지 않는다.
- `--teacher_rate_cap`은 30 Hz PD **목표 변화량** 제한이다. rate-cap 결과에서 목표 변화는 최대 cap의 0.9991배였지만, 실제 sim 관절 속도는 insert 구간 최대 2.27 deg/s로 설정한 목표 slew 1 deg/s보다 높았다. 이는 목표값 제한이 물리 관절 속도 제한과 같지 않음을 보인다. 실기 속도에 대한 주장은 아니다.
- 다음 원인 분리는 contact/plate-pose 시계열·open frame과 phase별 관절/TCP trace가 있어야 가능하다. 그 자료 없이 low 66–68 mm를 접촉 걸림 또는 release 타이밍 중 하나로 단정하지 않는다. gate 미달 동안 G1/G2는 시작하지 않는다.

## 2026-10-15 AO/G0: ratecap2 후속 판정
- `ao_g0_joint_ratecap2_20261015_2131/Q1_g0`에서 teacher 11/20, joint-reference 7/11, shield block 2,458이다. JSON/NPZ seated_exact mask는 20/20 일치한다. 새 cap 설정이 ≥17/20 gate를 만족한다는 근거는 없고, joint-reference는 SafeDiffIK gate가 아니다.
- Runtime shield 유지. 이를 끄는 것은 AO 실행 안전 조건을 바꾸며, 현재 자료로는 shield가 teacher의 일곱 correct-gap/high-seat 실패 원인이라고 할 수 없다. 일곱은 low 66.2–67.8 mm, 두 건은 gap −1/low −16.4, −16.9 mm다. contact/plate-pose trace 없이 충돌·경로·release 중 원인을 택하지 않는다.
- 고정된 phase cap 20/5/1/20°/s 아래서 17/20을 달성하는 새 프로파일은 미확정이다. 제한을 올리거나 shield를 우회해 수치를 맞추지 않는다. 다음 원인 분리는 plate pose/contact 및 손-held target trace를 저장하고, 시작 상태 settle 확인 뒤 기존 cap/shield 그대로 재현해야 한다.
- JSON/helper의 18 clipped frames는 이전 명령 target을 기준으로 재계산해 확인됐다. 보존된 ratecap2 NPZ의 `joint_action_clipped` 합 29는 과거 writer의 잘못된 기준(직전 teacher target) 때문에 생긴 11-frame 불일치다. 원본 산출물을 고치지 않고, 향후 산출물 계산을 수정하고 회귀시험을 추가했다.

## 2026-10-15 AO/G0: insertion-end depth sweep
- Teacher target-slew는 기존 20/5/1/20°/s와 runtime shield를 그대로 둔다. 이번 비교는 `closed_loop_paths(extra=...)`의 마지막 plate-bottom 목표만 기본 5 mm에서 10 mm/15 mm로 더 깊게 한다. 20 mm 이상 등 다른 깊이는 이 진단에서 허용하지 않는다.
- Claude가 각 후보를 새 경로·동일 plan/seed로 순차 실행한다. Teacher `seated_exact` ≥17/20이 후보 선정 기준이고, 이후 joint-reference 재생은 실행기 진단일 뿐 SafeDiffIK G0 gate를 대체하지 않는다. 두 깊이 모두 기준 미달이면 결과를 기록하고 원인 추적에 필요한 contact/plate-pose trace를 추가한다. 속도 상향이나 shield 해제는 하지 않는다.

## 2026-10-15 AO/G0 재개: 바구니 회전 hold + 추적형 SafeDiffIK replay
- 결정: generic `SafeDiffIKActionCfg.rotation_hold` 기본값은 꺼 두고 `basket_env.make_basket_cfg()`에서만 켠다. 바구니 실행기가 쓰는 spatial delta 규약에 맞춰 `hold_quat ← δ ⊗ hold_quat`으로 목표 회전을 적분한다. TCP 회전 속도 상한은 기존 10°/s 그대로다.
- G0 replay는 저장된 고정 FK delta를 반복하지 않고, 매 프레임 실제 학생 TCP 현재 pose에서 선생님 trace의 다음 절대 TCP pose까지 slot frame 오차를 계산한 후 기존 5 cm/s·10°/s step norm 제한으로 자른다. 그 실행 action은 `replay_act`; 선생님 상태에서 만든 원본 clean action `act`와 분리 저장한다.
- 회전 hold가 켜진 basket 환경에서 G0 tracking mode는 매 스텝 현재 TCP를 hold target의 기준으로 재설정한 뒤 그 step 오차를 적용한다. 이는 tracking correction을 held target에 중복 적분해 lag를 두 배로 만들지 않게 한다. 보통의 basket incremental action에서는 지속 hold가 유지된다.
- BC collector는 매 30 Hz마다 현재 sim 관측으로 오차를 재계산해 clean label을 저장한 뒤 DART 잡음만 실행 행동에 더하는 feedback 방식이다. `run_collect()`/`run_realplan()`/PPO env에도 basket tracking mode를 켜 이 action을 현재 TCP 기준 보정으로 실행한다. 단, `run_collect()`은 접시 `attach()`를 아직 호출하므로 AO 요구의 attach 없는 물리 수집을 G1 전에 고쳐야 한다.
- 수정: `rl/rl_utils.py`, `rl/plate_env.py`, `rl/basket_env.py`, `rl/basket/expert.py`, `rl/basket/train.py`, `calibration/tests.py`. 회귀: `bash calibration/run.sh tests` **54/54 통과**. G0 새 물리 gate는 미실행. 17:14 호스트 GPU에는 실시간 FoundationPose PID 639006이 남아 있어 Codex는 Isaac을 겹쳐 시작하지 않았다. G0 95% 전 G1/G2 불가.

## 2026-10-15 AO/G0: 관절 기준 재생은 별도 진단 모드로만 추가
- Claude의 최신 작업판 handoff에 따라 `expert.py --g0_joint`를 추가한다. 선생님이 각 30 Hz 프레임에 명령한 6축 PD joint target을 기록하고, 라벨은 이전에 명령한 joint target에서 다음 teacher target까지의 관절별 델타를 해당 phase의 기존 deg/s cap ÷ 30으로 제한해 만든다. 재생은 이 목표를 기존 PD target rate limiter와 safety shield에 넣고, 같은 프레임의 open label을 쓴다. attach는 금지다.
- **승인된 BC 계약이나 원래 G0 대체가 아니다.** 현재 AO 계획의 정책 행동은 slot-frame TCP delta + open이고 원 G0는 SafeDiffIK 재현이다. 관절 재생은 SafeDiffIK를 우회하므로 JSON에 `safe_diffik_g0_gate=not_evaluated_by_g0_joint` 및 진단 범위를 명시한다. joint-reference 재현율이 높아도 원 G0 통과·BC 실행 가능성으로 세지 않는다. 기존 `--g0` 경로는 유지한다.
- Q1 Isaac 실행은 Claude 담당이며 결과는 별도 경로에 쓴다. joint diagnostic threshold 95%는 재생 비교 지표일 뿐 계획 gate가 아니다; G1/G2는 SafeDiffIK G0 통과 전 시작하지 않는다.

## 2026-10-15 19:30 AO/G0 — SafeDiffIK 속도 상한을 보존하는 기록 cursor 진행
- 변경: G0 replay에서 학생의 translation/rotation correction이 기존 action cap에 걸리면, 현재 기록 TCP pose를 따라잡을 때까지 해당 recorded frame을 유지한다. 경계 timeout은 진단 flag로 남기고 replay를 재개한다; insert boundary 전에 release를 허용하지 않으며 기존 seated_exact 판정만 G0 분자/분모로 쓴다. 개별 target의 연속 clipping이 60 frame이면 진단 count 후 진행해 hang을 막는다.
- 이유: Q1 phase-gate 결과 8/18 (44.4%) 중 네 episode가 boundary timeout 뒤 open을 못 받았고, 5,991 tracking actions가 clip됐다. 소스를 대조하니 G0 tracking mode가 held target을 현재 TCP로 재기준화하고 step을 1.67 mm로 먼저 clip하므로 10 mm `hold_windup`은 이 branch에서 원인으로 작동하지 않는다. teacher stream이 capped student보다 빠르게 cursor를 전진시켰다는 가설을 검증하기 위한 재생 수정이다.
- 보존: action speed/rotation caps, safety shield, 충돌 여유 조건, physical-only/attach=false, 성공 기준, 일반 replay behavior는 불변. timeout을 통과 처리하는 것은 sequence 진행만 뜻하고 목표 성공을 의미하지 않는다.
- 검토 요청: 새 Q1 결과에서 env별 gap/low/seated_exact, release, boundary timeout, target clip timeout을 별도로 확인해 원인을 검증한다. 결과 전에는 G0/G1/G2 gate 상태를 미통과로 유지한다.

## 2026-10-15 AO/G0 opt-in 삽입 접촉 텔레메트리
- 진단 옵션 `expert.py replay --g0 --g0_telemetry`에서만 Plate↔Basket 필터 PhysX contact sensor를 켠다. 기본 환경/파지 replay의 센서 구성은 바꾸지 않는다.
- 30 Hz 삽입 control frame마다 physics update 뒤 `low`, slot 기준 `d_bottom`, plate normal 오차, 필터 접촉력 크기를 저장한다. `first_contact_frame`은 삽입-local 0-based frame 중 힘이 1.0 N 이상인 첫 표본이며, 그 이하/무표본은 `null`이다. 시각 해상도는 한 control frame(약 33 ms)이고 1 N은 판정 임계값이지 물리 접촉 자체의 보편 경계가 아니다.
- Teacher와 SafeDiffIK replay 각각의 시계열은 NPZ에 저장하고 JSON episode에 첫 프레임·표본 수를 남긴다. Telemetry는 진단 전용이며 gate/속도·성공 기준을 바꾸지 않는다. 동일 시드·plan 해시 및 `closed_loop_extra_m`을 JSON provenance에 포함한다.

## 2026-10-16 E 검출기 실험 — 철망 단절 보강과 2D 탐색은 분리
- label-seeded local fit은 내부 최대 2개 profile sample 단절만 continuity 계산에서 연결한다. continuity/support/ridge 판정 임계값은 낮추지 않는다. 부분 라벨 기준 46/59→48/59로만 늘었으므로 이를 정식 recall이나 실제 분면 성공률로 부르지 않는다.
- basket 모델만 시작점으로 쓰는 2D Hough 검색은 `pins --model_search_2d` opt-in 진단으로 추가했다. 확인 tip hit는 8/59였지만 confirmed pair 수는 기존 model-only x 검색의 7에서 1로 감소했다. 따라서 기본 동작에 연결하지 않고, row/pair 교차혼동이 해결되고 pair 성능을 보존할 때까지 생산 경로에서 사용하지 않는다. 사람 확인 59개는 부분 ground truth라 precision/전체 visible recall은 아직 산출 불가.

## 2026-10-16 Codex 초안 — 실기 quadrant 삽입 힘 감시 (보스 승인 대기)
- 범위: `--task plate --place basket --insert quadrant`의 straight-down `descend` spline에만 CLI opt-in `--insert-force-limit-n N`을 제공한다. 기본값 `None`이라 기존 움직임에는 monitor가 붙지 않는다. mover도 `insert/descend` smooth leg만 받아들이며, lift/carry/release/retreat에는 전달하지 않는다.
- 신호와 판정: `GetToolForce(ref=DR_BASE)`의 6축 wrench에서 첫 3개 힘 성분만 써서 `||F_xyz||₂ > N`이면 즉시 `MoveStop(stop_mode=1 / DR_QSTOP)`를 요청한다. 토크(Nm)를 힘 임계값에 섞지 않는다. spline 시작 직전에도 baseline을 읽어 이미 초과했거나 응답/형식이 유효하지 않으면 이동을 시작하지 않는다. 이동 중 service 응답 실패/0.25 s timeout, 비유한값, 잘못된 shape, monitor timeout도 fail-closed로 QSTOP 요청 및 비정상 종료한다.
- 정지 후 파이프라인은 gripper open, 자동 상승/후퇴, retry를 하지 않는다. 물체가 손에 든 채 정지할지 올릴지는 보스 결정 및 사람 감독 사항이다. QSTOP 서비스 호출의 `success`만 기록하며 실제 정지거리/정지 달성을 뜻하는 피드백으로 간주하지 않는다.
- 회귀는 ROS/로봇 없이 가짜 wrench 시계열(하한·문턱과 같음·초과·센서 단절)을 helper에 넣어 stop 요청 횟수/판정을 검사한다. 이것은 서비스 주기·응답 지연·컨트롤러의 QSTOP 실효성을 실증하지 않는다.
- **25 N은 예시 파라미터일 뿐 승인된 안전 한계가 아니다.** 낮은 값도 실기에서 쓰지 말 것. 먼저 Doosan API의 외력 기준/필터링·영점·좌표계·주기·응답 실패 동작을 문서/실기 비작동 상태에서 검증하고, tool/gripper/plate/fixture 질량·가속도 및 stopping distance를 포함한 현장 위험 평가와 보스 승인이 필요하다. `GetToolForce`의 TCP·센서 의미가 확인되지 않았고 sim contact N은 실기 tool force로 환산되지 않는다. 실기 실행은 별도 사람 승인 전 금지.
## 2026-10-16 E: model-only pair candidate diagnostics remain non-actionable
- For the experimental 2D Hough detector, expose the mean row correction and per-axis disagreement on every fully fitted pair. Keep the existing strict rule: both axis disagreements must be ≤3 mm for `status=detected`; only such pairs populate the operational `correction_xy_mm` and `gap_centre_base_xyz` fields.
- A numbered, visible pair that fails that gate may include `candidate_correction_xy_mm` and `candidate_gap_centre_base_xyz` for diagnosis, but must retain `status=pair_disagreement` and `gap_centre_base_xyz=null`. Unnumbered/out-of-view centers stay null. No candidate-only field is consumed as a production target.
- Rationale: Q3 photo model IDs 53/66 each fit checked tips within 2.14/1.15 px, while row y corrections differ 12.626 mm; their candidate center happens to be close to one partial-label center, but that single checked pair cannot validate relaxing the gate. This output avoids hiding the clue without converting it to a confirmed gap.
- Human review requested: assess whether to collect exhaustive pair labels or revise row-specific basket geometry before changing the pair gate. Current v5 results and limitations are in LOG 2026-10-16 02:50.

## 2026-10-16 AO: real-pipeline insertion policy remains explicit opt-in
- Reuse `expert.insert_policy_params()` as the sole observation/action definition, with deterministic mean, canonical real-pipeline hand frame, no DR, and believed gap error zero. The policy only adds its residuals to the existing `--sim_q` quadrant trims after a fresh post-lift pose; it does not replace grasp, path planning, IK, or collision checks.
- `--insert_policy` is rejected unless the user explicitly supplies `--execute`, one quadrant, basket quadrant insertion, `--sim_q`, and an existing checkpoint. Codex does not execute this pipeline. If post-lift pose is missing or the corrected replan fails, abort rather than use an uncorrected fallback. With the option absent, the old path remains.
- Tests establish observation/action contract and CLI opt-in only. They do not validate an E2 checkpoint, hardware execution, safety, or placement success. E2 checkpoint selection and human review remain pending.

## 2026-10-16 E: model-only tip audit is a partial-label localization diagnostic
- Score model-seeded candidate tips against the 59 checked image tips only after prediction, with one-to-one image-space matching and an explicit 5 px threshold. Ignore quadrant/number for this diagnostic; therefore it cannot validate identity or 3D gap targets.
- Current output is 8/59 checked-tip hits (Q1/Q2/Q3/Q4 2/14, 0/5, 5/18, 1/22). Because visible pins are not exhaustively labelled, this cannot estimate precision or full-scene recall. Do not tune gates or claim detector accuracy from this score.
- Q1 label-seeded misses suggest continuity/support rejection, while model-only misses show a larger unresolved projection/row correspondence problem. Keep these as separate failure modes; do not treat the in-sample miss explanation as proof of the OOS cause.
- Continuity is now emitted as a diagnostic ratio without changing the 0.45 acceptance threshold or any status path. After the existing 1–2-sample bridging, Q1 has 9/14 label-seeded fits; its remaining 5 weak-line misses split into 4 continuity values below 0.45 and 1 support value 0.528 below 0.55. This explains which current gate rejected these samples, not whether changing that gate preserves precision; leave thresholds unchanged absent exhaustive visible-pin negatives.
- The follow-up `pins_projection_residual_partialGT_20261016_v1` matches each checked root to a model pin by basket-root XY only after reading the checked layout, then compares projected pixels. Treat its per-row residuals as label-assisted geometry/camera diagnostics, never as model-only detection hits or a reason to tune acceptance. The correspondence and assumed height planes are not independent truth; keep the pair gate and detector thresholds fixed until exhaustive visible-pair labels and row/number identity are available.
- A same-photo per-row translation fitted from the other checked roots can reduce held-out tip reprojection errors (most clearly Q3), but correspondences still use all checked roots and the translation is itself label-derived. This is sensitivity analysis only; it does not validate a production/OOS offset or detection. Do not update basket geometry or relax gates without a separately captured basket pose and exhaustive visible-pin/pair review.

## 2026-10-16 Codex: insert PPO experiment R1 — explicit unseated penalty
- E3@10 seed15014 CPU raw audit: Q1 exact **61/114**; failure decomposition is **20 not seated, 9 seated but outside quadrant, 24 seated in a wrong gap**. Of the 20 unseated Q1 cases, 6 nevertheless report `landed_gap == target_gap`; the current reward gives these zero because it rewards exact only when `seated`, and distance penalty is zero for matching gaps. Overall Q1 errors are not exclusively reward-caused; observations/action dynamics may also limit correction.
- Implement an opt-in reward-only experiment: `--unseated-penalty 0.25` subtracts 0.25 from terminal return whenever `seated` is false. Default remains 0, so old behavior and the 16-observation/5-action deployment hook are unchanged. No success metric, plan set, randomization, action scale, architecture, or hyperparameter is changed.
- Rationale: give explicit negative signal to the Q1 subset that currently earns zero despite failing the primary seated criterion. Risks: stronger penalty could encourage reaching any seated state over target-gap exactness or disturb other quadrants; compare quadrant tables and keep the baseline checkpoint if the same 15014 final set does not reach ≥75% or any safety-relevant regression appears.
- This is a proposed experiment, not a result or a claim that reward caused the failures. Run only from a new output directory, warm-start `e3_best.pt`, and use 9014/10014 eval plans for checkpoint selection; 15014 remains final comparison only. Isaac/GPU execution is delegated to Claude because the Codex shell has no usable GPU.

## 2026-10-16 Claude — 바구니 충돌 모델에 마커 블록 추가
- 결정: `plate_plans.basket_boxes`가 `config.basket.marker_blocks`(size 0.14, height 0.10, above 0.01)를 읽어, layout에 있는 마커마다 바구니 프레임 상자를 벽 4개에 더한다. 모든 넣기 계획(실기 pipeline·random_plans·goal1_offline)이 이를 피한다.
- 이유: 10/16 마커를 판/블록에 다시 붙인 뒤 Q4 4번 실기에서 접시가 마커판에 부딪힘(보스). 블록 볼트가 바구니 안쪽으로 내려와 있다.
- 결과: 오늘 배치에서 Q3 4·5번 넣기 계획 불가 → 월요일 목표 Q3·Q4 8·11번. 마커 위치는 layout(고정카메라 측정)을 쓰므로 마커를 옮기면 layout 재저장 필요.
- 미검증: 상자 크기는 사진 추정. 실제 부딪힌 지점(접시 vs 그리퍼 vs 볼트)은 확인 안 됨. 보스 검토 요청.

## 2026-10-17 00:50 KST Codex AT — 마커 clearance 진단 metric 계약
- opt-in CLI `--marker_telemetry`는 `BASKET_MARKER_BLOCKS=1`, planned replay (`--plans --closed_loop`), 명시적인 새 JSON `--out`에만 허용한다. `--cl_policy_out`은 같이 받지 않는다. 기존 기본 동작 및 산출 JSON 키는 바꾸지 않고, opt-in일 때에만 동명 NPZ를 추가한다.
- metric은 각 삽입 30 Hz post-physics control frame에서 plate 충돌 메시의 모든 정점과 basket-local marker axis-aligned box 사이의 최소 Euclidean point-to-box 거리다. NPZ 축은 `(episode, frame, marker)`이며 marker ID와 frame index를 별도 저장한다. JSON에는 episode별 marker 최소값과 metric 정의를 둔다.
- 이 vertex-sample 값은 정확한 triangle-surface 거리도 접촉센서도 아니다. mesh face가 box를 관통해도 정점이 box 밖에 남으면 양의 값을 낼 수 있어 접촉 부재를 입증하지 않는다. gripper 형상은 metric에 포함하지 않는다. Isaac에서 실제 좌표 변환과 출력 모양을 Claude가 확인하기 전에는 이 값으로 충돌 원인을 판정하지 않는다.

## 2026-10-18 Codex — E: 동일-x 빗살 쌍은 삽입 틈 주소가 아님 (수정 보류)
- 코드 대조에서 `camera_tools` model pair group은 같은 x 위치의 두 y줄 빗살 투영을 묶고, `_assign_gap_numbers()`는 그 x를 `basket_gaps()`의 인접 빗살 사이 틈 중심 후보에 직접 넣는다. group의 3D 중점도 두 빗살 끝점의 중점이다. 한편 보스의 작업 계약은 핀 N을 빗살 N과 N+1 사이로 정의하고 `pin_gap()`도 `basket_gaps()`의 이웃 빗살 틈을 반환한다. 각 분면 모델의 빗살/틈 수가 Q1 7/6, Q2 9/8, Q3 13/12, Q4 13/12인 점 및 최근접 오차(반 pitch)는 이 좌표 역할 차이를 뒷받침한다.
- 합성 모델 x 대조에서 tine station 좌표가 `_gap_number_candidate()`를 통과해 잘못된 gap 후보 번호를 받을 수 있었다. 수치와 예시는 LOG 2026-10-18 01:25에 있다. 이것은 실물 정답/사진 정확도 결과는 아니다.
- **수정 전 검토 요청:** 현재 group은 tine-station 관측으로 보존하고, 연속한 두 x station의 관측을 결합해 사이의 model gap center를 보정한 뒤 `pin_gap()`으로 번호를 매기는 것이 계약상 일관된 후보 설계다. 다만 station-to-station pairing, 두 줄/네 선 보정 결합, floor gap center와 tine-tip correction의 좌표 관계를 정하고 그 계약 테스트를 추가해야 한다. 이 결정 전에는 E의 `number`, `gap_centre_base_xyz` 의미를 바꾸거나 detector/threshold를 수정하지 않는다. `gap_centre_base_xyz`는 오프라인 진단 출력이며 파이프라인 target으로 연결하지 않는다.

## 2026-10-18 01:51 KST Codex — E: station pair와 gap 후보 출력 분리
- 이전 보류 판단을 갱신한다. 보스의 주소 계약(`핀 N`은 tine N과 N+1 사이), 기존 `basket_gaps()` 배열, `pin_gap()` 선택 함수를 기준으로 같은 x의 cross-row pair는 tine station이며 gap은 이웃 station 사이에 놓인다는 점을 모델과 코드에서 재현했다. 검출 결과에서 `pairs`는 station 관측으로 유지하고 `gaps`를 인접 station 조합으로 별도 만든다. gap 번호는 두 station 중점 x에만 배정하며, gap center는 floor-level model center에 두 station correction의 평균을 적용한다.
- 양쪽 station이 모두 검출·일치해야 confirmed gap center를 낸다. 불일치/부분 관측은 candidate와 거부 상태만 기록한다. station pair 수(`confirmed_tine_pairs`)와 gap 수(`confirmed_gaps`)를 분리하고, 기존 `confirmed_pairs`는 기존 의미(번호가 부여된 confirmed gap 수)를 유지한다. `pairs.png`에는 `Tn` station과 `Gn` gap marker를 따로 표시한다. 이 출력은 진단 전용이며 실제 target 경로에 연결하지 않는다.
- station 누락을 가로질러 false gap을 만들지 않도록 인접 station x 간격이 모델 tine pitch의 0.5–1.5배일 때만 gap 후보를 만든다. 실제 profile이 이 범위를 벗어나는지 여부는 별도 배치/GT 확인 전에는 판단하지 않는다.
- `test_tine_station_pairs_form_numbered_gaps_between_adjacent_stations`는 네 분면, yaw 0.1°/180.1°, gap midpoint 번호, 보정된 floor center, station disagreement 거부, 누락 station 사이 gap 미생성을 검사한다. 전체 시험 **86/86 통과**. `py_compile`과 `git diff --check`도 통과했다.
- **보스 검토 요청:** 각 tine station에서 추정한 basket +x/+y correction을 이웃 station 사이 floor gap center로 옮기는 규칙이 진단용 후보로 적절한지 검토해 달라. 전수 visible-pin GT가 없어 이 구현은 정답률, 실측 위치 정확도, 80% gate 또는 실기 타깃 안전성을 입증하지 않는다. 기존 사진·라벨·모델·설정은 수정하지 않았고 CPU 오프라인 단위시험만 수행했다.


## 2026-10-18 02:06 KST Codex — E: keep tip-to-floor correction explicitly unvalidated
- A read-only aggregation of the 66 complete Q2/Q3/Q4 checked labels formed 8/11/10 available adjacent-tine gap samples and compared their tip-plane and root-plane residuals. Median `tip − root` Δy was −6.33/−2.66/+3.21 mm (Q2/Q3/Q4); mean absolute gapwise Δy difference was 6.50/2.70/3.19 mm. See LOG 02:06 and the source JSON path there.
- These are conditional ray/plane intersections under assumed root/tip heights and saved transforms, not independent 3D measurements. The quadrant-dependent differences do not prove that the basket itself translates differently with height, but they do show the available audit cannot validate carrying a fitted top-tip correction straight to a floor-level gap center.
- Keep the new `gaps`/center output diagnostic-only. A candidate center may be shown as a hypothesis but must not be described or consumed as a corrected floor target. Before assigning it physical meaning, either obtain independent basket/root pose evidence or validate an explicit camera/height/geometry transform on separately reviewed labels across quadrants. Human review requested: retain this candidate-only convention, or pause floor-center output pending such a transform/measurement?


## 2026-10-18 02:15 KST Codex — E: plane-height sensitivity limits the transfer conclusion
- Reconstructed all 66 source coordinates from saved metadata to numerical precision (max discrepancy 2.8e−14 mm), then perturbed the assumed tip-plane height by +1 mm. Basket-local y sensitivity was about 0.4–0.6 mm/mm, with sign by camera/quadrant. Local linear shifts of roughly 5–12 mm median per label could remove the prior tip-vs-root y residual; these shifts are not height measurements and do not imply the actual tine height is wrong. See LOG 02:15.
- Thus the Q2/Q3/Q4 residual differences identify an unresolved height/transform/geometry issue but cannot alone reject a rigid translation model. Keep candidate floor centers non-actionable; neither adopt them as corrected targets nor suppress the diagnostic evidence before the boss decides. A physical transfer claim needs independently measured tine heights or a validated camera/height transform plus root/floor pose evidence.


## 2026-10-18 02:24 KST Codex — E audit erratum: require both rows for a gap-center comparison
- The 02:06 aggregation had asymmetric boundary handling. Correct two-row support is Q2/Q3/Q4 **7/8, 11/12, 10/12** (28/32 total); Q2 gap 1 and Q3 gap 12/Q4 gaps 11–12 are one-row-only and excluded. See LOG 02:24.
- Recomputing each two-row center from four checked tine endpoints gives median tip−root Δy **−6.27/−2.66/+3.21 mm**, with mean absolute Δy **6.29/2.70/3.19 mm** in Q2/Q3/Q4. The sample-handling correction does not change the caution: this remains conditional on plane and saved-transform assumptions, not a measured target error. The 02:15 per-label height sensitivity is unaffected.

## 2026-10-19 보스 — 운반 경로: 오늘은 규칙, 점진적으로 자유 경로(AI)
- 결정: 넣기 운반 구간은 오늘 실증에서 규칙 기반(수직 상승 ≥15 cm → 관절 직선 + J2/J3 높이 보정 → 수직 하강, 196ca11·e01f411)으로 간다. 이후 단계적으로 rule-based → free form(AI 경로 생성 등)으로 바꾼다.
- 이유: 실증이 우선이고, 규칙 경로는 높이 하한을 항상 지키며 sim 검증 경로와 실기 경로가 같다. RRT는 충돌 모델만 피해 낮게 지나가 마커판과 부딪혔다(10/16). 숫자(15 cm, 마커·꽂힌 접시 +5 cm)는 보스 지시값이며 튜닝·학습값이 아니다. 학습된 것은 바구니 위 넣기 보정 PPO(gx30_seen)뿐.

### 2026-10-27 — 슬롯 CNN 실사진 hard negative 후보는 배포하지 않음
- 변경한 학습 근거: 라벨 없는 19장 중 Q4→Q3 거부 3장에서 사람이 확인한 끝단 false tip 4점만 negative로 추가했다. 실사진의 나머지 root/tip은 기존 ep02의 pseudo-label이며 독립 정답이 아니다.
- 비교한 손실: (1) 기존 CenterNet focal+offset 손실의 sim 3,000장/실사진 반복 혼합, (2) 마지막 예측층만 갱신하고 false-tip 주변 밖 logits를 고정하는 distillation.
- 채택 조건은 기존 v4 기준(tip recall≥.97, precision≥.93, Q1≥13/14, 번호 오답0)과 교차 strict 7/10 이상을 동시에 보존하는 것. 두 방법 모두 교차 strict가 6/10으로 감소해 미채택하고 `run_v10_cross/ep02.pt`를 유지한다. 같은 3장을 학습과 평가에 사용했으므로 끝단 confidence 감소를 일반화 성능으로 주장하지 않는다.

### 2026-10-27 — Tesollo 바구니 넣기는 그리퍼 계획기를 그대로 쓴다 (보스)
- 운반: 그리퍼 파이프라인 기본(direct: 직선 상승-이동-하강, RRT 없음), 파지→바구니 위를 한 스플라인. 삽입 각도 튜닝은 보스가 나중에 직접.
- 속도: 실기 로봇팔 구간 기본 80°/s, 보스가 줄이라고 하기 전에는 내리지 않는다(`--vel`로 지정).
- plate 삽입 시 손(손목) 회전은 30–45° 이내(보스: 이미 거의 슬롯 방향으로 쥠). 지금 계획은 이보다 커서 다음 설계 대상.
- 슬롯 CNN: 실사진 몇 장으로 추가 미세학습하지 않는다. 독립 라벨·원인 분석 뒤에 결정(CK).

### 2026-10-29 — direct 운반은 파지 후 sparse one-go spline으로 실행 (보스)
- 파지 완료가 정지 경계다. 그 이후 lift→carry→insert는 하나의 spline leg로 실행하며 총 waypoint는 20개 이하로 제한한다.
- direct 운반에서는 1 cm 단위 수직 waypoint, 파지 후 FK 환경 충돌 전수검사, RRT 및 60점 균등 재표본화를 사용하지 않는다. 현재 시연 배치는 운반 공간에 충돌물이 없다는 보스 판단을 전제로 한다.
- 환경 충돌검사를 제거해도 관절 한계, spline 관절 한계 여유, waypoint 간 최대 관절 변화, 계획 유효시간 검사는 유지한다. 접근·파지 경로의 기존 충돌검사는 변경하지 않는다.
- release 뒤 복귀는 별도 spline이다. release라는 작업 경계를 지나므로 파지→삽입 one-go와 합치지 않으며, 40° 이하 knot로 분할한다.

### 2026-10-29 — Tesollo 시연 waypoint와 속도 확정 (보스, 최신 지시)
- 기존 3/10/49/13/13/19를 접근 3, 7 cm 간격 상승 3, 수평 운반 20 이하, 하강·상승·base 복귀 cycle 10 이하로 바꾼다. 네 단계의 표시 waypoint 합은 40 이하로 제한한다.
- 파지 이후 환경 충돌검사는 수행하지 않는다. 슬롯 삽입에서만 수직하강하며, 같은 수직선으로 상승한 뒤 base/home으로 이동한다.
- 파지 후 lift+carry는 한 spline이다. 슬롯 하강 뒤 손을 연 다음 상승+home은 한 `return_home` spline이다.
- Tesollo arm 기본 속도는 기존 80°/s에서 **60°/s**로 변경한다. 이 지시가 10/27의 80°/s 결정보다 최신이다.

### 2026-10-29 — 파지 후 충돌검사 제거의 예외: 카메라로 다시 본 바구니 내부 접시
- 보스 후속 지시로, 첫 접시를 넣은 뒤 다음 실행의 고정 RGB-D에서 다시 검출된 바구니 내부 접시는 장애물로 취급한다. YOLOE plate mask와 depth를 base-frame OBB로 만들며, 바구니 pose 내부에 충분한 점이 있는 검출만 채택한다.
- 기존의 전체 환경/FK 충돌 sweep은 되살리지 않는다. `taught_plan`의 실제 controller spline을 조밀 표본화해 **들고 있는 접시와 live OBB 사이 여유만** 계산하며, 15 mm 미만이면 이동 전에 거부한다. 따라서 sparse waypoint와 one-spline 실행 구조는 유지된다.
- 오늘 넣었다고 기억한 슬롯 번호를 곧바로 장애물 형상으로 만들지 않는다. 이전 오프라인 시험에서 그 형상은 유효한 Q3:6→Q3:12 경로도 −10 mm로 거부했으며, 보스 지시는 재관측 기반이다. 실제 시연 전에는 바구니 속 edge-on 접시 검출과 OBB 크기를 화면에서 확인해야 한다.
