# calibration/ — 실로봇 쪽 (M1013 + 2F-85 + D455 2대)

평소 실행 순서는 루트 [README.md](../README.md) 2·3장을 봅니다. 여기는 파일 지도와 실행 환경입니다.
모든 수치(시리얼, 마커, 포트, 캡처 기준, 계획 파라미터)는 `config.yaml`에 있습니다. 실로봇 속도는 `config.yaml` `plate_ik_bc`의 `vel_deg_s`(15 °/s, 통일)와 바구니 하강·넣기 값이 정합니다. mover의 속도 상한은 10/13 보스 결정으로 없앴습니다(컨트롤러 안전 설정은 그대로).

## 파일

| 파일 | 실행 환경 | 내용 |
|---|---|---|
| `config.yaml` | – | 경로, 로봇(홈 자세·네임스페이스·IP), 카메라 시리얼, UDP 포트, 마커, 캡처·계획 기준 |
| `calib_core.py` | 어디서나 | 변환·posx, ArUco, hand-eye 풀이(eye-to-hand / eye-in-hand / 고정 카메라 평균·PnP), 최신 결과 선택, `selftest()` |
| `robot_model.py` | 어디서나 | M1013 FK/IK, 작업 셀 박스·거리장 충돌 검사·RRT, 경로 보간·시간 배분. `reach`(IK 도달 지도), `points`(충돌 샘플 캐시) |
| `sensors.py` | conda | D455 컬러 스트림, UDP 수신(로봇 pose 5006, FP pose 5005), 마커 샘플 캡처, `RobotRecorder`(실행 기록) |
| `calibrate.py` | conda | `wrist`(T_flange_cam), `fixed`(T_base_cam), `gt`(FP 오차 측정), `selftest` |
| `camera_tools.py` | conda | `preview`, `marker`, `probe`, `basket`(마커로 바구니 위치, `--define_current`), `wrist_look`·`pre_photo`(손목 사진 + 모델 핀) |
| `pipeline.py` | conda | 접시(`--task plate`): FP → 잡기 계획 → 바구니 계획 → 구간별 실행(`--auto`). 큐브 작업과 티칭(`--task teach/replay`)도 |
| `plate_plans.py` | conda | 잡기 라이브러리 선택, 티칭 경로, `golden_slot_plan`(바구니 이동·손 안 접시·기울임·높이 보정 + IK), Isaac export |
| `ros2_nodes.py` | 시스템 Python + ROS | `flange`(읽기 전용 pose·외력 스트림), `publisher`, `mover`(구간 실행), `viz`(RViz 미리보기 + 바구니) |
| `gripper.py` | 시스템 Python | Robotiq 2F-85 (USB-RS485), 가장 느린 속도 |
| `sim_scene.py` | Isaac Sim | 실제 셀 디지털 트윈(캘리브레이션된 카메라 시점 렌더, 궤적 재생) |
| `tests.py` | conda | 하드웨어 없는 회귀 시험 21개 |
| `run.sh` | bash | `connect`, `stream`, `viz`, `pipeline`, `mover`, `wrist_look`, `sim`, `tests` (환경 설정 포함) |
| `data/` | – | 캘리브레이션 결과·샘플·계획(덮어쓰지 않음) |

## 실행 환경이 중요한 이유

- ROS 2(`rclpy`)는 시스템 Python 3.12에서만 돕니다. conda Python에서 import하면 깨집니다. `run.sh connect|stream`이 conda 변수를 지우고 ROS를 source합니다.
- `pipeline.py`(conda)는 `ros2_nodes.py`를 import해서 mover와 같은 검증 함수로 계획을 미리 검사합니다. 그래서 `ros2_nodes.py`는 `rclpy`를 함수 안에서만 import합니다.
- `sim_scene.py`는 Isaac Sim의 python(`isaacsim-5.1.0/python.sh`), `rl/`은 Isaac Lab의 python을 씁니다. 둘 다 `robot_model.py`의 셀 배치를 공유합니다.

## 좌표와 결과 파일

- 변환은 4×4, 미터, `T_a_b` = a에서 본 b(b 좌표를 a 좌표로 바꾸는 행렬). Doosan posx는 mm·도·ZYZ.
- 고정 카메라: `data/real_handeye/<최신>/T_base_cam_pnp.json`을 `calib_core.latest_t_base_cam()`이 고릅니다.
- 손목 카메라: `data/wrist_handeye/<최신>/result.json` (현재 `20261006_111318_clean`).
- 다시 계산(`--solve`, `--resolve`)은 기존 결과를 덮어쓰지 않습니다(`--out` 필요, `--force`로만 교체).

## 안전 경계

- 기본 실행은 읽기·계획·관찰용 발행까지만 합니다. 실제 이동은 `pipeline.py --execute`에서 mover가 웨이포인트마다 사람의 `go`를 받아서만 합니다.
- mover 거부 조건: 속도 > 2 °/s, 승인 안 된 계획, 오프라인 계획, 30분 넘은 계획, 한 동작 45° 초과, 관절 한계 2° 이내, 현재 관절이 계획 시작점과 3° 이상 차이(`go` 직후 다시 확인), 도달 오차 1° 초과. 컨트롤러가 거부하면 재시도 없이 멈춥니다.
- 관절값이 1초 이내로 수신되지 않으면 계획하지 않습니다(0으로 대체하지 않음).
- 재부팅 후 PC에 컨트롤러 대역 주소가 필요합니다: `sudo ip addr add 192.168.127.5/24 dev enp5s0`.

## 접시 → 바구니 (2026-10-12)

실행 명령은 루트 [README.md](../README.md) 2장. 관련 설정은 `config.yaml`의 `plate_ik_bc`:

| 키 | 지금 값 | 뜻 |
|---|---|---|
| `golden_teach` | `data/teach/golden/full_01_best_20261008_1504.json` | 기준 넣기 티칭 |
| `basket_shift_m` | [0, −0.076, 0] | `--place golden`에서 바구니 위치(티칭 대비) |
| `basket_nudge_m` | [0.005, 0, 0] | 접시 경로 미세 보정 |
| `basket_yaw_deg` / `basket_tilt_deg` | 14.7 / 15 | 핀 줄에 맞춘 회전 / 미끄러져 들어가게 기울임(넣기 자세만) |
| `basket_tilt_x_comp_m` | 0.04 | 기울일 때 핀 줄 방향 보정 |
| `basket_transit_lift_m` / `basket_carry_lift_m` | [0.13, 0.10, 0.07] / 0.15 | 바구니 위·운반 높이(닿는 가장 높은 값) |
| `basket_pre_up_m` / `basket_insert_up_m` | 0.075 / 0.020 | 넣기 직전 정지점 / 넣는 깊이(티칭 대비 위) |
| `basket_pre_vel_deg_s` / `basket_insert_vel_deg_s` | 1.5 / 0.25 | 첫 하강 / 핀 안 속도 |
| `basket_retreat_up_m` | 0.08 | 기울였을 때 놓은 뒤 수직 상승(닿는 만큼) |
| `offset_m` (`plate_rl`) | [0.0025, 0.055] | 잡기 위치 보정(반경, 접선) |

바구니 모델(빗살 실측: 레일 4.9 cm, 끝 높이 약 11.5 cm, −x로 12.5°)은 `rl/basket.yaml`, 마커 배치는 `basket.layout`.
