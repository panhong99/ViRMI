# 식기 정리 로봇: 접시·그릇 → 식기 바구니 (D455 + YOLOE + Doosan M1013)

테이블 위 접시·그릇을 고정 카메라로 찾아 잡고, 식기 바구니의 빈 칸(접시는 핀 사이 슬롯)에 넣습니다.
엔드 이펙터가 둘이라 시연도 둘입니다.

| 시연 | 손 | 문서 |
|---|---|---|
| **A. 핸드 시연 (현재, 11/3 시연)** | Tesollo DG-5F-S 5지 핸드 | 아래 [A](#a-핸드-시연-tesollo-dg-5f-s) |
| B. 그리퍼 파이프라인 (10/12) | Robotiq 2F-85 | 아래 [B](#b-그리퍼-파이프라인-robotiq-2f-85) |

## A. 핸드 시연 (Tesollo DG-5F-S)

```text
고정 D455 ─ 관객 화면(demo_watch: target / obstacle) ─ 공유 프레임 /dev/shm/fixed_latest.npz
   ├ 바구니 위치 측정(basket_vision)  ├ 바구니 칸 상태(empty_quadrant: 그릇·꽂힌 접시)  └ 물체 종류·위치(perceive)
팔 접근: 홈 → 물체 위 → 수직 하강 (스플라인 1개, arm_plan)
손: 준비 자세 → 마지막 파지 자세 쪽으로 천천히 닫기 → 힘 파지 (한 동작)
파지 후: 수직 30 cm 상승 → 바구니 빈 칸 위 → (접시) 슬롯 1 cm 위 → 40°/s로 삽입   (경로는 손이 닫히는 동안 미리 계산)
놓기 → 손 모양 그대로 홈 복귀 → 손 펴기
```

빈 칸 선택: 그릇은 아무 빈 칸(Q1, Q2, Q4, Q3 순), 접시는 1번째 앞쪽·2번째 뒤쪽(같은 칸). 오늘 넣은 것은
`/tmp/tesollo_demo_state.json`에 기억하고, 카메라로 본 바구니 안 접시·그릇도 장애물로 칩니다.
접시 삽입은 보스가 펜던트로 티칭한 자세(`tesollo/taught_inserts.json`: 바구니 위 / 슬롯 1 cm 위 / 슬롯 안)를
측정한 바구니 위치·핀에 맞춰 옮겨 씁니다(손목 25° 덜 젖힘, 접시 바닥은 티칭 위치 유지).

### A-1. 실행

```bash
# 터미널 1: 로봇 연결 (계속 켜 둠) — 펜던트에서 PC로 제어권 이전 후
bash ~/pan/6D_pose/calibration/run.sh connect

# 터미널 2: 시연 (관객 화면이 같이 뜸; 물체를 보면 끝까지 자동)
~/miniconda3/bin/python3 ~/pan/6D_pose/tesollo/dg5fs_sdk.py --object auto --vel 60
```

핸드: 왼손 `169.254.186.73`, DIP ② Developer 모드, DGManager와 ROS 핸드 드라이버는 꺼 둠(핸드는 클라이언트 1개만 받음).
`~/miniconda3/bin/python3`(conda base, cffi 있음)로 실행하고, 계획 스크립트는 `foundationpose` 환경으로 자동 호출됩니다.

| 옵션 | 뜻 (기본값) |
|---|---|
| `--object auto\|plate\|bowl` | 카메라로 고르기 / 고정 |
| `--vel 60` | 팔 속도 °/s (= °/s²), `--insert_vel 40` 슬롯 삽입 구간 |
| `--lift 0.30` | 파지 후 수직 상승 [m] |
| `--grasp_ms 3000`, `--preclose` | 손 닫는 시간, 미리 닫는 비율(접시 0.7, 그릇 0.95) |
| `--taught_wrist_less 25` | 티칭 삽입 자세보다 손목을 덜 젖히는 각도 |
| `--new_demo` | 오늘 넣은 것 기억 지우기 (바구니를 비웠을 때) |
| `--hold` / `--step` | 파지 후 멈춤 / 단계마다 Enter |
| `--teach` | 바구니 위에서 멈추고 펜던트로 삽입 티칭, Enter마다 관절 기록 |
| `--approach_stop`, `--preclose 0`, `--taught_wrist_less 0`, `--open_first` | 이전 동작으로 되돌리기 (디버깅용) |

### A-2. 따로 돌려 보기 (로봇 안 움직임, `~/pan/6D_pose`에서)

```bash
PY=~/miniconda3/envs/foundationpose/bin/python
$PY tesollo/empty_quadrant.py --basket /tmp/tesollo_basket_<시각>.json   # 바구니 칸 상태
$PY tesollo/perceive.py                                                  # 테이블 물체
$PY tesollo/arm_plan.py plate                                            # 팔 접근 경로
$PY tesollo/taught_plan.py --quadrant Q3 --pin 6 --basket <바구니 json>   # 접시 삽입 경로
$PY tesollo/basket_plan.py --object bowl --quadrant Q1 --rise 0.30 --basket <바구니 json>
bash calibration/run.sh mover --plan <plan.json> --leg post_grasp --smooth --max_step_deg 90   # --execute 없으면 검사만
```

`calibration/` 안에서 YOLOE를 돌리면 `mobileclip_blt.ts`(572 MB)를 다시 받습니다. 저장소 최상위에서 실행하세요.

### A-3. 파일

| 파일 | 역할 |
|---|---|
| `tesollo/dg5fs_sdk.py` | 시연 진입점: 인식 → 팔 → 손 → 바구니 → 복귀 |
| `tesollo/arm_plan.py`, `arm_poses.yaml` | 물체 위치에 맞춘 팔 접근/상승 경로 (티칭 파지 자세 기준) |
| `tesollo/taught_plan.py`, `taught_inserts.json` | 접시: 티칭 삽입 경로를 바구니·핀에 맞춤 |
| `tesollo/basket_plan.py` | 그릇: 바구니 칸 가운데로 / 접시: 계획기 경로(`--planner`) |
| `tesollo/perceive.py`, `empty_quadrant.py`, `fixed_frames.py` | 물체 인식, 바구니 칸 상태, 공유 프레임 읽기 |
| `tesollo/poses.yaml`, `grasp_contact.json` | 손 자세, 마지막 파지 손가락 관절 |
| `calibration/demo_watch.py` | 관객 화면 (YOLOE, 깊이로 꽂힌 접시 장애물) |
| `calibration/basket_vision.py` | 바구니 위치 측정 |
| `calibration/ros2_nodes.py` (`run.sh mover`) | 로봇에 경로 실행 (검사 → 스플라인 1개) |

실행 기록: `/tmp/tesollo_basket_<시각>.json`(바구니), `/tmp/tesollo_demo_view.log`(관객 화면),
`calibration/data/rl_plans/<시각>_hand_taught/plan.json`(접시 경로), `..._taught/`·`..._merged/`(그릇 경로).

## B. 그리퍼 파이프라인 (Robotiq 2F-85)

```text
고정 D455 → YOLOE + FoundationPose → 접시 위치 → 잡기 경로 라이브러리(IK 접근 + BC)
          → 마커 4개 → 바구니 위치 → 베스트 티칭 경로를 바구니·잡은 자세에 맞춰 IK 보정
          → 운반 → 넣기(기울여 천천히) → 놓기 → 홈        (손목 D455: 바구니 위·넣기 직전 사진)
```

2026-10-12 기준: 접시 위치 무관 잡기, 바구니 위치를 바꿔도 넣기 성공, `--auto`로 사람 입력 없이 연속 동작.
속도는 항상 느리게: 운반 6°/s, 바구니 위→넣기 직전 1.5°/s, 핀 안 0.25°/s.

### B-1. 환경 (처음 한 번, A도 같음)

| 구성 | 값 |
|---|---|
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy, Doosan `~/pan/doosan_ws` |
| conda | `foundationpose` (인식, 계획, 카메라 도구) |
| 로봇 | M1013, 컨트롤러 `192.168.127.100`, PC `192.168.127.5/24` (`enp5s0`) |
| 카메라 | 고정 D455 `338122300585`, 손목 D455 `338122303684` |
| 그리퍼 | Robotiq 2F-85, PC USB-RS485 직접 제어(두산과 통신 안 함), 힘 64/255 |

```bash
conda env create -f environment.yml && conda activate foundationpose && bash build_all.sh
bash ~/pan/doosan_ws/setup_ros2_jazzy.sh                  # ROS 2 + Doosan (conda 밖)
sudo ip addr add 192.168.127.5/24 dev enp5s0              # 재부팅 후 주소가 없을 때만
```

### B-2. 실행 (`~/pan/6D_pose`에서)

```bash
# 터미널 1: 로봇 연결 (계속 켜 둠, 두산 RViz는 안 뜸)
bash calibration/run.sh connect

# 터미널 2 (선택): 경로 미리보기 RViz — 잡기부터 바구니까지 전체 궤적, 바구니 모델
bash calibration/run.sh viz

# 터미널 3: 접시 → 바구니 (펜던트에서 PC로 제어권 이전 후)
bash calibration/run.sh pipeline --task plate --plate_policy ik_bc --execute          # 정지점에서 확인
bash calibration/run.sh pipeline --task plate --plate_policy ik_bc --auto --execute   # 연속 자동
```

| 옵션 | 뜻 |
|---|---|
| `--execute` | 실제 이동. 빼면 계획·검사만 |
| `--auto` | 시작 문구 1번 뒤 사람 입력 없음. 그리퍼 자동(잡기 직전 FP로 접시 이동 확인, 물체 감지 확인) |
| `--place basket` | (기본) 마커로 바구니 위치 측정 후 넣기 |
| `--place golden` / `lower` | 마커 없이 고정 위치(`basket_shift_m`) / 제자리에 내려놓기 |
| `--stop_at_above` | 바구니 위에서 멈추고 끝 |

`--auto` 없이 실행하면: 바구니 위 `d`/`h`, 넣기 직전 `d`/`u`, 그리퍼 `c`/`o`/`s`를 묻습니다.
세부 값(깊이, 기울기, 높이, 속도, 보정)은 모두 `calibration/config.yaml`의 `plate_ik_bc`에 있습니다.

**바구니를 옮길 때**: 몸체를 잡고 옮기고 마커 4개가 고정 카메라에 보이게 둡니다. 멀리(+x)는 거의 못 가고 로봇 쪽으로 옮기는 게 안전합니다(넣기 자세는 그리퍼를 세워야 해서 팔이 덜 닿음). 닿지 않으면 움직이기 전에 거부합니다.
**마커를 다시 붙였을 때**: 바구니를 알려진 자리에 두고 `conda run -n foundationpose python calibration/camera_tools.py basket --define_current`.

### B-3. 도구

| 명령 | 용도 |
|---|---|
| `bash calibration/run.sh wrist_look` | 지금 자세의 손목 사진 + 모델 핀(초록), 로봇 안 움직임 |
| `bash calibration/run.sh mover --plan P --leg L [--execute]` | 계획의 한 구간만 실행(수동 이어 가기) |
| `python calibration/camera_tools.py basket` | 마커로 바구니 위치·정합 오차 확인 |
| `/usr/bin/python3 calibration/gripper.py open\|close\|goto --pos N\|status` | 그리퍼 직접 조작 |
| `bash calibration/run.sh tests` | 하드웨어 없는 회귀 테스트 |

결과: `calibration/data/real_pipeline/<시각>_plate_*/` — `robot_state.jsonl`(관절·FP·외력·단계 이벤트), `wrist_above.png`/`wrist_pre.png`, `preview_plan.json`, 비정상 종료 시 `faulthandler.log`.

### B-4. 자주 막히는 경우

| 증상 | 확인 |
|---|---|
| `basket pose unreliable … residual` | 마커가 움직였거나 가려짐(8 mm 초과). 접시·팔이 가리는지, 마커 고정 확인 |
| `unreachable … 로봇 쪽(-x)으로 옮기세요` | 바구니가 팔이 닿는 범위 밖. 로봇 쪽으로 당기기 |
| `Access control deny` | 제어권이 펜던트에 있음 → PC로 이전 |
| D455 `Device or resource busy` | 이전 실행의 인식 창이 남아 있음 → 닫기 |
| 펜던트 조그가 계속 눌린 것처럼 움직임 | 조그 중엔 `run.sh viz`를 끄고, `connect`를 다시 시작하지 않기 |
| `No module named 'rclpy'` | ROS 명령은 conda 밖(시스템 Python)에서 |

구현 파일은 [calibration/README.md](calibration/README.md), 진행 기록은 `collab/LOG.md`, 결정은 `collab/DECISIONS.md`.
