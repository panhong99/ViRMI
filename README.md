# Dish Rack Robot: 접시·그릇 → 식기 바구니

고정 카메라(D455)로 테이블 위 접시·그릇을 찾고, 두산 M1013 + Tesollo DG-5F-S 5지 핸드로 잡아
식기 바구니의 빈 칸에 넣습니다. 접시는 원하는 번호의 슬롯(핀 사이)에 꽂습니다.

| 로봇 (6배속) | 인식 화면: target / obstacle (6배속) |
|---|---|
| ![demo robot](assets/videos/demo_robot.gif) | ![demo gui](assets/videos/demo_gui.gif) |

## 파이프라인

```text
고정 D455 ─ YOLOE 인식 ─┬ 테이블 물체: 접시 / 그릇, 위치
                        └ 바구니 칸 상태: 빈 칸, 이미 들어간 접시·그릇(장애물)
   ↓
팔 접근 (물체 위 → 수직 하강, 스플라인 1개) → 5지 핸드 힘 파지 → 30 cm 상승
   ↓
손 안 접시 자세 측정 (고정 카메라, 점군)
   ↓
삽입 경로 생성 ─┬ GT 재생: 실기 성공 1회 경로를 목표 슬롯까지 옮기고 보정 + J4 고정 IK   (시연 기본)
               └ 학습 정책: 점군 PointNet PPO를 시뮬 rollout → 같은 IK                 (선택)
   ↓
슬롯 삽입 → 놓기 → 홈 복귀
```

## Work

| 항목 | 방법 | 결과 |
|---|---|---|
| 물체·바구니 인식 | YOLOE(텍스트 프롬프트) + 깊이, 바구니 칸 상태와 꽂힌 접시 판정 | 실시간 GUI에 target / obstacle 표시 |
| 파지 | 물체 위치에 맞춘 팔 접근 경로 + 티칭 파지 자세, 손을 천천히 닫는 힘 파지 | 접시·그릇 연속 파지 |
| 손 안 자세 측정 | 들어 올린 접시를 고정 카메라 점군으로 측정 (중심, 기울기, yaw) | 삽입 경로 보정에 사용 |
| 슬롯 삽입: GT 재생 | 성공 1회를 정답 경로로 삼아 슬롯 간격(20 mm)만큼 이동, 접시별 작은 보정(yaw, x) | 빨강·흰 접시 모두 목표 슬롯 ±1칸 |
| 슬롯 삽입: 학습 정책 | Isaac Lab에서 꽂힌 접시 점군을 목표로 PointNet PPO, 기존 제어기 가중치 1 → 0, 허용 오차 커리큘럼, 센서 오차 무작위화 | 시뮬 물리 재생 성공률 43.9 % → 81.2 %, 실기 목표 슬롯 정확히 삽입 |
| 그리퍼 버전 | Robotiq 2F-85, FoundationPose 6D 추정, IK 접근 + BC 파지, 마커로 바구니 측정 | 접시 위치·바구니 위치가 바뀌어도 삽입 |

## 하드웨어

| 구성 | 모델 |
|---|---|
| 로봇 팔 | Doosan M1013 |
| 손 | Tesollo DG-5F-S (왼손) / Robotiq 2F-85 |
| 카메라 | Intel RealSense D455 (고정), D455 (손목, 그리퍼 버전) |
| 소프트웨어 | Ubuntu 24.04, ROS 2 Jazzy, Isaac Lab, PyTorch |

## 설치

```bash
conda env create -f environment.yml && conda activate foundationpose
bash build_all.sh
```

## 실행

```bash
# 터미널 1: 로봇 연결 (펜던트에서 PC로 제어권 이전 후, 계속 켜 둠)
bash calibration/run.sh connect

# 터미널 2: 접시 2개 연속 (빨강 → 4번, 흰 → 10번 슬롯). 접시를 놓고 Enter, q = 멈춤
bash tesollo/demo_plates.sh
bash tesollo/demo_plates.sh --pins 3 9    # 다른 슬롯
bash tesollo/demo_plates.sh --dry         # 실행 없이 명령만 확인
```

로봇 없이 확인: `python calibration/tests.py` (하드웨어 없는 회귀 테스트).

## 구조

| 폴더 | 내용 | 주요 파일 |
|---|---|---|
| `tesollo/` | 핸드 시연 | `dg5fs_sdk.py`(시연 진입점), `gt_replay.py`, `policy_insert.py`, `measure_held.py`, `demo_plates.sh` |
| `calibration/` | 카메라·로봇·바구니 공용 코드, 그리퍼 파이프라인 | `demo_watch.py`(인식 GUI), `basket_vision.py`, `plate_plans.py`, `ros2_nodes.py`, `tests.py` |
| `rl/` | Isaac Lab 시뮬, 학습 | `dock_env.py`, `dock_pc_train.py`(점군 PPO), `hand_basket.py`, `basket/` |
| `IL/` | 그리퍼 BC 데이터 수집 | `collect.py` |
| `experiments/` | 탐색 실험 | `side_cam/`, `slot_cnn/`, `inhand/`, `pyroki_bm/`, `curobo_bm/` |
| `real_time_project/`, `ros2_ws/` | YOLOE + FoundationPose 실시간 6D 추정, ROS 2 브리지 | `main.py` |
| `estimater.py`, `learning/`, `mycpp/` | FoundationPose 원본 코드 | |
| `collab/` | 작업 기록 | `LOG.md`, `DECISIONS.md` |

데이터(`calibration/data/`, `IL/data_*`, `rl/logs/`, `outputs/`)와 모델 가중치는 저장소에 포함하지 않습니다.
