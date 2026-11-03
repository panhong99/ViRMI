# 프로젝트 단계 (보스 11/03)

포트폴리오 정리용으로 단계를 나눈다. 단계마다 기간, 목표, 한 일(작업 묶음별), 대표 수치, 코드·결과 위치를 남긴다.
새 단계가 시작되면 아래에 섹션을 추가하고, 끝나면 git 태그 `phaseN-YYYYMMDD`를 붙인다.

| 단계 | 기간 | 한 줄 요약 | 태그 |
|---|---|---|---|
| Phase 1 | 2026-09-09 ~ 2026-11-03 | 접시·그릇을 인식해 5지 핸드로 잡고 식기 바구니 슬롯에 꽂는 시연 (11/03 데모) | `phase1-20261103` (데모 당일 `demo-20261103`) |
| Phase 2 | 2026-11-03 ~ | 분면 간 옮기기, 파지·접근·삽입을 학습(RL/지도학습)으로 치환, 측면 카메라 삽입 보정, 슬롯·바구니 자유도 | |
| Phase 3 (final) | Phase 2 후 | RL 손 안 조작 (in-hand manipulation) | |

---

## Phase 1: 식기 바구니 정리 시연 (09/09 ~ 11/03)

**목표**: 고정 카메라로 테이블 위 접시·그릇을 찾아 로봇 팔(Doosan M1013)과 손(Robotiq 2F-85 → Tesollo DG-5F-S 5지)으로
잡고, 식기 바구니의 빈 칸 또는 지정 슬롯에 넣는다.

### 작업 묶음

| # | 작업 | 방법 | 결과 | 코드 |
|---|---|---|---|---|
| 1 | 실시간 6D 자세 추정 | YOLOE(텍스트 프롬프트) 검출 + FoundationPose 6D, ROS 2 브리지 | 접시·그릇 실시간 6D, GUI | `real_time_project/`, `ros2_ws/` |
| 2 | 카메라-로봇 캘리브레이션 | 손목·고정 카메라 hand-eye, 마커 PnP, TCP 재측정 | 고정 카메라 → 로봇 기준 좌표 | `calibration/` |
| 3 | 그리퍼 파지 (2F-85) | IK 접근 + BC(행동 복제) 파지, 시연 데이터 수집 | 접시 위치가 바뀌어도 파지 | `IL/collect.py`, `calibration/plate_plans.py` |
| 4 | 시뮬 강화학습 파이프라인 | Isaac Lab pick-and-place 환경, 단계별 커리큘럼 PPO, 센서 오차 무작위화 | 단계 통과 학습, 실기 재생 검증 | `rl/` |
| 5 | 바구니 인식 | 마커 기반 → 마커 없이 YOLOE로 칸 상태(빈 칸, 꽂힌 접시) 판정 | 빈 칸 자동 선택, target/obstacle GUI | `calibration/basket_vision.py`, `calibration/demo_watch.py` |
| 6 | 측면 카메라 슬롯 인식 | 측벽 카메라 슬롯 CNN, 시점 교차 학습, 실기 hard negative | 핀 번호 실시간 표시 | `experiments/side_cam/`, `experiments/slot_cnn/` |
| 7 | 5지 핸드 통합 | DG-5F-S SDK 제어, 힘 파지, DGManager 자세 티칭 | 접시·그릇 연속 파지 | `tesollo/dg5fs_sdk.py` |
| 8 | 손 안 자세 측정 | 들어 올린 접시를 고정 카메라 점군으로 측정 (중심, 기울기, yaw) | 0.1 s 측정, 삽입 경로 보정 | `experiments/inhand/`, `tesollo/measure_held.py` |
| 9 | 슬롯 삽입: GT 재생 | 성공 1회 경로를 목표 슬롯까지 옮기고 보정 + J4 고정 IK, Q3 경로를 Q4로 옮김 | 빨강·흰 접시 목표 슬롯 ±1칸, Q4 핀 1–12 경로 생성 | `tesollo/gt_replay.py` |
| 10 | 슬롯 삽입: 점군 RL | 꽂힌 접시 점군 목표 PointNet PPO, 기존 제어기 가중치 1 → 0 | 시뮬 물리 재생 성공률 43.9 % → 81.2 % | `rl/dock_env.py`, `rl/dock_pc_train.py` |
| 11 | 경로 계획 비교 | cuRobo, PyRoki 벤치마크, 티칭 경로 기반 계획 | 시연용 계획기 선택 근거 | `experiments/curobo_bm/`, `experiments/pyroki_bm/` |
| 12 | 시연 시스템 | 안전 데모 스크립트, 상주 이동 프로세스, 시간 지정 스플라인, 그릇+접시 연속 시연 | 11/03 데모 | `tesollo/demo_safe.sh`, `tesollo/demo_all.sh` |
| 13 | 멀티 에이전트 협업 | Claude(구현) / Codex(검토) 역할 분담, 공유 기록 | 결정·로그 누적 | `collab/` |

### 대표 수치
- 슬롯 삽입 점군 PPO: 시뮬 물리 재생 성공률 43.9 % → 81.2 % (`outputs/dock_rl_20261030/`).
- GT 재생 삽입: 빨강·흰 접시 모두 목표 슬롯 ±1칸 (`outputs/gt_replay/`).
- 손 안 자세 측정: 0.1 s (`outputs/inhand_20261030/`).
- 시연 속도: 접시 1장 60 s 중 정지 32 s(54 %)를 찾아냄. 리프트+운반+삽입 구간을 15.1 s에서 줄이도록 시간 지정 스플라인과 상주 이동 프로세스 도입 (11/03).

### 결과물
- 시연 영상·GIF: `assets/videos/`, README.
- 포트폴리오 자료: `outputs/portfolio_nis_20261031/`, `outputs/portfolio_slot_cnn_20261026/`, `outputs/kimm_ai_award_20261027/`.
- 발표 자료: `outputs/plate_basket_architecture_*_20261019.pptx`.

### 시연 명령 (Phase 1 끝 기준)
```bash
bash /home/pan/pan/6D_pose/calibration/run.sh connect          # 터미널 1
bash /home/pan/pan/6D_pose/tesollo/demo_plates.sh              # 접시 2장
bash /home/pan/pan/6D_pose/tesollo/demo_all.sh                 # 그릇 + 접시
```

---

## Phase 2: 학습 기반 식기 정리 (11/03 ~)

받은 일 (보스 11/03 미팅):
1. 분면에서 분면으로 옮기기 (Q3 ↔ Q4).
2. 대부분을 학습으로 치환
   - 파지 자세: GraspNet 계열 데이터셋으로 "어디를 어떻게 잡을지" 지도학습. 파지 위치로 가는 건 계획/정책, 잡는 건 힘 파지.
   - 분면·파지 접근 경로, 슬롯 삽입 자세 보정 등을 RL로.
3. 측면 카메라 CNN으로 삽입 자세 보정 (yaw, x, y).
4. 슬롯 번호 자유도, 바구니 위치 자유도 높이기.

지금 바로 병렬로 할 것:
- (a) GraspNet 활용 가이드 조사: 그리퍼 전용인지, 핸드 가능 여부, 시뮬 환경.
- (b) 측면 카메라 활용 개선: 바구니·접시 점군을 미리 만들어 두고 TAX-Pose류 상대 자세 추정.
- (c) 경로 생성 RL: GT(성공 경로)를 가이드로 사용, 데이터 양 검토.

## Phase 3 (final): RL 손 안 조작

- Phase 2 완료 후 시작. 5지 핸드 in-hand manipulation을 RL로.
