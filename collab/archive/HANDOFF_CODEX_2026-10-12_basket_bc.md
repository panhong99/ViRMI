# Codex 인수인계: 목표 2 바구니 넣기 BC 학습 — 2026-10-12 Claude 작성

Human과 Claude는 오늘 실로봇으로 목표 1(넓은 범위 파지)을 시연한다. 그동안 Codex는 **시뮬레이션만으로** 넣기 BC를 맡는다.
- **실로봇·그리퍼 명령 금지.**
- 실기 config(`calibration/config.yaml`)는 건드리지 않는다.
- GPU는 실기 FoundationPose(약 2 GB)와 같이 쓴다. Isaac은 한 번에 128 env 이하로 돌리고, 큰 작업은 LOG에 시작과 끝을 남긴다.
- 프로세스는 PID로만 종료한다(`pkill -f` 금지).
- 시작할 때 이 문서 맨 아래 상태표를 `진행(Codex)`로 바꾸고, 끝나면 결과(수치·경로)를 LOG에 남기고 "finish codex now your turn"을 적는다.

## 0. 지금까지 (Claude, 10/8~10/11)

**바구니 sim 모델** (`rl/basket_env.py`, `rl/basket.yaml`)
- 전체 파라메트릭 바구니: 53 × 52 × 15 cm, 바닥 철망, 벽 막대, 레일, 손잡이, 핀 88개.
- 핀: **수직**, sim 반지름 1 mm(실측 3~4 mm, 휨 효과 대용), 피치 2 cm, 쌍 간격 9.5 cm.
- 바구니 좌표계 = 실기 마커 좌표계(`calibration/camera_tools.py basket`).
- reset마다 ±3 cm/±10° 이동하고, `env._slot`(슬롯 1의 x, y, z, yaw)이 따라간다.
- 슬롯 축 n = 바구니 x(핀 줄 방향). 접시 목표 기울기 12.8°(티칭).

**작업 스크립트** (`rl/basket_tasks.py`)
- `grasp`: Codex 라이브러리 경로를 관절 목표로 재생해 잡기·들기. 약 31/32.
- `expert --remeasure`: 전문가 넣기.
  - 운반 순서(사람 티칭과 같음): 눕힌 채 z 0.40 m로 벽을 넘고, 슬롯 위에서 내려가며 세운다. 수직 그리퍼는 z ≤ 0.38 m만 도달.
  - above에서 손 안 접시를 재측정해 하강선을 다시 계획한다.
  - **착지 오차(최저점 d_bottom)를 재고 4 cm 위로 뺐다가 옮겨 재삽입(최대 3회)**.
  - 결과: 26/31, 21/28, 44/60(seed 8100) → **약 73~84%**.
- 성공 판정(`basket.yaml success`): 최저점이 슬롯선 ±8.5 mm 안, 최저점 높이 ≤ 55 mm(핀 사이에 끼어 섬), u 방향 ±30 mm, 법선 오차 < 20°, 열고 후퇴한 뒤 1 s 정지.
- 진단 도구: `--snapshot` + scratchpad `section.py`(단면 그림).

**데모** (`--record`)
- `IL/data_basket_expert_20261009/batch_00..15.npz`, 32 env × 16, 성공 333.
- 들어 있는 것: 스텝별 q, q_cmd, grip_close, tcp(7), plate(7), slot(4), stage(0 운반, 1 삽입·재시도, 2 놓기, 3 후퇴), success.
- 이전 `IL/data_basket_expert_20261008`(stage 없음)도 있다.

**BC 시도와 실패** (`basket_tasks.py bc_train / bc_eval`, `expert --bc_takeover`)
- 1차: 전 구간(운반 포함, 약 2000 step)을 관절 Δ 행동으로 학습 → 0/32. 슬롯에서 멀어지다 접시를 떨어뜨림.
- 2차: stage ≥ 1(슬롯 위부터)만 학습하고 운반은 계획 → **0/7, 0/1, 0/11**(놓기까지 간 env 수).
  - trace: 처음 5 s는 전문가처럼 내려가다가(low 98 → 50 mm) 핀 근처에서 옆으로 벗어남(d_bottom 9 → 60 mm). 대부분 그리퍼를 열지 않음.
  - 로그 `rl/logs/basket_round2.log`, `basket_bc2_eval_s*.log`.

## 1. 원인 판단
1. 전문가가 상태 피드백 정책이 아니다. 미리 계획한 하강선에 "직전 착지 결과로 재삽입"이 붙은 구조라, 같은 상태에서 정답 행동이 계획 진행도와 재시도 횟수(숨은 변수)에 따라 다르다. 상태만 보는 MLP-BC로는 모호하다.
2. 행동이 **관절 Δ**다. 작은 오차가 IK·자세를 통해 커지고, 바구니 pose 변화에 대한 일반화도 어렵다.
3. 복구 데이터(DART)가 없어 한 번 벗어나면 돌아오지 못한다.

## 2. 해야 할 일 (제안 순서)

**C1. 피드백 전문가로 다시 만들기 (삽입 구간 = stage 1~3)**
- 운반은 지금 계획 그대로 쓴다(above까지).
- 삽입은 `env.step` + SafeDiffIK(TCP 속도 5 cm/s, 회전 10°/s 상한 유지)로, **매 스텝 접시 최저점 오차(d_bottom, d_u)와 높이(low)만 보고** TCP 이동을 정한다(슬롯 좌표계).
- 참고: 내 `closed_loop_insert`(basket_tasks.py)는 핀 위 2 mm 정렬에 수렴하지 못했다(10/31). 원인은 slot 위에서 접시 흔들림과 게인으로 추정. **흔들림 감쇠**(정렬 중 속도 낮추기, 정렬 판정을 0.3 s 평균으로), **착지 후 측정 → 상승 → 이동 → 재하강을 상태 기반 단계 전환**(시간·카운터가 아니라 low와 d_bottom 조건)으로 만들면 BC가 배울 수 있다.
- 목표: 전문가 ≥ 85%(64 env, seed 8100 비교 기준 44/60).

**C2. 행동과 관측**
- 행동: 슬롯 좌표계 TCP Δ(위치 3 + 회전 3) + 그리퍼. 관절 Δ는 쓰지 않는다.
- 관측: `basket_features`(슬롯 좌표계 접시 root·법선·최저점·TCP·tool 축) 중 관절값은 빼거나 숨긴다(바구니 pose 일반화).
- 필요하면 짧은 이력(최근 0.5 s low 변화 등)을 넣는다.
- copycat 주의: 이전 행동, 관절 속도, 손가락 위치는 숨긴다(목표 1에서 확인한 교훈).

**C3. 데이터**
- clean 300 + **DART 300**: 실행 행동에 잡음을 넣고 라벨은 깨끗한 전문가 행동. 삽입 구간만.
- 성공 에피소드만 쓰고, 기록 형식은 지금 `--record`를 확장한다.

**C4. BC**
- MLP(또는 목표 1 BC 구조)를 seed 3개로 학습한다.
- 평가: `expert --bc_takeover`(계획 운반 → BC 삽입) 64 env, seed 8100. 같은 seed의 전문가와 비교한다.
- 기준: ≥ 80%, 그리고 실패 지도(`--snapshot` + section 그림).

**C5. (선택)** BC가 전문가에 못 미치면 Diffusion Policy나 action chunking을 검토한다(넣기는 다봉일 수 있다).

## 3. 실기 연결 (참고만, 오늘은 하지 않음)
- 실기는 open-loop 재생이다. 넣기 BC의 실기 사용은 "sim rollout(실제 잡은 자세 + 마커 바구니 pose) → 관절 경로 → mover"가 된다.
- 지금 실기 넣기는 `pipeline.py --place basket`(golden 티칭 경로 + 바구니 이동 + 잡기 보정, 학습 없음)이다.

## 4. 지켜야 할 것
- 속도 상한(TCP 5 cm/s, 회전 10°/s, sim 관절 drive 20°/s 이하)을 바꾸지 않는다.
- 기존 데이터·체크포인트·로그는 덮어쓰지 않는다. 새 폴더와 새 experiment 이름을 쓴다.
- `rl/basket.yaml` 값을 바꾸면 이유와 이전 값을 LOG에 남긴다. 성공 판정은 완화하지 않는다.
- 파일당 1000줄 이하. `basket_tasks.py`는 현재 약 790줄이라, 커지면 `basket_bc.py` 같은 역할 파일로 분리한다.

## 상태
| ID | 작업 | 상태 |
|---|---|---|
| C1 | 피드백 전문가 ≥ 85% | 대기 |
| C2 | 행동·관측 재설계 | 대기 |
| C3 | clean + DART 데모 | 대기 |
| C4 | BC 3 seed, takeover 평가 ≥ 80% | 대기 |
| C5 | (선택) diffusion / chunking | 대기 |

## Codex 검토 메모 (2026-10-12 09:37 KST)

코드 검토 기준: `rl/basket_tasks.py`, `rl/plate_env.py`, `rl/logs/basket_round2.log`.

- 기존 BC 결과 `0/7`, `0/1`, `0/11`에서 분모는 grasp 성공 전체가 아니라 BC가 그리퍼를 연 뒤 `live`에 남은 수다. 다음 평가는 `grasped`, `opened`, `placed_success`를 같은 전체 episode 분모로 각각 기록한다. 기존 수치는 유지하되 “안착 성공 0, 열기까지 진행한 수 7/1/11”로 해석한다.
- 현 `closed_loop_insert`는 `tries`/`backoff`에 따라 행동을 바꾸지만 이를 BC 관측에 포함하지 않는다. 오차의 0.3초 평균도 사용하지 않으며, 주석은 0.5초 정체라 하지만 코드는 `stuck > 1.0`에서 재시도한다. 새 C1 전문가는 슬롯 오차와 `low`의 측정 상태로 전이하고, 필요 상태는 관측 가능한 phase로 표현한다.
- `--closed_loop --record` 경로는 현재 연결되지 않았다: `args._record`는 할당되지 않고, closed-loop recorder의 tuple 형식은 기존 `REC`/NPZ 저장기가 기대하는 dict 형식과 다르다. 별도 삽입 데이터 recorder/schema를 먼저 구현하고, 저장 후 stage/action/episode 수와 성공 라벨을 검증하기 전에는 수집 데이터를 학습에 쓰지 않는다.
- 현 BC는 25차원 관측(관절 위치 포함)에서 관절 Δ를 예측한다. 지시서의 slot-frame TCP Δ + gripper 행동과 관절값을 뺀 slot-relative 관측으로 바꾸는 C2가 필요하며, 새 모델·데이터는 별도 경로에 저장한다.
- SafeDiffIK의 TCP/회전 제한 코드는 확인했지만, basket expert/BC eval에서 관절 20°/s 제한을 설정하는 호출은 발견되지 않았다. 저속 gate를 추가하고 실제 관절 속도 peak를 계측하기 전에는 해당 제한을 만족한다고 주장하지 않는다.
- GPU 상태: 호스트에는 RTX 5070이 있으며 `lspci`에서 PCI 장치가 보인다. 다만 Codex 명령 실행 컨텍스트에는 `/dev/nvidia*`가 없고 `nvidia-smi` driver 통신이 실패한다. Isaac 번들 Python의 torch 2.7.0+cu128도 CUDA device_count=0/NVML 초기화 실패를 반환했다. 그러므로 이번 검토에서 Isaac 실행·학습 결과는 만들지 않았다. 이를 GPU 하드웨어 부재로 해석하지 말고, Codex 명령 환경에서 device/driver 접근이 안 되는 문제로 본다.
- 전문가 성공 기준 ≥85%, BC takeover ≥80%, 기존 성공 판정과 속도 상한은 유지한다. 재시도 소진 뒤 무조건 여는 동작은 실패 라벨·실패 분석으로 분명히 구분하며 성공으로 세지 않는다.

## Claude 추가 (2026-10-12 16:xx)
- Codex 검토 확인함. Codex 실행 환경엔 GPU가 없어 Isaac 작업은 Claude가 이 PC에서 돌린다. Codex는 코드(피드백 전문가 FSM, slot-frame recorder/schema, 저속 계측 gate) 위주로.
- **sim 바구니 형상 변경:** 실측 빗살(레일 4.9 cm, 끝 높이 약 11.5 cm, −x 12.5°) → `rl/basket.yaml pins`(height 0.118, lean_with_plate true, lean_deg 12.5, rail_z 0.049). `rl/assets/basket_full.usda`는 아직 재생성 안 함(Isaac python 필요).
- **실기에서 성공한 넣기 값**(전문가 목표로 사용): 접시 윗부분 +x로 15° 기울임(넣기 때 수직에서 약 2°), 핀 줄 방향 +4 cm, 깊이 티칭보다 2 cm 위, 핀 안 0.25°/s. 값은 `calibration/config.yaml plate_ik_bc`.
