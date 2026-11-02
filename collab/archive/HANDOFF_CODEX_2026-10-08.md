# Codex 인수인계: 목표 1(임의 위치 파지) 마무리 — 2026-10-08 Claude 작성

Human은 10/9(금)~10/11(일) 동안 자리에 없다. **이 기간 실로봇과 그리퍼 명령은 금지**(Human만 실행). 시뮬레이션, 기록 분석, 코드 작업만 한다.
실기 확인은 10/12(월) Human과 함께 한다. Claude는 같은 기간 목표 2(바구니)를 맡는다. 계획표는 `collab/PLAN_2026-10-08.md`의 A1·A1b·A2·A3이다.
작업을 시작하면 PLAN 상태를 `진행(Codex)`로 바꾸고, 끝나면 `완료`로 바꾼 뒤 결과(수치·경로)를 LOG에 남긴다.

## 0. 현재 시스템 (읽고 시작)

**실기 흐름** (`calibration/pipeline.py --task plate --plate_policy ik_bc --execute`):
1. FoundationPose로 접시 root를 구한다(`plate_root_in_base`).
2. `plate_plans.library_target`으로 반경 r과 방위 az를 구한다. 이때 offset을 az만큼 회전해서 더한다.
3. `plate_plans.plan_from_library`가 경로를 만든다.
   - 라이브러리에서 가장 가까운 성공 반경을 고르고, 1축에 az를 더한다.
   - 다리 구성: 홈 → q_pre(관절 1회 이동) → 남은 직선과 BC 잡기·들기.
   - FK 셀 간섭 검사: 프레임·D455 ≥ 50 mm, 테이블 ≥ 20 mm.
4. mover가 구간마다 smooth spline으로 6°/s 실행한다. 그리퍼는 Human이 고른다.

**라이브러리** (**실로봇 미검증**, 10/8 실기 5회는 `--no_library` 모드였다) `calibration/data/plan_library/20261008_133418/library.npz`
- 만드는 명령: `rl/eval.py library`
- 반경 1.000~1.320 m, 2 mm 간격, 161개. 성공 142개(1.038~1.320 m).
- 들어 있는 것: `radius_m`, `q_deg` (T+1, n, 6), `q_pre_deg`, `close_step`, `success_step`, `ok`, `touched_plate`, `home_deg`

**BC** `~/IsaacLab/logs/rsl_rl/m1013_2f85_ik_sweep_pw1_nofinger/2026-10-08_12-25-33_stage2_bc/model_0.pt`
- 입력 46차원 중 숨기는 것: 팔 관절 [0,6], 손가락 [6,8], 관절 속도 [8,16], TCP xy [16,18], 접시 xy [25,27], target [31,34], 이전 action [36,43]. rim_rel ×20.
- 데이터: `IL/data_wide2_claude(_dart)` 400개 + `IL/data_ikwide_clean/_dart` 800개
- phase_weight 1.0, 4000 step, seed 3
- **손가락 입력을 보이면 copycat으로 성공률이 0%까지 떨어진다**(LOG 12:35).

**설정** `calibration/config.yaml`
- `plate_ik_bc` 블록: checkpoint, library, max_azimuth_deg 60, tool 0.165/0.06
- `plate_rl.offset_m` = [0.025, 0.040]

**Human 피드백 (10/8 실기 5회)**
- "파지 높이가 조금씩 어긋난다. 그 외에는 괜찮다."
- "조금 더 아래로 내려간 느낌" 1회
- 경로가 "앞으로 내려왔다가 접시 쪽으로 간다". 이것은 이미 홈 → q_pre 직접 이동으로 바꿨다.

## A1. 파지 높이 편향 분석 (먼저, 반나절)
- **입력**: `calibration/data/real_pipeline/20261008_1[23]*_plate_*/robot_state.jsonl`
  - `kind=state`: posj(deg), posx(mm, flange)
  - `kind=fp`: plate_root(m)
  - `kind=event`: gripper where=grasp
  - 성공한 실행만 쓴다(event에 `lifted`가 있는 실행).
- **회차별 계산**:
  1. gripper close 시점의 posj → `robot_model.Kinematics().fk(rad)` @ [0, 0, 0.165] → TCP z
  2. 같은 시점 직전 1 s 안의 FP plate_root z 중앙값
  3. 두 값의 차 Δ = TCP z − plate z
- **sim 기준값**: 라이브러리 `q_deg[close_step, i]`의 TCP z − sim 접시 root z(`rl/config.yaml` `geometry.table_top_z + box_h` = 0.0946).
- **결론 형식**: 회차별 (r, az, Δ_real, Δ_sim, Δ_real − Δ_sim) 표와 평균·표준편차.
  - 표준편차 < 2 mm → **고정 z 보정 하나**(A1b-1)
  - 그 이상이거나 FP z와 상관이 크면 → **높이 축 라이브러리**(A1b-2)
- 결과는 LOG와 `rl/reports/20261009_grasp_height/`(JSON, 그림)에 남긴다. 그림 규칙: grid 없음, 위·오른쪽 축선 없음, 전체 제목 없음.

## A1b. 높이 보정 반영
- **A1b-1 (고정 보정)**: sim 접시를 dz만큼 올리거나 내린 라이브러리를 다시 만든다.
  - 경로는 base 좌표로 재생되므로, sim 접시가 +dz면 실제 그리퍼도 +dz 높게 잡는다.
  - `rl/eval.py run_library`의 `place()`에서 `pose[:, 2] = pe.REST_Z + dz`, `box_under_plate`가 받침을 따라 올리는지 확인한다.
  - `--plate_dz` 인자를 추가한다.
- **A1b-2 (높이 축)**: dz ∈ {−8, −4, 0, +4, +8} mm로 라이브러리 5개(또는 축 하나)를 만든다. `plate_plans.plan_from_library`가 FP z(오프셋 적용)로 가장 가까운 dz를 고른다.
- 완료 기준: sim에서 dz별 성공 반경 대역 보고, config 갱신은 **새 경로로만** 한다(기존 라이브러리는 덮어쓰지 않음). 실기 확인은 10/12 Human.

## A2 + A3. 범위를 테이블 전체로 + 경로 최적화 (주말)
**목표**
- 반경 0.85~1.32 m, 프레임 안쪽(base x 0.87~1.63, |y| ≤ 0.52)에서 팔이 닿는 모든 곳
- 지금 실패하는 곳: 1.04 m 미만(받침·팔 충돌), 프레임 기둥 근처(간섭 거부)

**방법**
1. **전문가 확장** (`IL/collect.py`)
   - 접근 방위 φ: 반경 방향 대비 ±40°
   - 접근 pitch: 그리퍼를 더 세우거나 눕힘
   - 둘 다 에피소드마다 무작위로 둔다. 지금 전문가는 ready 자세의 tool 방향 고정 + 평행이동만 한다.
   - φ/pitch를 쓰려면 rim 목표점(`plate_env.RIM_OFFSET`, 지금은 접시 기준 고정 방향)을 φ만큼 돌리고, 시작 자세(ready)도 그 방향으로 맞춰야 한다. `plate_env.py`의 rim 정의, `reset_to_ready`, `SafeDiffIKAction` orientation latch를 같이 봐야 한다.
2. **BC**: 같은 입력 설계(평행이동 불변, 손가락 숨김, phase_weight 1)로 기존 데이터와 새 데이터를 같이 학습한다. tool_axes 입력이 φ/pitch를 구분한다.
   - 확인: 3 seed, IK 접근 gate, `--dump` 실패 지도
3. **라이브러리 격자 (r, φ[, dz])**: `run_library`를 φ 축으로 확장한다(env 수 = r × φ). 실행 시 `plan_from_library`가 후보 φ 중 셀 간섭 여유가 가장 크고 성공한 것을 고른다.
4. **경로 최적화 (A3)**: 라이브러리 생성 후 각 경로를 오프라인으로 다듬는다.
   - 홈 → q_pre: 관절 직선 대신 관절 이동량·시간 최소, 간섭 여유 ≥ margin(+10 mm)인 부드러운 경로(`robot_model.CollisionChecker`, `smooth_path` / `time_parameterize` 참고)
   - BC 구간: 경유점 thinning만 한다(학습한 동작이므로 바꾸지 않는다).
   - 전후 비교표: 이동 시간(6°/s 기준), 관절 이동량, 최소 간섭 여유, 경유점 수

**완료 기준**
- sim gate ≥95%(위 영역 균일 샘플 1024)
- 라이브러리 오프라인 시험: 영역 격자 5 cm마다 plan 생성, 실패 지도 그림
- 실기는 10/12

## 지켜야 할 것
- 속도: TCP 5 cm/s, 회전 10°/s, 실기 6°/s 상한을 바꾸지 않는다. 시뮬레이션의 `--approach_fast`는 경로 계산 전용이다.
- RL을 쓴다면 보상은 [−1, 1], 최대 ±10. 매 chunk 결정적 평가를 하고 BC보다 5%p 떨어지면 중단한다. 이번 범위에서는 필요 없을 것으로 본다.
- 기존 체크포인트·라이브러리·데이터는 덮어쓰지 않는다. 새 experiment 이름과 새 날짜 폴더를 쓴다.
- `calibration/config.yaml`을 바꾸면(checkpoint, library) LOG에 이전 값을 남긴다. `plate_rl`(기존 모드)는 건드리지 않는다.
- GPU: Claude도 바구니 sim 작업을 한다. Isaac 대형 작업(4096 env)은 LOG에 시작과 끝을 남긴다. 프로세스는 PID로만 종료한다(`pkill -f` 금지).
- 파일 규칙: 역할별 몇 개 파일, 기능은 함수/모드로, 옵션이 많으면 YAML, 파일당 1000줄 이하.
- 끝나면 LOG에 "finish codex now your turn"를 남긴다.
