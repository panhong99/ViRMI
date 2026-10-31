# 지금 상태 (인수인계, 2026-10-30 13:40, VS Code Claude → 마리코·티와리)

이 파일 하나로 지금까지의 실기 진행을 이어받는다. 세부 근거는 collab/LOG.md 10/29–10/30, 커밋 메시지.

## 0. 보스 규칙 (항상)
- 보스라고 부르고 한국어, 짧게, 수치 중심. 보고는 4섹션(받은 일 / 작업 설계 / 결과 / 개선).
- 로봇·핸드 동작 명령은 절대 직접 실행하지 않는다. 보스가 실행한다. 읽기 전용 관절·카메라 읽기는 된다.
- 프로세스 종료는 PID로만. git push는 보스만. 새 기능은 opt-in(기본 동작 유지).
- 커밋은 테스트 종료코드로만 게이트: `~/miniconda3/envs/foundationpose/bin/python calibration/tests.py > log && git commit` (grep 금지).
- 보스 소유가 아닌 미커밋 파일은 커밋하지 않는다: calibration/run.sh, real_time_project/main.py, outputs/kimm_ai_award_20261027/DAILY_LOG.md(일일 기록, 커밋 안 함).
- 속도는 CLI 인자. 임의로 낮추지 않는다(제안만). 높이를 말할 때 TCP와 접시 최저점을 구분.
- 보스에게 주는 명령은 전체 절대경로, 검증한 것만.

## 1. 시연 (핸드 DG-5F-S + M1013, 11/3 데모)
- 안전 데모(태그 demo-safe-20261029, 바구니 고정): `bash /home/pan/pan/6D_pose/tesollo/demo_safe.sh`
  - 태그 이후 바뀐 파일이 있으면 묻는다 → `yes`. 추가 인자는 그대로 dg5fs_sdk.py로 전달.
  - `--new_demo`: 오늘 넣은 접시 기억(/tmp/tesollo_demo_state.json) 초기화. 기억이 Q3 꽉 참이면 "no empty quadrant"로 멈춘다.
- 10/30 현재 가장 잘 된 접시 명령 (Q3 pin 6, L5/R5 · L7/R6로 들어감):
  `bash /home/pan/pan/6D_pose/tesollo/demo_safe.sh --taught_pivot slot --inhand_check --inhand_yaw --slot_dy 30 --slot_dx -17 --new_demo`
  - `--taught_pivot slot` : 손목 25° 덜 젖히기의 기준점을 실제 접시 밑(티칭 칸 바닥)으로. 기존(model)은 179 mm 떨어진 모델 점이라 4칸 밀렸다(L1/L2 원인).
  - `--inhand_check` : 30 cm 들어 올린 위치에서 멈추고 고정 카메라 depth로 손 안 접시 측정(experiments/inhand/plate_inhand.py, 0.1 s).
  - `--inhand_yaw` : 측정 yaw를 빼서 재계획(taught_plan --yaw_corr, J1-J3·J5·J6, J4 고정; yaw 16°에 J6 ~17°). 0.5 s.
  - `--slot_dy / --slot_dx [mm]` : 삽입 전체를 바구니 기준으로 이동(+dy = L 줄 쪽, 칸 간격 17.4 mm).
  - 주의: slot_dx는 모든 칸에 같이 적용 → 두 번째(뒤쪽, pin 10–12) 접시는 프레임 쪽으로 더 간다. 13:21 pin 12 시도에서 프레임에 박을 뻔해 보스 비상정지.
- bowl (10/30 수정, basket_plan.py): 상승 중 J4 회전(손목 특이점) 해결, 2 cm 더 하강, plate와 속도 맞춤(40 vs 43 cm/s).
- 결과 기록: outputs/inhand_20261030/ (측정 json/png, success_20261030_1319.json = 성공 실행 관절값). 관절 기록기(읽기 전용)가 tesollo/teach_log_20261029_110607.jsonl에 계속 쌓는다.

## 2. 지금 남은 문제 (우선순위)
1. 잡을 때마다 손 안 yaw가 14–26°로 흔들림 → 보정 후에도 ±1칸. 보스 판단: 근본은 TCP 파지 자세를 일정하게 하는 것.
   후보: (A) 손바닥/엄지를 접시 테두리에 먼저 대고 대칭으로 천천히 닫기(소프트웨어) (B) 받침대 멈춤턱/V홈 (C) 들어서 V받침에 내려놓고 다시 잡기.
2. 뒤쪽 칸(pin 10–12) 프레임 여유 계산 후 안전한 최대 칸 제안 (미완).
3. 같은 조건 반복으로 흔들림 크기 측정(지금 n=2–4). slot_dx는 두 결과 보간상 약 −11 mm.
4. 대기 과제(보스): 접시가 테이블 뒤쪽에 있어도 잡는 접근 경로 생성.
5. 고정 카메라 외부 보정 의심: 테이블 면이 0..−30 mm로 기울어 보임(약 3°). 데모 전 재보정은 보스 결정.

## 3. 측면 카메라 (338122303684, 측벽) 슬롯 CNN
- 실시간 GUI: `cd /home/pan/pan/6D_pose && ~/miniconda3/envs/foundationpose/bin/python experiments/side_cam/live_view.py --pin 6 [--record 5]` (24 fps, 녹화 시 depth 포함).
- 번호는 빈 바구니 기준(calibration/data/side_cam/tine_ref.json)에 맞춰 붙임 → 접시가 가려도 칸 유지. pin k = 클립(먼 쪽)부터 k번째와 k+1번째 빗살 사이.
- 한계: 접시를 거의 옆면으로 봐서 자세(yaw/기울기) 측정은 불가(보스 정답으로 확인). 칸 표시·녹화용.

## 4. 도킹 RL / 시뮬 (10/29 밤)
- 운동학 env 84–99 %, Isaac 물리 재생 22–86 %, 순응 재생 0 % → 시뮬은 실기 근거가 못 됨(LOG 10/30 아침 보고). 실기 측정+보정으로 전환.

## 5. 협업
- 티와리(Codex)는 TASKBOARD 할 일, 서로 커밋 검토. 보스는 결과·승인만 받는다.
- 보스 확인이 필요한 건 "보스 확인 필요:" 줄로.

## 6. 점군 목표 도킹 RL (10/30 15:00– 주말, 보스 지시: GT 점군에 맞추며 P 제어기를 RL로 점진 제거)
- 코드: `rl/dock_env.py`(opt-in `pc_goal, pc_disc_dist, pc_yaw, angle_tol_deg, speed_action, phys_critic, freeze_in_tines`), `rl/dock_pc_train.py`(PointNet PPO, P 가중치·반경 커리큘럼, `--export_physics`), `rl/dock_pc_view.py`(GUI), `rl/phys_critic.py`(실험, 버그 데이터로 학습돼 보류).
- 실기 후보 **v11** `outputs/dock_rl_20261030/pc_rand_v11/final.pt`(v7 + 무작위 점·가림·측정/목표 오차; Isaac 랜덤화 78.6 %, 깨끗 81.2 %). 이전 최고 v7 `outputs/dock_rl_20261030/pc_p40_v7/final.pt`: Isaac 물리 77.4 % / 81.1 %(seed 51024/61024) vs P 43.9 / 50.6 %. 핵심 = 피치 잔차 한계 40°(가르친 경로가 ~10° 치우침) + 각도 판정 6°(20°는 물리에서 넘어지는 접시를 성공으로 셈) + 목표 x +10 mm.
- 10/30 22:3x 전 물리 수치는 export 버그(바닥 접촉 뒤 knot 잔차 0)로 무효. 상세 LOG 10/30 22:3x.
- 보스 확인 필요: 실기 손목이 ±40° 피치 보정을 낼 수 있는지(지금 실기 `--wrist_less 25`).
- **실기 실행기(10/31, 미실행)**: `bash /home/pan/pan/6D_pose/tesollo/demo_safe.sh --taught_pivot slot --inhand_check --policy_insert --new_demo [--leg_enter]` — 손 안 측정 뒤 v11 보정 경로로 삽입, 거부되면 기존 티칭 경로. 상세 LOG 10/31 14:0x. 보스가 월요일 확인.
- PPT(포트폴리오 3–7장) 자료: `outputs/portfolio_nis_20261031/`. 월요일: 실기 핸드 삽입 영상(6장 GIF 교체), 가우시안 스플래팅용 셀 촬영.
