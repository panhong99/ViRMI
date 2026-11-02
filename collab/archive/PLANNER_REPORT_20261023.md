# 경로 플래너 보고서 (10/23, Claude)

## 1. 받은 일

- 10/23 보스: 플래너가 어떻게 동작하는지, 입력이 무엇인지, 파이프라인 연산 순서, 출력이 어디에 쓰이는지, RRT가 언제 어떻게 쓰이는지 정리.
- 범위: 실기 명령 `run.sh pipeline --task plate --plate_policy ik_bc --place basket --insert quadrant ... --sim_q --auto --basket_source vision --markers_off --one_go` 의 경로 부분.

## 2. 작업 설계 (플래너 구조)

### 2.1 한 줄 요약

외부 플래너(MoveIt, OMPL)는 쓰지 않는다. **사람이 정한 경로 모양(규칙) + 점마다 IK + 충돌 검사 + 스플라인**이 기본이고, 규칙이 실패할 때만 RRT가 나선다. 파지 경로만 Isaac에서 학습해 둔 라이브러리를 쓴다.

### 2.2 구성 요소 (코드 위치)

| 부품 | 하는 일 | 코드 |
|---|---|---|
| 로봇 모델 | URDF(M1013)에서 FK, 수치 IK(scipy least_squares, 관절 한계 안), 관절 한계 | `calibration/robot_model.py` `Kinematics` |
| 충돌 검사기 | 셀(프레임·테이블)·바구니·서 있는 접시·장애물을 5 mm 격자 거리장(EDT)으로. 로봇 링크 + 그리퍼 원기둥 + 들고 있는 접시 점들을 검사. 여유 비율(clearance ratio) ≥ 1 이면 통과 | `robot_model.CollisionChecker`, `plate_plans.quadrant_checkers` |
| 파지 라이브러리 | Isaac BC 정책으로 미리 굴린 파지 경로 1672개(반경별, 성공 1645). 접시 반경에 가장 가까운 성공 경로를 J1 회전(방위각)으로 맞춤 | `plate_plans.plan_from_library`, `config plate_ik_bc.library` |
| 넣는 자세 탐색 | 칸 위치 + 손 안 접시 자세(H) → 접시가 빗살 줄에 수직으로 서는 손 자세. 후보 = 접시 면 2방향 × 접시 둘레 회전 −90~+90°(10~15° 간격) | `plate_plans.quadrant_plan` |
| 운반 규칙 | 들기(곧게 위) → 수평 이동 → 곧게 내리기. 들고 가는 접시의 가장 낮은 점이 바구니 벽·서 있는 접시·장애물보다 높게 | `plate_plans.high_carry` |
| RRT (예비) | 관절 공간 양방향 RRT + 지름길 + 다듬기, 바구니 위 '바닥 높이' 제약 | `robot_model.CollisionChecker.plan`, `plate_plans.rrt_carry` |
| 한 번에 가기 | 파지 후 운반·넣기를 점 줄이기(RDP) + 고른 매듭 + 등시간 스플라인 하나로 | `plate_plans.one_go_plan` |
| 실행 | 두산 `movesj`(관절 공간 스플라인) | `calibration/ros2_nodes.py` mover |

### 2.3 입력

| 입력 | 어디서 | 쓰임 |
|---|---|---|
| 고정카메라 외부 파라미터 `T_base_cam` | 손눈 보정 결과(`calib_core.latest_t_base_cam`) | 카메라 관측 → 로봇 base 좌표 |
| 접시 자세 | FoundationPose(YOLOE 'plate' 마스크) 5프레임 안정값 | 파지 반경·방위각, 들어 올린 뒤 손 안 접시 자세(H) |
| 바구니 자세 `T_base_basket` | `--basket_source vision`: 고정카메라 RGB-D + YOLOE 바구니 마스크 + 바구니 모델 맞춤(FFT 탐색 + ICP + 손잡이) | 칸 위치, 바구니 충돌 모델 |
| 목표 칸 | `--quadrant` + `--pin` 또는 `--region front/back` (10/23부터 front = 작은 번호, back = 큰 번호) | 넣는 자세 |
| 이미 꽂힌 접시 | `--basket_plates Q4:3 ...` | 충돌 모델(서 있는 접시 상자) |
| 장애물 | `--obstacles`: 고정카메라가 본 바구니 밖 접시(잡을 접시 제외), `--quadrant auto`면 bowl도 | 충돌 모델 + 운반 높이 |
| 현재 관절값 | 로봇 상태 스트림 | IK 시작점, 홈 확인 |
| 설정 | `calibration/config.yaml` `plate_ik_bc`, `basket_quadrant_sim`(분면별 기울기·겨냥) | 여유, 상승 높이, 속도 |

### 2.4 파이프라인 연산 순서 (`calibration/pipeline.py` `run_plate`)

1. **[1/5] 로봇 확인**: 관절값 수신, 홈 자세인지 확인.
2. **인식 시작**: YOLOE + FoundationPose 프로세스(고정카메라 소유, 0.5 s마다 프레임 공유).
3. **바구니 자세** (vision): 공유 프레임 5장 → `basket_vision.py` → 프레임끼리 6 mm / 1.5° 안에서 맞아야 통과. 10/23 개선 후 프레임당 약 1.1 s(전에는 6–8 s).
4. **[2/5] 접시 인식**: FP 자세가 안정되고 라이브러리 반경 범위 안에 들면 다음 단계.
5. **파지 경로**: 라이브러리에서 가장 가까운 반경 후보들 → 셀 충돌 검사 → 여유가 가장 큰 것(`to_grasp`, `lift`).
6. **넣기 계획** (`quadrant_plan`):
   1. 칸 중심 + 분면별 보정(sim_q: 기울기, 겨냥, 높이) → 넣는 손 자세 후보.
   2. 후보마다 칸 위 11 cm → 꽂힘까지 1 cm 간격 13점 IK. 이웃 점에서 관절이 8° 넘게 튀거나 한계 3° 안이면 버림(`ik/limit`).
   3. 내리기 충돌 검사(벽, 그리퍼·팔 vs 서 있는 접시). 들고 있는 접시 vs 서 있는 접시는 일부러 빼고 봄(옆 칸으로 포개지듯 내려가는 것이 정상).
   4. 운반: 규칙(`high_carry`) — 위로 올린 높이(`basket_carry_rise_m` 30 cm, 또는 `--carry_rise`), 바닥 = 바구니 벽·서 있는 접시·장애물 꼭대기 + 여유. 실패하면 설정에 따라 RRT.
   5. 홈 복귀(`to_home`), 실패 시 복귀(`abort_home`).
7. **한 번에 가기** (`--one_go`): 파지 뒤 운반 + 넣기 79점 → 23점 안팎으로 줄여 스플라인 하나. 다시 충돌 검사(놓기 전 = 운반 검사기, 놓은 뒤 = 넣기 검사기), 내리기 수직 ≤ 2 mm.
8. **미리보기**: `preview_plan.json` → RViz(`run.sh viz`).
9. **[3–5/5] 실행** (`--execute`): 접근·파지(멈춤) → 한 번에 가기(멈추지 않음) → 놓기 → 복귀. 속도 `--vel_deg_s`.

### 2.5 출력과 쓰이는 곳

| 출력 | 파일 | 쓰는 곳 |
|---|---|---|
| 파지 계획 | `calibration/data/rl_plans/<시각>/plan.json` (`to_grasp`, `lift`) | mover(movesj), RViz |
| 넣기 계획 | `.../<시각>_taught/plan.json` (`to_above`, `descend`, `ascend`, `to_home`, `abort_home`, `quadrant_plan` 보고) | mover, RViz, 한 번에 가기 입력 |
| 한 번에 가기 | `.../<시각>_one_go/plan.json` (`one_go` + `drop_knot`, `retreat`) | mover |
| 미리보기 | `calibration/data/real_pipeline/<run>/preview_plan.json` (경로 + 바구니 + 장애물 + 비교 경로) | RViz |
| 실행 기록 | `.../<run>/robot_state.jsonl` (관절·힘·이벤트) | 사후 분석, 장애물 데모 재생, sim 재현 |
| 착지 기록 | (실기 후) 사진·핀 | 삽입 RL 학습 데이터 |

### 2.6 RRT는 언제, 어떻게

- **기본 실기 경로에서는 쓰지 않는다.** `CARRY_PLANNER=direct`(기본)는 규칙 운반만 쓰고 RRT 예비를 끈다(`pipeline.configure_carry_planner`). 규칙이 실패하면 경로 거부.
- **켜지는 경우**
  1. `--obstacles`: 규칙(위로 넘기)을 먼저 모든 후보로 시도하고, 하나도 안 될 때 RRT로 장애물 옆을 돌아감(`basket_carry_rrt_fallback`).
  2. 오프라인 계획 도구(`plate_plans.py random_plans / quadrant_plans`): config `basket_carry_mode: rrt`, `basket_carry_rrt_fallback: true` 그대로 사용.
  3. `CARRY_PLANNER=curobo`(opt-in): cuRobo → 경로 라이브러리 → RRT 순서.
- **방식**: 관절 공간에서 시작·목표 자세 둘레 ±90°(span) 안을 무작위로 뽑아 두 나무를 키우고, 이어지면 지름길(shortcut) + 다듬기(tidy). 바구니 위에서는 접시 최저점이 바닥 높이보다 낮으면 충돌로 봄(`FloorCheck`). 손목(J4, J6)이 시작보다 100° 넘게 돌면 버리고 다음 seed. 노드 상한 1500–4000.
- **비용**: 수 초~수십 초(충돌 검사를 점마다). 규칙은 0.5–2 s.
- 결론: RRT는 **길을 찾는 도구**(스무딩 아님). 스무딩은 한 번에 가기 단계가 따로 한다.

## 3. 결과 (10/23 기준 숫자)

| 항목 | 값 | 근거 |
|---|---|---|
| 계획만 걸린 시간 (실기 09:22) | 80 s = 인식 시작 20 s + 바구니 42 s + 접시·계획 18 s | `real_pipeline/20261023_092240_plate_7kqy_mx2` |
| 바구니 인식 개선 | 프레임당 5–6 s → 1.1 s, 자세 동일 | 커밋 b8bb9c2 |
| 한 번에 가기 | 79 → 23점, 예상 36 → 14 s, 수직 벗어남 ≤ 0.3 mm | 10/22 오프라인 |
| 접시 위치 넓힘 (sim 300) | 성공 32 %, 실패: 0.85 m 안쪽 파지 경로 없음(라이브러리 범위), y 0.40 + x < 0.95 운반 관절 한계 | `calibration/data/basket/random_20261023_094101` |
| 바구니 좌우 넓힘 (sim 300, 접시 고정) | 성공 54 %. Q3 76 %, Q4 29 %. 실패 대부분 넣는 자세 IK/관절 한계 | `calibration/data/basket/random_20261023_111458` |
| Q4 실패 원인 확인 | IK 시작점 16가지로 다시 풀어도 6/24 그대로 → 해를 못 찾은 것이 아니라 파지 때 정해진 손 안 접시 각도로는 그 칸의 넣는 자세가 관절 한계 밖 | 10/23 실험 |
| 두 접시 (실기 설정, 규칙 운반) | Q4 3 → 7: 무시해도 5.7 cm(운반 0.371 m). 상승 10 cm로 낮추면 무시 0.2 cm → 회피 0.3–0.9 cm(실기엔 위험) | 10/23 실험 |

## 4. 개선 (다음 할 일)

1. **파지 방향을 목표 칸에 맞춰 고르기** (Q4 대책): 넣는 자세가 관절 한계 안에 들어오는 파지 접근 방향(라이브러리 phi)을 먼저 고른다. 파지 후 손 안 접시 각도가 바뀌므로 Q4 성공률이 오르는지 같은 300개로 비교.
2. **운반 규칙에 '옆으로 돌기' 추가**: 지금 규칙은 위로 넘기만 함. 서 있는 접시·bowl 옆을 지나는 후보를 규칙에 넣으면 RRT 없이 빠르게 피할 수 있음.
3. **여유 기준**: 들고 있는 접시 vs 서 있는 접시 운반 여유 1 cm는 실기에 빠듯함 → 운반 구간만 2–3 cm로(옵션).
4. **계획 시간**: 서 있는 접시가 있으면 6–8 s(충돌 거리장 새로 만들기 + 후보 반복). 거리장 캐시 재사용, 후보 순서(지난 성공 phi 먼저)로 줄이기.
5. **실기 확인 대기**: 바구니 인식 속도(약 10 s 예상), `--keep_face` 두 접시, 11/3 시연 흐름(`--quadrant auto --demo_view`).
