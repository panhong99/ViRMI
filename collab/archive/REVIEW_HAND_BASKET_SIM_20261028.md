# CN 검토 — Tesollo 손 접시 삽입 sim

검토 범위: `rl/hand_basket.py`, `tesollo/hand_sim_job.py`, 저장 결과 `rl/logs/hand_basket/`, 그리고 실제 경로에 보정을 붙인 `tesollo/basket_plan.py --sim_corr`. 코드와 저장 결과만 읽었고 GPU·Isaac·실로봇은 실행하지 않았다.

## 결론

현재 결과는 **접시만 강제로 하강시킨 조건에서 pin별 3D 보정값을 찾을 수 있다는 가능성**을 보여 준다. 하지만 성공 판정, 손/접촉 모델, 불확실성 검증, 경로 안전 검사가 실기 배치 조건과 다르므로 `756/1024 = 73.8%`를 손 삽입 성공률로 쓰거나 CEM 보정을 실로봇에 승인할 근거는 아니다. 특히 planner가 거부한 보정을 계획 생성 뒤에 적용하는 `--sim_corr`는 기존 basket collision gate를 우회하므로 현 상태에서는 실기 기본 경로로 쓰면 안 된다.

## 핵심 반박

### 1. 성공 판정이 기존 basket `seated/exact`보다 느슨하고 측정점도 다르다 — 높음

| 항목 | `hand_basket.py` | 기존 `rl/basket` | 차이 |
|---|---|---|---|
| gap 정렬 | **접시 중심** x가 목표 gap의 pitch/2 안 | 메시의 **최저 rim 점** `d_bottom`이 slot line에서 8.5 mm 안 | 기울어진 접시는 중심과 실제로 먼저 들어가는 rim 점이 다름 |
| row 방향 | 접시 중심 y ±60 mm | `d_u` ±30 mm | 2배 느슨함 |
| 자세 | basket x축에서 ±40° | 가르친 12.8° slot plane에서 ±20° | 목표 자세가 다르고 허용각이 큼 |
| 깊이 | rim 아래 70 mm | floor 위 55 mm 이하 = basket rim 기준 약 −89 mm 이하 | 약 19 mm 얕아도 성공 가능 |
| 정지 | 2 s 후 한 프레임 판정 | 1 s 동안 위치 변화 <10 mm | 실제 안착/정지 확인 없음 |

근거: `rl/hand_basket.py:157-166`, `rl/basket.yaml:30-37`, `rl/basket_tasks.py:549-556`. 따라서 동일 장면에 두 판정을 함께 계산하기 전에는 기존 gripper basket 결과와 성공률을 직접 비교할 수 없다.

### 2. 손이 없고 접시 pose를 매 physics step 덮어쓴다 — 높음

- 손가락, 손바닥, 팔이 scene에 없다. 손가락–핀/rail 충돌, release 순간 손이 빠질 공간, 운반 중 미끄러짐을 평가하지 않는다.
- 하강 중 접시는 `write_root_pose_to_sim`으로 목표 pose에 강제 배치되고 속도도 0으로 덮어쓴다(`rl/hand_basket.py:138-145`). 접촉이 경로를 밀어내거나 로봇이 멈추는 효과가 없다.
- 기록된 pin 관통 ≤1.1 mm 검사는 **명목 H + CEM 보정, 600점 근사, pin만** 확인한 것이다. 무작위 H, 손, rail·wall·floor 및 면/edge 충돌은 확인하지 않았다. 이 검사는 강제 이동의 안전성을 충분히 입증하지 않는다.

### 3. CEM 보정이 planner의 collision 거부를 우회한다 — 높음

`tesollo/basket_plan.py:263-289`는 기존 `quadrant_plan`이 만든 경로 뒤에 CEM 이동을 적용해 IK만 다시 풀며, 기존 MultiChecker/basket collision 검사를 다시 호출하지 않는다. 주석에도 planner offset으로 넣었을 때 wall model이 전부 거부했다고 적혀 있다. Q3:8의 보정은 basket frame y +28.3 mm, 여러 pin은 z 약 ±12 mm다. sim에서 접시가 안착했다는 이유로 planner의 wall 거부를 무시할 수 없다.

실기 후보가 되려면 보정 후 전체 carry+descend를 기존 collision checker로 다시 검사하고, 손 형상과 잡힌 접시 형상을 포함해야 한다. checker가 거부하면 sim 판정과 형상 모델의 불일치를 먼저 해결해야 한다.

### 4. 불확실성 범위는 실측 분포가 아니다 — 중간/높음

- unknown 조건의 hang ±15°, grip 각축 ±15 mm, tilt ±3°와 measured 조건의 ±5°/±5 mm/±2°는 사람이 정한 독립 uniform 범위다. 실측 반복분포, 축간 상관, 계통 편향을 반영하지 않는다.
- basket pose, pin 위치·기울기, 실제 pin 직경/탄성, 접시 형상·질량·마찰, 로봇 추종오차는 무작위화하지 않는다. 현재 sim pin radius는 1.0 mm이고 문서의 실제 직경은 약 3.5 mm라 실제보다 틈이 넓다(`rl/basket.yaml:63`).
- all-pin 평가는 hang 60°, Q3 pin 6–12와 Q4 pin 4–12만 포함한다. Q1/Q2, hang 50/70, 이동한 basket에 일반화하지 않는다.

### 5. 저장 결과도 실기 승인 기준에 못 미친다 — 중간

- hang 60°, measured 범위: 무보정 `111/1024 = 10.8%`, CEM `756/1024 = 73.8%`(Wilson 95% CI 71.1–76.4%).
- pin별 CEM은 최저 Q4:12 `37/64 = 57.8%`(CI 45.6–69.1%), 최고 Q4:4 `54/64 = 84.4%`다. 모든 pin이 거의 무조건 성공해야 한다는 목표와 거리가 있다.
- CEM은 pin마다 별도 `(dx,dy,dz)`를 최적화한 lookup table이다. 새 basket pose나 새 in-hand pose를 관측해 보정하는 정책/RL이 아니다. train seed와 eval seed는 나뉘었지만 같은 simulator와 같은 명목 hang을 공유한다.

## 다음 승인 조건

1. 같은 rollout에 기존 `basket_state`의 `d_bottom`, `d_u`, `normal_err`, `low`, 1 s stillness, target gap exact를 기록하고 이를 유일한 주 성공지표로 사용한다.
2. 강제 pose 단계는 진단용으로만 남기고, 다음 평가는 DG-5F-S 충돌 형상과 grasp constraint/release를 포함한다. 손 또는 접시가 pin/rail에 닿으면 강제로 관통시키지 않고 접촉 반응을 따른다.
3. 보정 적용 뒤 carry+descend 전 구간을 기존 checker로 재검증한다. wall 거부를 우회한 경로는 실기 후보에서 제외한다.
4. `measure_held.py` 반복 측정으로 H 오차 분포를 만들고, basket pose·pin geometry·plate/friction·tracking error도 실측 범위로 randomize한다. 학습에 쓰지 않은 hang/basket pose/seed로 평가한다.
5. 최소한 각 pin 100회에서 기존 exact 판정 ≥95%, pin별 Wilson 하한도 함께 제시하고, 무작위 H 전체에서 최대 침투/충돌 0을 확인한 뒤에만 `--sim_corr` 실기를 검토한다.

현재 권고: `--sim_corr`는 계속 opt-in·plan-only로 유지하고, 보스 실증 명령에는 넣지 않는다.
