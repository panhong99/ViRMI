# 월요일 접시 없는 바구니 손목 사진 — 보스 실행 절차

목적: **현재 바구니 배치**에서 실제 Q1·Q2·Q3·Q4마다 두 줄의 빗살 시작점(root)과 끝점(tip)을 전부 볼 수 있는 손목 RGB를 저장한다. 접시를 손에 들거나 바구니에 꽂은 상태에서는 찍지 않는다. 아래 이동 명령은 **보스만 직접 실행**한다.

## 시작 전

1. 바구니와 마커를 최종 위치에 고정하고, 접시를 치운다. 10/16 `wrist_look` 저장값은 바구니 중심 **(1.2296, −0.3227, 0.1442) m, yaw +1.084°**였지만 같은 날 뒤의 pipeline 기록은 약 **(1.008, −0.175) m**였다. 월요일에 x≈0.90 m로 당기거나 마커를 다시 붙였으면 어느 저장값도 현재 위치로 간주하지 않는다.
2. 보스가 로봇을 홈에 두고 ROS 연결 및 제어 권한을 확인한다. 각 분면 촬영 **직전** 현재 마커로 바구니를 재측정해 새 계획을 만든다. `look_plans --measure`는 고정 카메라로 마커를 읽고 새 `plan.json`과 분면별 `.req.json`을 만들며 UDP로 RViz 계획 경로를 보낸다. 이 명령 자체는 로봇을 움직이지 않는다. 마커 fit 잔차가 설정 한계(기본 8 mm)를 넘거나 해당 분면이 `no look pose`/`no path`이면 이동하지 않는다.

## 한 번에 촬영 (`look4`)

위 조건(바구니·마커 고정, 바구니에서 접시 제거, 그리퍼 빈손, 로봇 홈, ROS 연결)을 확인한 뒤 기본 Q1→Q2→Q3→Q4 순서로 실행한다. 직전 offline 계획에서 x≈0.90 m일 때 네 분면 모두 look pose가 있었고, x≈1.01 m에서는 Q1 look pose가 없었다. 현재 위치에서 Q1 계획이 실패하면 억지로 진행하지 않는다.

```bash
bash calibration/run.sh look4 --execute
```

실행은 시작 확인 문구를 한 번 받고, 각 이동 leg마다 `go`를 확인한다. 분면마다 새 마커 측정·계획 → look 이동 → 손목 사진 → 홈 복귀 순서다. 일부 분면만 찍으려면 예: `LOOK_QUADRANTS="Q1 Q3" bash calibration/run.sh look4 --execute`. 촬영 실패는 이동 완료와 별도로 표시되며, 해당 분면 사진이 없으면 AZ 라벨링을 재개할 수 없다. 이 명령을 실행하기 전후의 영역 확인과 이동 승인은 보스가 직접 한다.

## Q1 → Q2 → Q3 → Q4, 분면마다 반복

아래의 `Q1`을 해당 분면으로 바꾼다. 한 분면의 **새 계획 생성 → 저속 이동 → 촬영 → 홈 복귀**를 마친 뒤 다음 분면을 계획한다. 출력된 `plan.json`과 `.req.json`의 **실제 경로**를 복사해 넣는다. 계획은 기본 30분이 지나면 mover가 거부하므로 오래된 계획·요청을 재사용하지 않는다.

```bash
conda run --no-capture-output -n foundationpose python calibration/plate_plans.py look_plans --measure --quadrants Q1

# 아래 두 경로는 바로 위 출력의 해당 분면 새 경로로 교체한다.
PLAN=/absolute/path/to/new/plan.json
REQ=/home/pan/pan/6D_pose/calibration/data/basket/wrist_look/NEW_Q1.req.json

bash calibration/run.sh mover --plan "$PLAN" --leg Q1_look --vel_deg_s 2 --acc_deg_s2 2
bash calibration/run.sh mover --plan "$PLAN" --leg Q1_look --smooth --vel_deg_s 2 --acc_deg_s2 2 --execute
conda run --no-capture-output -n foundationpose python calibration/camera_tools.py pre_photo --req "$REQ"
bash calibration/run.sh mover --plan "$PLAN" --leg Q1_home --vel_deg_s 2 --acc_deg_s2 2
bash calibration/run.sh mover --plan "$PLAN" --leg Q1_home --smooth --vel_deg_s 2 --acc_deg_s2 2 --execute
```

`mover`의 첫 호출은 검사 전용이고, `--execute`를 붙인 호출은 **실제 이동**이다. 계획기가 출력하는 예시 이동 명령의 20°/s 대신, 현재 mover 기본 저속인 **2°/s**와 가속도 **2°/s²**를 명시했다. 실제 이동은 시작 문구와 `go` 입력을 요구한다. 출발 자세가 계획과 3° 이상 다르거나 도착 오차가 1°를 넘으면 진행하지 말고 원인을 확인한다. 홈 복귀 확인 후 다음 분면으로 간다.

## 사진 확인·전달

각 `pre_photo`는 요청의 `png` 경로에 **모델 선을 그린 `.png`**, 바로 옆에 **원본 `.raw.png`**와 **메타데이터 `.json`**을 새로 저장한다. JSON에는 `K`, `T_base_flange`, `T_flange_cam`, `T_base_basket`가 들어간다. `T_base_flange`는 촬영 시 관절을 다시 읽은 값이 아니라 **계획된 도착 관절의 FK**이므로, `mover` 도착 확인을 통과했어도 실제 자세와 완전히 같다고 가정하지 않는다.

원본 4장을 확인해 각 분면의 **두 줄 전체**, 가장 먼 1번부터 마지막 빗살까지 **root와 tip 모두**, 프레임 경계·마커 종이·그리퍼 가림이 없는지 본다. 하나라도 잘렸으면 그 사진은 전수 라벨 정답에 쓰지 않는다. Q1은 이전 사진에서 가려졌으므로 특히 우선 확인한다. 보스가 각 분면의 `.raw.png`와 같은 이름의 `.json` 네 쌍, 그리고 `.req.json` 경로를 Codex에게 알려 주면 번호·선 검증을 재개한다. `run.sh wrist_look`은 현재 관절을 읽지만 **예전 taught basket pose + shift**를 쓰므로 바구니 이동 뒤 이 절차의 `--measure`/`pre_photo`를 대체하지 않는다.

이 문서는 코드와 기존 파일 형식을 읽어 작성했다. `look4` 셸 분기와 실패 흐름은 mock으로 검토했지만 위 로봇·카메라 명령의 실제 실행 및 네 분면 사진 시야는 아직 확인되지 않았다.
