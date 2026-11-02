# Codex 작업 요청 (10/13 저녁, 보스 → Codex, 정리: Claude)

먼저 읽기: `CLAUDE.md`, `collab/PLAN_2026-10-22_demo.md`(10/22 시연 목표와 설계), `collab/LOG.md` 10/13 항목, `collab/METRICS_basket.md`.

## 배경 (한 줄씩)
- 접시 잡기 → 세척기 바구니 핀 사이에 세워 넣기(place) 실기 성공. 지금은 바구니가 로봇 기준 −y 쪽, 3사분면(가까운-왼쪽)에만 넣을 수 있음.
- 10/22 발표 목표: (1) 팔이 닿는 범위 안 어디에 바구니를 둬도 넣기(±y 양쪽), (2) 바구니 4분면 각각에 넣기, (3) 분면 안 핀 번호를 매겨 지정 번호에 넣기. 넣기 동작 자체는 학습 없이 기하 + 손목 카메라 보정.
- 도달 범위 지도 결과: 4분면 모두 가능한 바구니 중심 x 0.50~0.85 m, y −0.35~+0.35 m(`calibration/plate_plans.py basket_reach`).

## Claude가 지금 하는 것 (겹치지 말 것)
- 바구니 위치·분면별 운반/넣기 경로를 IK로 계산하는 계획기 + 바구니 벽·손에 든 접시 충돌 검사 + RViz 표시.
- 수정 중인 파일: `calibration/plate_plans.py`(quadrant 계획), `calibration/robot_model.py`(회전된 상자, 여러 검사기 합치기), `calibration/ros2_nodes.py`(viz: 계획에 담긴 바구니 자세·손 안 접시 표시). 이 세 파일은 건드리지 말 것.

## Codex에게 부탁 (우선순위 순)
### A. 손목 카메라 핀 쌍 검출 + 번호 매기기 (오프라인 프로토타입)
- 입력: 이미 저장된 손목 사진과 요청 json. `calibration/data/real_pipeline/2026101[23]_*/wrist_{above,pre}.raw.png` + 같은 폴더 `wrist_*.req.json`(T_base_flange, T_base_basket), 손목 hand-eye `calibration/data/wrist_handeye/20261006_111318_clean/result.json`(T_flange_cam), 바구니 핀 모델 `camera_tools.basket_pins_in_base(T_base_basket)`, 투영 예시 `camera_tools.wrist_snapshot`.
- 할 일: 모델 핀을 사진에 투영한 뒤, 각 투영 주변에서 실제 핀(가는 세로 철사, 지름 3~3.5 mm)을 찾아 위치를 맞추기(엣지/허프 선/템플릿 등 고전 CV부터). 좌-우로 나란히 정렬된 핀 2개 = 1쌍, **손목 화면 기준 위→아래로 1번부터** 번호. 모델 대비 보정량(mm, 바구니 x 방향)과 번호를 그린 PNG 출력.
- 결과물: `calibration/camera_tools.py`에 함수 + 모드 하나(예: `pins --req <json>`) 추가(새 스크립트 파일 만들지 말 것). 정확도 표(사진별 검출된 쌍 수, 모델 대비 보정량)를 LOG에.
- 주의: 그제 바닥 철망 구멍을 핀 머리로 오인한 적 있음. 빗살은 −x 쪽으로 약 12.5° 기울어 있음(`rl/basket.yaml` pins).
### B. 바구니 ArUco 마커 크기 실험 준비
- 마커 10 → 5 → 2.5 cm로 줄일 때 위치 흔들림·피팅 잔차 비교용 측정 모드: `camera_tools.basket_measure`를 N회 반복해 마커별 위치 표준편차, `basket_fit_robust` 잔차, 4개 인식률을 표로 저장. 마커 크기는 `calibration/config.yaml basket.marker_size_m`을 인자로 덮어쓸 수 있게.
- 실제 마커 인쇄·부착은 보스가 함. 지금 10 cm 상태로 기준값 한 번 측정 가능(고정 카메라만 사용, 로봇 이동 없음).
### C. (10/22 이후 준비) failure detection 선행 연구 조사
- 주제: 조작 중 실패(잘못된 슬롯/번호, 접촉 이상) 감지와 재시도, 손목 카메라 기반, sim2real. 5~10편 요약(문제 설정, 입력, 모델, 실험, 우리 task에 쓸 점)을 `collab/` 아래 문서 하나로.

## 규칙 (반드시)
- 실로봇·그리퍼 동작 명령 금지(보스가 직접 실행). 카메라 읽기만 가능. 프로세스 종료는 PID로만.
- 기존 캘리브레이션 결과, 체크포인트, 데이터, 라이브러리 덮어쓰기 금지(새 파일·새 폴더로).
- 코드: 역할별 기존 파일에 함수/모드로 추가, 인자가 많으면 YAML. 그림: 격자 없음, 위·오른쪽 축선 없음, 전체 제목 없음.
- 데이터·가중치는 git에 넣지 않음. 진행과 결정은 `collab/LOG.md`, `collab/DECISIONS.md`에 기록. 사용자는 "보스"로 부름.
