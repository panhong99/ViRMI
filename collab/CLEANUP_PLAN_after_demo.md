# 시연 후 정리 계획 (11/3 핸드 시연 끝난 뒤 실행)

작성 2026-10-29. 아직 아무것도 지우거나 옮기지 않았다. 항목마다 보스 확인 후 실행한다.
규칙: 지우기 전에 대상 확인(크기·마지막 사용), 코드는 테스트 통과 후 커밋, 데이터·가중치는 git에 넣지 않는다.

## 1. 지금 상태 (10/29 기준)

| 위치 | 크기 | 내용 | git |
|---|---|---|---|
| `calibration/data/slot_cnn/` | 37 GB | 슬롯 CNN 시뮬 데이터 sim_v1~v5 (v4 9 GB, v2·v3 각 6 GB) | 제외 |
| `calibration/data/basket`, `real_pipeline` | 1.5 GB, 1.3 GB | 바구니 측정·그리퍼 실기 기록 | 제외 |
| `calibration/data/rl_plans` | 67 MB | 실행마다 만들어지는 plan.json (30분 지나면 무효) | 제외 |
| `.envs/` | 8.9 GB | curobo 등 빌드 환경 | 제외 |
| `IL/data_*` | 1.4 GB | 10/08~10/13 BC 데이터 30여 개 | 미추적 |
| `rl/logs`, `rl/reports` | 1.1 GB, 122 MB | RL 학습 로그 | 미추적 |
| `outputs/` | 137 MB | 슬롯 CNN 실행 결과 20여 개, 발표 자료 | 미추적 |
| `mobileclip_blt.ts` (최상위, `tesollo/`) | 572 MB × 2 | YOLOE 텍스트 인코더, 같은 파일 두 개 | 제외 |
| `experiments/pyroki_bm/*.json` | 1.4 MB | 계획기 벤치 결과 | 미추적 |
| `collab/` | 문서 50여 개 | 날짜별 PLAN/HANDOFF/REPORT | 일부 추적 |

## 2. git에 꼭 넣어야 하는데 아직 안 들어간 것 (시연 재현에 필요)

- [ ] `tesollo/taught_inserts.json` — 보스 티칭 삽입 자세. 없으면 접시 삽입이 계획기로 돌아감
- [ ] `tesollo/grasp_contact.json` — 마지막 파지 손가락 관절 (없으면 첫 파지는 미리 닫기 없이 동작, 선택)
- [ ] `tesollo/teach_log_20261029_110607.jsonl` — 티칭 중 관절 기록 (근거 자료, 선택)
- [ ] 보스 수정분 `calibration/run.sh`, `real_time_project/main.py` — 보스가 직접 확인 후 커밋

## 3. 데이터 정리

1. 보관할 것만 `~/pan/archive_6D_pose/<날짜>/`로 옮기고, 목록(경로·크기·용도)을 `collab/DATA_INDEX.md`에 남긴다.
   - 보관: 실기 기록(`real_pipeline`, `basket`), 최종 모델 학습에 쓴 데이터(슬롯 CNN 최종 버전 1개, BC 최종 1개)
   - 삭제 후보: 슬롯 CNN sim_v1~v3 (v4·v5로 대체, 약 17 GB), 10/08~10/09 IL 탐색 데이터, `rl_plans`의 10/29 이전 계획
2. `mobileclip_blt.ts` 중복 1개 제거하고 최상위 하나만 둔다. YOLOE 실행 위치를 최상위로 고정한다 (지금은 `calibration/`에서 돌리면 다시 받음 → 코드에서 경로 지정).
3. `outputs/`는 발표·보고에 쓴 것만 남기고 나머지는 보관소로.

## 4. 코드 정리

| 대상 | 문제 | 방향 |
|---|---|---|
| `calibration/plate_plans.py` 2245줄, `camera_tools.py` 2137줄, `pipeline.py` 1987줄 | 1000줄 넘음 (코드 규칙 위반) | 역할별 분리: 바구니 모델 / 계획 / 검사 |
| `calibration/` 안 그리퍼 전용 코드와 핸드 공용 코드가 섞임 | 시연 A가 B 폴더의 파일을 씀 | 공용(`camera`, `robot`, `basket`)과 손별(`gripper/`, `tesollo/`) 폴더로 |
| `tesollo/dg5fs_sdk.py` 766줄 | 인식·팔·손·바구니가 한 파일 | 손 제어 / 시연 순서 / 바구니 선택으로 나눔 |
| `tesollo/basket_plan.py`의 접시 계획기 경로 (`--planner`, `--gripper_path`, `sparse_demo_plan`) | 티칭 삽입으로 대체됨 | 시연 후 안 쓰면 archive로 |
| `calibration/archive`, `rl/archive` (67개 추적 중) | 옛 코드 | git에서 빼고 태그로 남김 |
| 최상위 `checking_yoloe.py`, `datareader.py`, `pose_sender_kimm.py`, `run.log` | 용도 불명확 | 확인 후 이동/삭제 |
| `.gitignore` | `IL/data_*`, `rl/logs`, `outputs/slot_cnn_*`, `*.ts`, `.envs/`가 빠짐 | 추가 |

## 5. 문서 정리

- `README.md`: 10/29에 시연 A(핸드)를 앞에 추가함. 정리 후 폴더 구조가 바뀌면 경로 갱신.
- `collab/`의 날짜별 PLAN/HANDOFF/MORNING_REPORT는 `collab/archive/`로 옮기고, 남길 것은 `WORK_RULES`, `CONTEXT`, `DECISIONS`, `LOG`, `TASKBOARD`, `SYSTEM_SPEC`, `HUMAN_NOTES`.

## 6. 순서

1. (시연 직후) 2번 커밋 → 태그 `demo-20261103` (정리 전 상태로 언제든 돌아갈 수 있게)
2. 3번 데이터 정리 (보스 확인 후 이동/삭제)
3. 4번 코드 정리 — 한 번에 한 파일, 매번 `calibration/tests.py` 통과 후 커밋, 시연 명령이 그대로 도는지 확인
4. 5번 문서, README 경로 갱신
