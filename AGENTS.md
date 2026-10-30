# Agent 협업 규칙 (Claude / Codex 공통)

> 작업 시작 시 `collab/STATE_NOW.md`(지금 상태·인수인계)를 가장 먼저 읽는다 (보스 10/30).

## 최우선 실증 조건 (Human, 2026-10-07)

- 목표는 2주 안에 실기 pick-and-place 실증. 성공률보다 사고 방지가 우선이다.
- 로봇은 항상 가장 느린 속도로 동작하도록 설계한다. 접근·파지·들기·이동·놓기·홈/ready 이동과 그리퍼 모두 포함한다.
- 학습·평가·GUI·실기 실행에 동일한 저속 조건을 적용한다. 성공률이나 시간 제한을 맞추기 위해 속도를 올리지 않는다.
- "가장 느림"의 실기 수치는 장비의 실제 설정으로 확인한다. 임의의 sim 수치를 실기 최저 속도라고 단정하지 않는다.
- 보상 벌점과 TCP 목표 변화량 제한만으로 실제 속도를 보장했다고 판단하지 않는다. 실제 TCP/관절 속도·가속도 및 파지 후 흔들림을 계측하고, 실행 제어 계층의 속도 제한을 확인한다.
- 저속에서 시간이 부족하면 에피소드 시간을 늘린다. 기존 고속 정책의 성공률은 저속·실기 배치 승인 근거로 사용하지 않는다.

Claude와 Codex가 human-in-the-loop 방식으로 context를 공유한다. 공유 파일은 모두 `collab/` 아래에 있다.

- `collab/CONTEXT.md`: 연구 목표, 가설, 성공 기준, 제약, 평가 방식, 건드리면 안 되는 것
- `collab/DECISIONS.md`: 결정과 이유의 누적 기록
- `collab/LOG.md`: 날짜별 진행 누적
- `collab/HUMAN_NOTES.md`: 사람(Human)의 의견 입력용. agent는 읽기만 하고 "처리됨" 표시만 한다

## 작업 시작 시

1. `collab/CONTEXT.md`, `collab/DECISIONS.md`, `collab/LOG.md` 최근 항목, `collab/HUMAN_NOTES.md`를 먼저 읽는다.
2. `collab/HUMAN_NOTES.md`의 미반영 의견을 우선 처리한다.

## 작업/토론 종료 시

1. 논의한 결정과 이유는 `collab/DECISIONS.md`에, 진행 상황과 다음 할 일은 `collab/LOG.md`에 기록한다.
2. 실험 결과는 metric 수치와 log 경로를 함께 남긴다.
3. 반영한 `collab/HUMAN_NOTES.md` 항목은 "처리됨"으로 표시한다. 내용은 수정하거나 삭제하지 않는다.

## 역할

- **Builder**: 구현, 실험 실행, 초안 작성
- **Critic**: 리뷰만 하고 코드는 수정하지 않는다. 동의만 하지 말고 반드시 반박 지점, 리스크, 누락된 검증을 찾는다
- 이번 세션의 역할은 Human이 프롬프트에서 지정한다. 지정이 없으면 Builder
- 역할은 고정하지 않는다. 1~2주 사용 후 Human이 조정한다

## 검증 원칙

- loss, data split, metric 계산 같은 핵심 로직은 변경 시 반드시 `collab/DECISIONS.md`에 기록하고 Human의 검토를 요청한다.
- 토론만 하지 말고 가능하면 실제로 실행해서 결과를 근거로 삼는다.
- max iteration이나 metric threshold 같은 종료 조건을 명시한다.
