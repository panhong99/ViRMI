# 접시–바구니 실패 감지·복구 선행 연구 (Codex, 2026-10-13)

목적: 10/22 시연 이후, 지정 핀 번호 오류·삽입 정렬 실패·접촉 이상·놓은 뒤 불안정을 감지하고 복구하는 연구 설계. 아래 논문 사실과 우리 적용안(추론)을 구분한다. 원문·저자 프로젝트·공식 코드만 근거로 사용했으며 우리 장비에서 재현한 성능은 아니다.

## 읽을 순서

1. **Farid et al. + FINO-Net**: 손목 시계열로 실패를 분류하고, 놓친 실패와 불필요한 정지를 따로 평가.
2. **Sentinel + REFLECT**: 빠른 이상 감지와 느린 진행/원인 설명을 분리.
3. **AHA + TRANSIC**: 실패 사례를 시뮬레이션에서 생성하고 실제 보정 데이터로 간극을 줄임.
4. **Recovery RL**: 검증된 복구 동작이 확보된 뒤 별도 복구 정책을 고려.

| 연구 | 입력 | 모델/구조 | 실험 | 우리 task에서 검토할 점 |
|---|---|---|---|---|
| REFLECT (CoRL 2023) | 다중 센서 관측의 계층적 사건 요약 | 요약 → LLM 실패 설명 → 수정 계획 | RoboFail의 실행·계획 실패 | 핀 ID·접시 축·접촉·그리퍼·정지 이벤트를 구조화 |
| AHA (ICLR 2025) | 시각 관측과 작업/질문 | 실패 설명용 VLM, FailGen 시뮬 실패 데이터 | 시뮬 학습 후 실제·새 task 실패 reasoning 및 downstream recovery | 옆 칸/기울기/미안착 실패 taxonomy와 라벨 설계 |
| Sentinel (2024) | 시간별 정책 action chunk와 작업 영상 | 행동 일관성 통계 + VLM 진행 감시 | 시뮬·실기 mobile manipulation/diffusion policy | 목표에 안 다가가는 정체와 갑자기 흔들리는 행동을 별도로 탐지 |
| FINO-Net (2023) | RGB·depth·audio 시계열 | CNN/ConvLSTM, RGB-D early fusion + audio late fusion | FAILURE 데이터의 여러 tabletop 조작·후속 실패 | 접촉 중과 놓은 뒤 실패를 각각 관찰 |
| TRANSIC (CoRL 2024) | 시뮬 정책 상태와 인간의 실제 correction | 시뮬 base + correction으로 학습한 residual | 접촉이 많은 실기 furniture assembly 등 | 실패 전후 자세와 보스의 수정량을 짝으로 기록 |
| Recovery RL (arXiv 2020) | 상태·제약 위반 경험 | task policy + safety critic/recovery policy | 6개 시뮬 domain, 실제 image obstacle avoidance | 삽입 보상과 안전 복구 목표를 별개로 유지 |
| Failure Prediction with Statistical Guarantees (RSS 2022) | 센서 관측 이력, 고정 black-box policy | PAC-Bayes 실패 예측기, class별 error bound | 드론 시각 제어 + 손목 RGB-D 로봇 파지 | frame 정확도보다 episode false negative와 경고 선행시간 평가 |

## 1. REFLECT

- 문제: 실패 여부만으로는 다음 수정 행동을 결정하기 어렵다.
- 방법: 과거 다중 센서 경험을 계층적으로 요약하고, 실패 원인 설명을 수정 계획에 사용한다. RoboFail로 실행 실패와 계획 실패를 평가한다.
- 우리 적용(추론): `requested_pin`, `observed_pin`, 정렬 오차, 놓기 시점과 결과를 사건으로 저장한다. 먼저 기하/접촉 관측이 신뢰 가능해야 하며, LLM 설명 자체를 충돌 없는 운동 명령으로 간주하면 안 된다.
- 한계: 의미적 설명/계획 연구이며 접시 림의 mm 정렬 정밀도를 입증한 논문이 아니다.
- 출처: [공식 논문 PDF](https://proceedings.mlr.press/v229/liu23g/liu23g.pdf).

## 2. AHA

- 문제: 일반 VLM은 조작 실패를 인식하고 이유를 설명하는 데 약하다.
- 방법: 성공 시연 keyframe을 절차적으로 변형하는 FailGen으로 실패 trajectory와 설명 데이터를 만들고 VLM을 학습한다. 시뮬 학습으로 실제·미학습 task의 실패 reasoning을 평가하며, 다른 manipulation framework의 복구에 연결한다.
- 우리 적용(추론): 슬롯 한 칸 오류, 접시 기울기, 너무 얕은 삽입, 놓은 뒤 넘어짐을 구분해 합성 데이터와 보스 라벨을 만든다. 접촉 이상은 화면만으로 보이지 않을 수 있어 힘/실측 정보도 필요하다.
- 한계: 자연어 설명의 우수성이 핀 번호·깊이의 정량적 정확도와 같지 않다. 실패 영상만 만드는 것과 유효한 물리 실패 모델을 만드는 것도 구분해야 한다.
- 출처: [논문](https://arxiv.org/abs/2410.00371), [저자 프로젝트](https://aha-vlm.github.io/), [공식 코드](https://github.com/NVlabs/AHA).

## 3. Sentinel

- 문제: imitation policy는 학습 분포 밖에서 불규칙하게 움직이거나, 일관되게 움직이면서도 task를 진행하지 못한다.
- 방법: action chunk의 시간적 일관성으로 빠른 erratic failure를 감시하고, VLM 영상 질의로 느린 task-progress failure를 감시한다. 두 detector의 보완성을 시뮬과 실기에서 평가한다.
- 우리 적용(추론): 빠른 루프는 접촉·속도·궤적 불연속, 느린 루프는 요청 번호와 관측 번호 및 삽입 높이의 정체를 확인한다. 현재 기하 spline에는 생성 정책의 action-distribution 검사를 그대로 적용하기보다 위치/진행 지표를 쓴다.
- 한계: diffusion action 분포를 전제로 한 부분을 현재 기하 계획기에 그대로 복사할 수 없다. VLM 지연을 빠른 접촉 정지에 사용하지 않는다.
- 출처: [논문 본문](https://arxiv.org/html/2410.04640v2), [공식 코드](https://github.com/agiachris/sentinel).

## 4. FINO-Net

- 문제: 조작 중 충돌뿐 아니라 놓은 뒤 전도·쏟아짐 같은 실패도 감지·분류해야 한다.
- 방법: RGB-D와 audio의 시공간 특징을 CNN/ConvLSTM으로 처리해 실패 detection과 type classification을 수행한다. 논문은 detection F1 0.87, classification F1 0.80을 보고한다.
- 우리 적용(추론): 넣기 전/접촉 중/그리퍼 개방 후를 포함한 손목 영상 clip을 저장한다. 마이크가 없는 현재 장비에서는 vision+force variant를 별도 검증해야 하며 논문 성능을 그대로 기대하지 않는다.
- 한계: 데이터의 task·센서 구성이 다르며 잘못된 핀 번호를 판별하는 실험은 아니다. 깊이를 못 읽는 근거리에서 RGB-D 모델을 그대로 적용할 수 없다.
- 출처: [원문 PDF](https://arxiv.org/pdf/2305.04639).

## 5. TRANSIC

- 문제: 접촉·마찰·기하 등 다양한 sim2real 오차를 사전 모델링만으로 해결하기 어렵다.
- 방법: 시뮬 정책을 실제 실행하면서 인간이 intervention/correction을 제공하고, 이를 residual policy로 학습해 결합한다. 복잡한 접촉 조작에서 사람이 제공한 수정 데이터의 효과를 평가한다.
- 우리 적용(추론): 보스의 안전한 정지 자세에서 `수정 전 상태 → 수정량 → 수정 후 성공/실패`를 기록한다. 고정 base를 유지해 수정 정책 때문에 기존 성공 동작이 사라지는지 비교한다.
- 한계: 이것은 실패 detector만의 연구가 아니다. 인간 correction이 있고 실제 정책·센서 인터페이스가 검증되어야 한다.
- 출처: [원문 본문](https://arxiv.org/html/2405.10315v2), [저자 프로젝트](https://transic-robot.github.io/).

## 6. Recovery RL

- 문제: task 보상만 최적화하면 안전을 위반할 수 있다.
- 방법: task 성능과 제약 만족을 두 정책으로 분리한다. offline constraint 데이터로 위험을 학습하고 위험 상태에서 recovery policy를 사용한다.
- 우리 적용(추론): 정렬 실패 때의 수직 후퇴·재관측을 먼저 검증된 동작으로 준비하고, 이후에만 recovery를 학습한다. 끼인 접시의 자동 재시도는 힘 한계와 빠져나오는 방향이 검증되기 전 실시하지 않는다.
- 한계: 위험 critic의 정확성이 중요하고, 이 방법 자체가 안전 보장이나 our rack 실기 검증을 대체하지 않는다.
- 출처: [원문 PDF](https://arxiv.org/pdf/2010.15920), [공식 코드](https://github.com/abalakrishna123/recovery-rl).

## 7. 손목 영상 실패 예측의 통계적 평가

- 문제: 실패 detector도 false positive(불필요한 정지)와 false negative(실패를 놓침)를 낸다.
- 방법: 고정 정책을 black box로 두고 센서 이력에서 실패를 예측하며, PAC-Bayes와 class별 오류 bound로 일반화를 분석한다. 손목 RGB-D 파지와 드론 실험을 포함한다.
- 우리 적용(추론): 프레임을 무작위로 섞지 않고 episode 단위로 나누며, 바구니 pose/조명/가림의 다른 조건을 독립 test로 둔다. 실패를 얼마나 일찍 알렸는지와 놓친 실패를 우선 보고한다.
- 한계: bound의 분포·표본 가정을 확인해야 한다. 임의의 sim2real 변화나 모든 접촉 사고를 보장하는 결과는 아니다.
- 출처: [RSS 공식 원문](https://www.roboticsproceedings.org/rss18/p042.pdf).

## 우리 실험 제안 (10/22 이후, 구현 전 검토용)

- 실패 라벨: 정상 / 잘못된 분면 / 잘못된 핀 번호 / 핀 미정렬 / 접촉 정체 / 미안착 / 놓은 뒤 전도 / 관측 불가. 관측 불가를 정상으로 학습시키지 않는다.
- 입력: 손목 RGB의 짧은 이력 + 요청 분면·핀 ID + 핀 검출 confidence·정렬 오차 + 유효한 측정 시점의 외력/로봇 상태. 고정 카메라는 놓은 뒤 지지·전도를 확인하는 보조 시점으로 사용한다.
- 첫 baseline: 학습 없는 기하 번호 검사 + 정렬/가림 reject + 진행 정체 검사. 이후 작은 영상 시계열 classifier, 마지막에 VLM 실패 설명을 비교한다.
- 데이터: 현재 저장 실행을 episode별로 라벨링하고, 인접 슬롯·가림·기울기 perturbation을 sim에 추가한다. 실기 실패 데이터는 보스가 승인한 안전한 관측/정지 조건에서만 확보한다. 기존 성공 시연만으로 classifier를 평가하지 않는다.
- 평가: detection precision/recall, episode false-negative rate, 잘못된 번호 confusion matrix, 경고 lead time, false stop/hour, 재시도 후 성공률·횟수·시간. 학습/검증/시험은 episode와 바구니 pose 단위로 분리한다.
- 복구: 먼저 정지→검증된 후퇴→재관측→다시 계획의 상태 전이를 만들고 각 전이를 검증한다. 자연어 설명과 detector confidence만으로 위험한 재시도를 허용하지 않는다.
