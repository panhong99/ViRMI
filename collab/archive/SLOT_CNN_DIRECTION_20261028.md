# 슬롯 CNN 방향 판단 (2026-10-28, CK)

## 결론

**추가 epoch 학습은 중지한다.** 선택 모델 `run_v10_cross/ep02.pt`는 기존 라벨 4장에서 tip 84/84, 영역 안 precision 0.933, Q1 14/14, 번호 오답 0이지만, 마커 없는 교차 시야 10장에서는 strict 번호판정이 7/10이다. 실패 3장은 모두 **Q4 카메라 시야 → Q3 슬롯**이며 tip 수는 26/26이므로, 주된 병목은 tip 누락이 아니라 바구니 끝 프레임을 14번째 tip으로 보는 **끝단 구분과 번호 결정**이다.

11/3 시연의 `demo1103`와 `demo1103_fixed`는 이 CNN에 의존하지 않는다. 둘 다 `--no_wrist_view`로 실행하며, `pipeline.py`가 바구니 모델의 `region_pins()` 후보와 `plate_plans.pin_gap()`을 사용해 핀을 고른 뒤 `quadrant_plan()`을 계산한다. 두 명령 모두 `--insert_policy`나 `experiments/slot_cnn`을 호출하지 않는다. 따라서 CNN 개선을 시연 선행 조건으로 두지 않는다.

## 실패 3/10의 분해

| 원인 | 근거 | 판단 |
|---|---|---|
| (a) 독립 실사진 라벨 부족 | 완전 수동 라벨은 반복 평가한 4장뿐이다. 새 마커 없는 19장은 손 라벨이 없어 259/260을 정확도라고 부를 수 없다. 끝단 false tip 수동 라벨도 3장 4점뿐이다. | **일반화 판정과 실사진 미세학습에는 부족** |
| (b) 시점·가림·기하 | Q3 시야→Q4는 5/5 strict. Q4 시야→Q3는 2/5 strict이고 실패 3장도 26/26 검출, whole row 6/6, rows-agree 3/3이다. 다만 14개 주기 후보 중 어느 끝이 프레임인지 정할 수 없어 ambiguous row가 각각 1, 2, 1개 발생한다. | **현재 직접 원인** |
| (c) 모델 출력 한계 | ResNet18은 root/tip heatmap과 sub-pixel offset만 출력한다. 후보가 실제 빗살 끝인지 바구니 프레임 교차점인지, 행의 양 끝 중 어느 것을 버릴지는 학습 목표에 없다. | **구조적 한계** |

추가 실사진 3장을 섞은 `run_v11_realhn`은 끝단 오검출 평균 probability를 0.640→0.537로 낮췄지만 strict가 7/10→6/10으로 감소했다. 출력 보존 distillation도 6/10이었다. threshold 0.5는 strict 7/10을 회복했지만 tip 79/84, Q1 9/14로 재현율을 잃었다. 두 후보 모두 배포하지 않고 ep02를 유지한다. 단순한 두 행 기하 합의도 v4 pose perturbation 100회에서 번호 오답을 만들었으므로 사용하지 않는다.

## 다음 수단

1. **11/3 시연:** CNN 없이 현재 `demo1103`/`demo1103_fixed` 경로를 사용한다. 슬롯은 바구니 모델 좌표와 지정된 front/back 영역으로 정한다.
2. **연구용 CNN 재개 조건:** 새 촬영을 지금 요청하지 않는다. 재개할 때는 마커 없는 독립 손목 사진 **최소 24장**을 먼저 완전 라벨한다: Q4 시야→Q3 16장, Q3 시야→Q4 대조 8장. 거리·yaw·가림을 나누고 모든 tip 및 끝 프레임 false 후보를 표시한다. 이는 필요한 false endpoint를 수십 개 확보하기 위한 설계 수량이며, 정확도 보장 수량은 아니다.
3. **모델 변경:** 기존 heatmap을 연장 학습하지 말고, 행 전체 crop을 입력으로 받아 `near end / far end / valid tine`을 판정하는 endpoint head 또는 프레임/빗살 segmentation을 추가한다. 평가는 사진 단위 분리로 하고, tip recall·precision 외에 `strict 번호판정`, `번호 오답`, `거부율`을 함께 본다.
4. **기하 후처리:** 모델이 endpoint 판정을 주지 못할 때는 ambiguous를 유지한다. coarse pose에 가까운 쪽을 임의 선택해 한 칸 밀린 번호를 내는 것보다 거부가 안전하다.

## 근거 경로

- 선택 모델·곡선: `outputs/slot_cnn_run_v10_cross_report/summary.json`
- 교차 시야 장면별 결과: `outputs/slot_cnn_cross_view_run_v10_ep02/summary.json`
- hard-negative 비교: `outputs/slot_cnn_real_hardneg_20261027/`
- 기존 라벨 평가: `calibration/data/slot_cnn/eval_run_v10_cross_ep02/metrics.json`
- 시연 진입점: `calibration/run.sh`의 `demo1103`, `demo1103_fixed`
- 핀 선택·계획: `calibration/pipeline.py:830`, `calibration/pipeline.py:1265`, `calibration/plate_plans.py:1718`
