# 손목 핀·고정 카메라 마커 결과 (Codex, 2026-10-13)

## A: 오프라인 프로토타입

저장 사진 34장 중 33장 처리, 1장은 저장 K가 없어 제외. 검출 기준 통과 쌍 7개(7장)이며 수작업 정답이 없으므로 정확도·recall로 해석하지 않는다. 철망 구멍·접시/마커 가림·불연속 선·모호한 대응을 거부한다. 기하 투영 오차와 횡방향 잔차가 남아 **자동 보정이나 실기 투입은 미승인**이다.

분면은 계획기의 바구니 좌표 기준과 동일: +x 먼 쪽, +y 왼쪽; Q1 먼/오른쪽, Q2 먼/왼쪽, Q3 가까운/왼쪽, Q4 가까운/오른쪽. 좌우 두 열의 동일 x 핀을 한 쌍으로 묶는다. 각 분면에서 화면 위→아래 번호를 주며 미확정은 `?`. 화면 자동 회전(이번 사진 90°)은 JSON에 기록하며 `--rotate 0`으로 원본 방향 출력 가능. 사진별 번호와 삽입 gap 번호는 다른 개념이므로 연결 전에 별도 매핑 검증이 필요하다.

보정량은 바구니 +x 방향의 모델–영상 피팅 값이다. 실제 3D 정답과의 mm 오차가 아니다. Hough 보조 피팅의 횡방향 픽셀 잔차도 JSON에 보존한다. 모델 중점이 화면에 있는 개수는 실제 보이는 핀 수가 아니다.

출력: `calibration/data/basket/pins_codex_verified_20261013/`의 summary.csv/json 및 사진별 pairs.png/json. 기존 사진/캘리브레이션 불변.

| 사진 폴더 / 시점 | 기준 통과 쌍 | 보정량 중앙값 mm | 비고 |
|---|---:|---:|---|
| 20261012_133446_plate_vdpni1fm / wrist_above | 1 | +3.000 |  |
| 20261012_133446_plate_vdpni1fm / wrist_pre | 0 | — |  |
| 20261012_135137_plate_ab74yk62 / wrist_above | 0 | — |  |
| 20261012_135137_plate_ab74yk62 / wrist_pre | 0 | — |  |
| 20261012_140945_plate_kt4f65jt / wrist_above | 0 | — |  |
| 20261012_140945_plate_kt4f65jt / wrist_pre | 0 | — |  |
| 20261012_141623_plate_68z27cg9 / wrist_above | 0 | — |  |
| 20261012_141623_plate_68z27cg9 / wrist_pre | 0 | — |  |
| 20261012_143246_plate_cmdy2pgf / wrist_above | 0 | — |  |
| 20261012_143246_plate_cmdy2pgf / wrist_pre | 0 | — |  |
| 20261012_144358_plate_eoxz7tqb / wrist_above | 0 | — |  |
| 20261012_144358_plate_eoxz7tqb / wrist_pre | 0 | — |  |
| 20261012_145522_plate__c5aos6x / wrist_above | 1 | +3.117 |  |
| 20261012_145522_plate__c5aos6x / wrist_pre | 0 | — |  |
| 20261012_150405_plate_ozt_ayxg / wrist_above | — | — | Saved camera K required; do not infer intrinsics from image size |
| 20261012_150405_plate_ozt_ayxg / wrist_pre | 0 | — |  |
| 20261012_151641_plate_ej2zobtt / wrist_above | 1 | +5.611 |  |
| 20261012_151641_plate_ej2zobtt / wrist_pre | 0 | — |  |
| 20261012_161612_plate_5tnwpbs2 / wrist_above | 0 | — |  |
| 20261012_161612_plate_5tnwpbs2 / wrist_pre | 0 | — |  |
| 20261012_162838_plate_a7fgbuz1 / wrist_above | 1 | -3.247 |  |
| 20261012_162838_plate_a7fgbuz1 / wrist_pre | 0 | — |  |
| 20261013_132344_plate_xg0lstv_ / wrist_above | 0 | — |  |
| 20261013_132344_plate_xg0lstv_ / wrist_pre | 0 | — |  |
| 20261013_135535_plate_4j9f7lo8 / wrist_above | 1 | +1.406 |  |
| 20261013_135535_plate_4j9f7lo8 / wrist_pre | 0 | — |  |
| 20261013_164915_plate_d_2krw2l / wrist_above | 1 | -5.457 |  |
| 20261013_164915_plate_d_2krw2l / wrist_pre | 0 | — |  |
| 20261013_165813_plate_niupvylr / wrist_above | 0 | — |  |
| 20261013_165813_plate_niupvylr / wrist_pre | 0 | — |  |
| 20261013_170727_plate_fla_xm3m / wrist_above | 0 | — |  |
| 20261013_170727_plate_fla_xm3m / wrist_pre | 0 | — |  |
| 20261013_171458_plate_wl96m6l2 / wrist_above | 1 | -2.710 |  |
| 20261013_171458_plate_wl96m6l2 / wrist_pre | 0 | — |  |

## B: 10 cm급 기준값

고정 D455, 실제 설정 흑색 정사각형 98 mm, 10회 × 20프레임. 4마커 동시 인식률 100%, 평균 피팅 잔차 7.326 mm, 최대 7.819 mm. 아래는 200프레임 위치 표준편차(mm)이며 절대 정확도는 아니다. 반복별 중앙값 표준편차가 0에 가까운 경우에도 픽셀 양자화·시간 상관이 있어 완벽한 정확도를 뜻하지 않는다.

| 마커 | x std mm | y std mm | z std mm |
|---|---:|---:|---:|
| 1 | 1.154 | 1.802 | 2.153 |
| 2 | 0.653 | 1.527 | 3.148 |
| 3 | 0.406 | 4.010 | 4.526 |
| 4 | 1.060 | 4.119 | 3.918 |

원본 측정·인식률·repeat 중앙값 통계: `calibration/data/basket/marker_stats_codex_baseline_20261013/{summary.json,markers.csv}`.

5 cm·2.5 cm 비교는 보스가 실제 마커를 인쇄·부착한 뒤 동일 조건으로 측정해야 한다. 기존 큰 마커에 작은 치수만 입력하는 가상 비교는 하지 않았다. `--marker_size_m`은 메모리 내 이번 실행만 변경하고 config 파일을 수정하지 않는다.

## 실행 예

```bash
conda run -n foundationpose python calibration/camera_tools.py pins --req <wrist_above.req.json> --out <새폴더>
conda run -n foundationpose python calibration/camera_tools.py basket_stats --out <새폴더> --repeats 10 --frames 20 --marker_size_m 0.05
```

두 번째 명령은 실제 5 cm 흑색 마커가 부착된 이후에만 사용한다. 카메라 읽기 전용이며 로봇·그리퍼 명령 없음.

## 검증·다음 단계

- 기존 모의 테스트 + 핀 합성 shift ±4 mm/0, 바닥 구멍 거부, 접시 가림 거부: 24/24 통과.
- 다음: 저장 사진에 수작업 핀 끝/쌍 ID 정답을 붙여 precision/recall·보정 픽셀 오차를 계산하고, 현재 모델 투영 방향의 불일치를 분석. 미검출을 억지로 성공 처리하지 않는다.
- 연구 7편은 `collab/FAILURE_DETECTION_REVIEW_2026-10-13.md`에 별도 정리.
