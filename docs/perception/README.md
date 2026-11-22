# Perception Benchmark & Human-in-the-Loop GT Guide

`person_tracker`의 모델(`v8n/11n/26n × Det/Seg/Pose`)·추적 알고리즘(`1~5`)·3D LiDAR Localizer 오프라인 벤치마크 결과와, 노트북에서 이어서 작업하기 위한 Human-in-the-Loop GT 어노테이션·실행 가이드입니다.

> **개인정보 / 용량 원칙**: 실제 카메라 프레임(스냅샷 PNG), rosbag, `.pt` 가중치, `.pptx`/`.drawio`는 Git에 올리지 않고 정량 요약(`summary.md`, `summary_*.csv`), 성능 차트(`charts/*.png`), BBox 좌표 기반 GT CSV(`gt/*.csv`)만 관리합니다.

---

## 1. 폴더 구성

```text
docs/perception/
├── README.md                  # 이 문서 (노트북 이어하기 · 사용법 · 결과 요약)
├── gt/
│   ├── pose_estimate2_gt.csv  # Human-in-the-Loop 클릭 교정 GT (2,403f 중 2,300f 라벨링)
│   ├── one_man_gt.csv         # 단일 인물 이탈·재등장 GT (1,870f 전체 라벨링)
│   ├── model_benchmark.csv    # model_benchmark 타겟 0 GT (2,184f 전체 라벨링)
│   └── model_benchmark1~5.csv # model_benchmark 추가 5명 개별 타겟 GT (2,184f 중 1,565~1,960f)
└── results/
    ├── pose_estimate2_gt/     # GT 적용 정량 평가 결과 (all9 + 5모드 + LiDAR)
    ├── one_man_gt/            # [NEW] one_man GT 정량 평가 결과 (all9 + 5모드, RTX 4070)
    ├── model_benchmark_gt/    # [NEW] model_benchmark 타겟 0 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark1_gt/   # [NEW] model_benchmark 타겟 1 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark2_gt/   # [NEW] model_benchmark 타겟 2 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark3_gt/   # [NEW] model_benchmark 타겟 3 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark4_gt/   # [NEW] model_benchmark 타겟 4 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark5_gt/   # [NEW] model_benchmark 타겟 5 GT 정량 평가 결과 (all9 + 5모드)
    ├── model_benchmark/       # 카메라 모델 9종 연산 분해 · 전경 분리 비교 (2,184f, auto-lock)
    ├── pose_estimate2/        # 다인 교차 혼잡 시나리오 (2,403f, auto-lock 기준)
    ├── one_man/               # 단일 인물 이탈·재등장 시나리오 (1,870f, auto-lock)
    ├── full/                  # /robot1 카메라+LiDAR+TF 통합 주행 (1,290f)
    ├── test_full/             # /robot1 통합 주행 (1,522f)
    ├── robot1_full/           # /robot1 근거리·가림 주행 (862f)
    └── follow_with_cam/       # 카메라 단독 추종 (1,786f)
```

---

## 2. 방금 완료한 `pose_estimate2` GT 라벨링 · 벤치마크 결과 (`results/pose_estimate2_gt/`)

### A. GT 라벨링 요약 (`gt/pose_estimate2_gt.csv`)
- **평가 Bag**: `/robot4/top_cam/image_raw/compressed` (`2,403` frames)
- **라벨 분포**: 총 `2,403f` 중 `2,300f` 라벨링 완료 (`f1~f103` `103f`는 타겟 등장 전 `unlabeled`)
  - `gt_present=1`: **`1,996f`** (사람 클릭 교정 `35회`, 보조 트래커 `26n-seg` 자동 전파 `1,961f`)
  - `gt_present=0`: **`304f`** (`N`/우클릭 부재 지정 `194f` + 보조 트래커 미검출 `110f`)

### B. 기존 휴리스틱(`sim>=0.40`) 대비 실제 GT(`IoU>=0.5`) 평가에서 드러난 핵심 결과

기존 자동 잠금 실행([pose_estimate2](results/pose_estimate2/summary.md))에서는 프레임 1의 작업자를 자동 잠금해 `26n-pose` `True Lock`이 `41.7%`로 낮게 나왔으나, 실제 타겟이 등장하는 `f104`를 클릭으로 지정한 **실제 GT 기준([pose_estimate2_gt](results/pose_estimate2_gt/summary.md))**에서는 전혀 다른 실제 변별력이 확인되었습니다.

#### 1) 추적 알고리즘 비교 (`yolo26n-pose`, `1,996` 양성 / `304` 음성 프레임)
| Mode | GT True Lock % | GT Hijack % | GT Miss % | GT False Lock % | Mean IoU | Direct Sw | Recovered (Avg f) | Total (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `1:ByteTrack` | 10.1% | 0.0% | 89.9% | 6.9% | 0.964 | 0 | 0 | 6.62 |
| `3:BoT-SORT` | 10.1% | 0.0% | 89.9% | 6.9% | 0.951 | 0 | 0 | 10.68 |
| `2:Task-ReID` | 93.6% | 6.3% | 0.1% | 64.5% | 0.891 | 8 | 0 | 8.32 |
| `4:BoT+ReID` | 93.2% | 6.8% | 0.1% | 65.5% | 0.854 | 10 | 0 | 12.62 |
| **`5:Adaptive`** | **96.1%** | **3.8%** | **0.1%** | 62.8% | **0.914** | **2** | **5 (14.0f)** | **8.69** |

- **모션 전용 한계 입증**: `ByteTrack`/`BoT-SORT`는 초반 가림 이후 타겟을 완전히 놓쳐 **`GT Miss 89.9%` (`GT True Lock 10.1%`)**에 머뭅니다.
- **단순 ReID vs `5:Adaptive`**: `Task-ReID`/`BoT+ReID`는 `Direct Sw 8~10회`, `GT Hijack 6.3~6.8%`를 보였으나, `5:Adaptive`는 **`GT True Lock 96.1%` 1위**, **`GT Hijack 3.8%` 최저**, `Direct Sw 2회`, `Recovered 5회`로 가장 높은 정확도를 달성했습니다.
- **참고 (`GT False Lock 62.8%`)**: `gt_present=0`으로 표시된 이탈·가림 경계 구간(`f300~f337`, `f751~f781` 등)에서 유사 색상 타인 또는 가려진 순간에 재잠금이 걸린 비율이므로, 추후 `search_min_sim`/`reid_match_thresh` 상향 튜닝 포인트로 활용할 수 있습니다.

#### 2) 모델별 비교 (`5:Adaptive`, `IoU >= 0.5`)
| Model | Infer (ms) | ReID (ms) | Total (ms) | Max FPS | Mask FG % | GT True Lock % | GT Hijack % | Fail Episodes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `v8n-pose` | 1.88 | 3.28 | 7.20 | 177.5 | 27.5% | 95.8% | 4.0% | 11 |
| `11n-pose` | 2.91 | 3.75 | 9.23 | 136.5 | 29.6% | **96.1%** | **3.8%** | 15 |
| `26n-det` | 2.96 | 2.76 | 8.02 | 157.2 | 38.3% | 95.5% | 4.3% | 11 |
| `26n-seg` | 3.63 | 4.29 | 10.85 | 114.1 | 46.8% | 96.0% | 4.0% | 5 |
| **`26n-pose`** | **2.69** | **3.19** | **8.30** | **156.2** | **24.2%** | **96.1%** | **3.8%** | 16 |

- `26n-pose`는 `11n-pose`와 함께 **GT True Lock `96.1%` 공동 1위**, **GT Hijack `3.8%` 최저**를 달성하면서도, `26n-seg`(`10.85ms`) 대비 **추론+ReID 총 지연시간이 23.5% 더 빠르고(`8.30ms`, `156.2 FPS`)**, `Pose` 1회 추론으로 제스처·3부위 ReID·LiDAR 몸통 섹터를 동시에 공급합니다.
  *(참고: `pose_estimate2_gt.csv`가 `26n-seg` 보조 트래커 기반으로 생성되어 `26n-seg`의 Mean IoU(`0.963`)와 `Fail Episodes(5)`가 상대적으로 유리하게 잡히는 점은 감안해야 합니다.)*

---

## 3. 내일 노트북(RTX 3060 / CPU)에서 바로 이어서 하는 순서

### Step 1. 최신 코드·결과 받기 & 빌드
```bash
cd ~/arti   # (팀 경로에 맞게)
git checkout feat/perception-benchmark
git pull origin feat/perception-benchmark

source /opt/ros/jazzy/setup.bash
source ~/venvs/arti_venv/bin/activate
colcon build --packages-select person_tracker --symlink-install
source install/setup.bash
```

### Step 2. 다른 Bag에도 Human-in-the-Loop GT 라벨링 추가하기
실제 실행 창에서 타겟이 틀리거나 놓치는 순간만 마우스로 클릭하면 CSV가 저장됩니다.
```bash
# 예: full 또는 one_man 또는 test_full
python3 src/person_tracker/research/gt_annotator.py \
  --bag ~/arti_bags/full \
  --out ~/bench_gt/full_gt.csv \
  --device cuda:0   # CPU 환경이면 --device cpu
```
- **단축키 요약**:
  - `Left-Click`: 클릭한 인물 GT 지정 + 보조 트래커 즉시 재잠금 (`click`)
  - `Left-Drag (12px↑)`: 미검출 프레임 수동 BBox 지정 (`manual_box`)
  - `Right-Click` 또는 `N`: 타겟 부재/완전 가림 (`gt_present=0`)
  - `Space`: 재생/일시정지 · `A`/`D`(화살표): 1프레임 앞/뒤 · `C`: 트래커 잠금 초기화 · `S`: 저장 · `Q`/`ESC`: 저장 후 종료
  - `--resume ~/bench_gt/...gt.csv` 옵션을 주면 기존 GT를 불러와 이어서 수정 가능

### Step 3. 노트북 실환경(RTX 3060 / CPU) 속도 + GT 정확도 벤치마크 돌리기
```bash
# GT가 있는 pose_estimate2를 3060 노트북에서 재측정 (실배포 디바이스 FPS 확인용)
python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/arti_bags/pose_estimate2 \
  --models 26n-det,26n-seg,26n-pose \
  --modes 1,2,3,4,5 \
  --device cuda:0 \
  --reid-scale \
  --gt docs/perception/gt/pose_estimate2_gt.csv \
  --gt-iou 0.5 \
  --out ~/bench_results_laptop
```
> 결과 비교 및 세부 CLI 옵션은 [`src/person_tracker/research/README.md`](../../src/person_tracker/research/README.md)에 정리되어 있습니다.

---

## 4. `one_man` · `model_benchmark` 6타겟 GT 정량 벤치마크 결과 (RTX 4070 Laptop GPU, `all9` × `5모드`)

`--models all9 --modes 1,2,3,4,5 --device cuda:0 --reid-scale --gt-iou 0.5` 조건으로 신규 7개 GT를 일괄 평가했습니다.

### A. GT 데이터셋 분포 요약
| GT CSV | 대상 Bag | 평가 프레임 (`Pos / Neg / Unlabeled`) | 타겟 등장 구간 | 수동 교정 | 결과 폴더 |
|---|---|---:|---|---:|---|
| [`one_man_gt.csv`](gt/one_man_gt.csv) | `one_man` (1,870f) | `1,450 / 420 / 0` | `f1 ~ f1870` | 2 | [`results/one_man_gt`](results/one_man_gt/summary.md) |
| [`model_benchmark.csv`](gt/model_benchmark.csv) | `model_benchmark` (2,184f) | `1,301 / 883 / 0` | `f1 ~ f2184` | 23 | [`results/model_benchmark_gt`](results/model_benchmark_gt/summary.md) |
| [`model_benchmark1.csv`](gt/model_benchmark1.csv) | `model_benchmark` (2,184f) | `712 / 1,248 / 224` | `f225 ~ f1266` | 96 | [`results/model_benchmark1_gt`](results/model_benchmark1_gt/summary.md) |
| [`model_benchmark2.csv`](gt/model_benchmark2.csv) | `model_benchmark` (2,184f) | `487 / 1,471 / 226` | `f227 ~ f1308` | 1,461 | [`results/model_benchmark2_gt`](results/model_benchmark2_gt/summary.md) |
| [`model_benchmark3.csv`](gt/model_benchmark3.csv) | `model_benchmark` (2,184f) | `783 / 1,135 / 266` | `f267 ~ f1308` | 1,137 | [`results/model_benchmark3_gt`](results/model_benchmark3_gt/summary.md) |
| [`model_benchmark4.csv`](gt/model_benchmark4.csv) | `model_benchmark` (2,184f) | `514 / 1,327 / 343` | `f344 ~ f946` | 1,334 | [`results/model_benchmark4_gt`](results/model_benchmark4_gt/summary.md) |
| [`model_benchmark5.csv`](gt/model_benchmark5.csv) | `model_benchmark` (2,184f) | `241 / 1,324 / 619` | `f725 ~ f1324` | 1,298 | [`results/model_benchmark5_gt`](results/model_benchmark5_gt/summary.md) |

### B. `one_man_gt` 핵심 결과 (`1,450` 양성 / `420` 음성)
| Mode (`yolo26n-pose`) | GT True Lock % | GT Hijack % | GT Miss % | GT False Lock % | Mean IoU | Fail Episodes |
|---|---:|---:|---:|---:|---:|---:|
| `1:ByteTrack` | 25.6% | 0.0% | 74.4% | 0.2% | 0.969 | 7 |
| `3:BoT-SORT` | 25.6% | 0.0% | 74.4% | 0.5% | 0.883 | 7 |
| `2:Task-ReID` | **99.9%** | 0.0% | 0.1% | 3.3% | 0.972 | 10 |
| `4:BoT+ReID` | **99.9%** | 0.1% | 0.0% | 4.5% | 0.890 | 10 |
| **`5:Adaptive`** | **99.9%** | **0.0%** | **0.1%** | **2.1%** | **0.972** | 8 |

- `5:Adaptive` 전 모델(`all9`) 기준: `v8n-det` `99.7%` ~ `26n-seg` **`100.0%`** (`Fail Episodes 0`), `26n-pose` **`99.9%`** (`GT Hijack 0.0%`, `Direct Sw 0`, `Recovered 5`).

### C. `model_benchmark` 6명 개별 타겟 통합 결과 (`IoU >= 0.5`, 총 `4,038` 양성 프레임)

#### 1) 모델별 6타겟 평균 정확도 (`5:Adaptive`)
| Model | Pos 가중 평균 True Lock % | Macro 평균 True Lock % | Macro False Lock % | 타겟별 GT True Lock % (`T0 / T1 / T2 / T3 / T4 / T5`) | 평균 Infer / Total (ms) |
|---|---:|---:|---:|---|---:|
| `v8n-det` | 60.9% | 56.9% | 43.1% | 98.5% / 74.6% / 3.7% / 14.8% / 55.4% / 94.6% | 4.07 / 7.89 |
| `v8n-seg` | 64.7% | 61.8% | 44.3% | 98.8% / 5.6% / 38.6% / **73.2%** / 56.0% / 98.8% | 4.44 / 9.63 |
| `v8n-pose` | 62.9% | 55.0% | **5.4%** | 97.8% / 67.0% / 0.8% / 55.6% / 33.1% / 75.5% | 4.58 / 8.80 |
| `11n-det` | 61.2% | 57.3% | 35.9% | 98.7% / 74.4% / 3.7% / 15.3% / 56.4% / 95.4% | 5.09 / 8.77 |
| **`11n-seg`** | **66.0%** | **63.5%** | 44.1% | **98.8%** / 15.0% / **51.1%** / 66.3% / 52.5% / 97.5% | 5.74 / 11.08 |
| `11n-pose` | 55.2% | 42.3% | **4.8%** | 97.7% / 68.7% / 1.0% / 18.9% / 56.2% / 11.2% | 6.23 / 10.81 |
| `26n-det` | 64.3% | 60.9% | 39.1% | 98.7% / 72.5% / 3.9% / 25.3% / **67.3%** / 97.5% | 6.37 / 10.43 |
| `26n-seg` | 56.3% | 52.2% | 42.2% | 98.7% / 4.9% / 4.7% / 55.0% / 50.8% / **99.2%** | 7.90 / 14.12 |
| **`26n-pose`** | **65.8%** | 54.4% | **11.3%** | 97.8% / **75.4%** / 3.3% / 58.4% / 56.0% / 35.3% | 6.72 / 11.85 |

#### 2) 추적 알고리즘별 6타겟 평균 (`yolo26n-pose`)
| Mode | Pos 가중 평균 True Lock % | Macro 평균 True Lock % | Macro Hijack % | Macro False Lock % | 타겟별 GT True Lock % (`T0 / T1 / T2 / T3 / T4 / T5`) |
|---|---:|---:|---:|---:|---|
| `1:ByteTrack` | 26.5% | 24.7% | **4.6%** | **0.3%** | 17.1% / 13.3% / 3.3% / 56.8% / 56.2% / 1.2% |
| `3:BoT-SORT` | 18.2% | 18.2% | 8.4% | **0.4%** | 17.1% / 13.5% / 3.7% / 10.5% / **59.7%** / 4.6% |
| `2:Task-ReID` | 69.7% | 62.0% | 32.7% | 23.9% | **98.5%** / **76.4%** / **5.5%** / **65.8%** / 52.9% / 73.0% |
| `4:BoT+ReID` | **70.1%** | **62.9%** | 26.4% | 22.0% | 98.1% / 76.1% / 5.3% / 65.6% / 57.0% / **75.1%** |
| `5:Adaptive` | 65.8% | 54.4% | 32.1% | **11.3%** | 97.8% / 75.4% / 3.3% / 58.4% / 56.0% / 35.3% |

- **주 타겟(`T0: model_benchmark_gt`)**: 모든 `5:Adaptive` 모델이 `97.7% ~ 98.8%`의 매우 높은 유지율을 보였습니다(`ByteTrack`/`BoT-SORT`는 `17.1%`).
- **Pose vs Det/Seg 오잠금 방어 트레이드오프**: `Det`/`Seg` 계열은 부재(`gt_present=0`) 구간에서 타인을 타겟으로 오잠금하는 `Macro False Lock`이 `35.9% ~ 44.3%`로 높은 반면, **`Pose` 계열(`v8n-pose 5.4%`, `11n-pose 4.8%`, `26n-pose 11.3%`)은 오잠금을 1/3~1/8 수준으로 억제**합니다. 다만 `T2`/`T5`처럼 원거리·부분 가림으로 Pose 관절이 소실되는 타겟에서는 `Adaptive`의 보수적 임계치로 인해 `Task-ReID`/`BoT+ReID` 대비 재잠금률이 낮아지는 경향이 확인되었습니다.

---

## 5. 아직 GT 라벨링 미진행 Bag 목록 (후속 작업)
- `~/Downloads/arti_benchmark_bags/complete_bag/robot1_full`
- `~/Downloads/arti_benchmark_bags/complete_bag/test_full`
- `~/Downloads/arti_benchmark_bags/follow_with_cam`
- `~/Downloads/arti_benchmark_bags/follow_with_cam0`
- `~/Downloads/bag3/pose_estimate`
- `~/Downloads/bag3/robot4/full`
