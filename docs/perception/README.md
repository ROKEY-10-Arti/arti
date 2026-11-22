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

## 4. `one_man` · `model_benchmark` 6타겟 GT 정량 벤치마크 결과 (`baseline` 기준, RTX 4070 Laptop GPU)

> 아래 표는 `baseline` (`8207418`) 기준 수치이며, `9cc391a` (`Pose edges+max`, `Seg geom_fast`) 적용 후 최신 결과 파일은 `results/*_gt/`에 반영되어 있고 전·후 비교는 **7절**에 정리되어 있습니다.

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

---

## 6. Pose 마스킹(`edges`) · Seg(`geom_fast`) 개선안 랩탑 GT 검증 실행

근접 추종·부분 가림 개선을 위해 `feature_extractors.py` / `multi_tracker_core.py`에 아래 2가지 기본값이 반영되었습니다.
- **Pose (`--pose-mode edges --edge-score-mode max`)**: 상완(`5-7`, `6-8`)·측면 자세·단측 어깨/팔꿈치 근접 상체 마스크를 보강해 BBox 폴백을 줄이고, 마커 연결선별 1:1 유사도를 `max`로 결합
- **Seg (`--seg-mode geom_fast`)**: `128px` ROI 다운스케일로 ReID 지연을 ~40% 단축하고, 화면 상/하단 잘림·종횡비 기반 해부학 분할로 근접 시 상의(`upper`) 오분류 감소

랩탑(GPU)에서 최신 코드를 받아 바로 GT A/B 검증을 돌리는 명령어입니다.

```bash
# 1) 최신 코드 반영 & 빌드
cd ~/arti
git checkout feat/perception-benchmark && git pull origin feat/perception-benchmark
source /opt/ros/jazzy/setup.bash && source ~/venvs/arti_venv/bin/activate
colcon build --packages-select person_tracker --symlink-install
source install/setup.bash

# 2) 개선 기본값(Pose edges + Seg geom_fast) GT 벤치마크 — pose_estimate2 & one_man
python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/bag3/pose_estimate2 \
  --models 26n-det,26n-seg,26n-pose \
  --modes 1,3,5 --device cuda:0 --reid-scale --no-snapshots \
  --gt docs/perception/gt/pose_estimate2_gt.csv --gt-iou 0.5 \
  --out ~/bench_v2_pose_estimate2_gt

python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/Downloads/arti_benchmark_bags/one_man \
  --models 26n-det,26n-seg,26n-pose \
  --modes 5 --device cuda:0 --reid-scale --no-snapshots \
  --gt docs/perception/gt/one_man_gt.csv --gt-iou 0.5 \
  --out ~/bench_v2_one_man_gt

# 3) model_benchmark 6타겟(T0~T5) 개선안 일괄 GT 평가 (결과 폴더에 타겟별 summary.md 생성)
for idx in "" 1 2 3 4 5; do
  gt_name="model_benchmark${idx}.csv"
  out_name="T${idx:-0}"
  python3 src/person_tracker/research/bag_benchmark.py \
    --bags ~/Downloads/arti_benchmark_bags/model_benchmark \
    --models 26n-det,26n-seg,26n-pose \
    --modes 5 --device cuda:0 --reid-scale --no-snapshots \
    --gt "docs/perception/gt/${gt_name}" --gt-iou 0.5 \
    --out ~/bench_v2_model_benchmark/${out_name}
done

# 4) (선택) 기존 방식(baseline) vs 개선안(edges / geom_fast) 동일 GT 직접 대조
python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/Downloads/arti_benchmark_bags/model_benchmark \
  --models 26n-seg,26n-pose --modes 5 --device cuda:0 --reid-scale --no-snapshots \
  --pose-mode baseline --edge-score-mode none --seg-mode baseline \
  --gt docs/perception/gt/model_benchmark2.csv --out ~/bench_ab_baseline_T2

python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/Downloads/arti_benchmark_bags/model_benchmark \
  --models 26n-seg,26n-pose --modes 5 --device cuda:0 --reid-scale --no-snapshots \
  --pose-mode edges --edge-score-mode max --seg-mode geom_fast \
  --gt docs/perception/gt/model_benchmark2.csv --out ~/bench_ab_improved_T2
```

---

## 7. `baseline` vs 개선안(`Pose edges+max` · `Seg geom_fast`, `9cc391a`) 전 모델·전 GT 일괄 비교 결과

8개 GT 전체(`pose_estimate2_gt`, `one_man_gt`, `model_benchmark` `T0~T5`)를 `--models all9 --modes 1,2,3,4,5 --device cuda:0 --reid-scale --gt-iou 0.5`로 재실행해 `baseline` 대비 변화를 전수 비교했으며, 최신 CSV·차트는 [`results/`](results) 아래 `*_gt/` 폴더에 반영되어 있습니다.

### A. 전체 8-GT 통합 비교 (`5:Adaptive`, `IoU >= 0.5`)
| Model | 8-GT Pos 가중 `True Lock %` (`Base → V2`) | 8-GT Macro `True Lock %` (`Base → V2`) | 8-GT Macro `False Lock %` (`Base → V2`) | `Pose Fallback %` (`Base → V2`) |
|---|---:|---:|---:|---:|
| `v8n-det` | 77.6% → 77.6% (`+0.0`) | 67.1% → 67.1% (`+0.0`) | 40.4% → 40.4% | 0.0% → 0.0% |
| `v8n-seg` | 79.9% → 74.2% (`-5.6`) | 70.9% → 57.5% (`-13.3`) | 41.4% → 39.6% | 0.0% → 0.0% |
| **`v8n-pose`** | 78.9% → **83.3%** (**`+4.4`**) | 65.7% → **72.7%** (**`+7.0`**) | **12.3% → 12.3%** | **21.4% → 3.5%** |
| `11n-det` | 77.7% → 77.7% (`+0.0`) | 67.3% → 67.3% (`+0.0`) | 34.7% → 34.7% | 0.0% → 0.0% |
| `11n-seg` | 80.5% → 75.4% (`-5.0`) | 72.1% → 59.8% (`-12.3`) | 41.2% → 39.7% | 0.0% → 0.0% |
| `11n-pose` | 74.8% → 74.7% (`-0.1`) | 56.2% → 57.1% (`+0.9`) | 11.8% → 15.1% | **18.3% → 2.4%** |
| `26n-det` | 79.4% → 79.5% (`+0.0`) | 70.0% → 70.0% (`+0.0`) | 36.6% → 36.6% | 0.0% → 0.0% |
| `26n-seg` | 75.3% → 73.3% (`-2.0`) | 63.7% → 57.0% (`-6.7`) | 39.2% → 39.2% | 0.0% → 0.0% |
| `26n-pose` | 80.5% → 79.4% (`-1.0`) | 65.3% → 64.4% (`-0.9`) | 16.6% → 16.3% | **17.0% → 2.5%** |

### B. `model_benchmark` 6타겟(`T0~T5`) 모델별 전·후 비교 (`5:Adaptive`)
| Model | Pos 가중 `True Lock %` (`Base → V2`) | Macro `False Lock %` (`Base → V2`) | 6타겟 평균 ReID (ms) (`Base → V2`) | V2 타겟별 `GT True Lock %` (`T0 / T1 / T2 / T3 / T4 / T5`) |
|---|---:|---:|---:|---|
| `v8n-det` | 60.9% → 60.9% (`+0.0`) | 43.1% → 43.1% | 1.28 → 1.21 | 98.5% / 74.6% / 3.7% / 14.8% / 55.4% / 94.6% |
| `v8n-seg` | 64.7% → 54.2% (`-10.5`) | 44.3% → 41.9% | 2.28 → **1.50** | 98.7% / 5.5% / 4.7% / 73.4% / 26.3% / 55.6% |
| **`v8n-pose`** | 62.9% → **71.3%** (**`+8.4`**) | **5.4% → 5.4%** | 1.54 → 2.01 | 97.5% / **89.6%** / 3.5% / **79.9%** / 19.1% / **96.7%** |
| `11n-det` | 61.2% → 61.2% (`+0.0`) | 35.9% → 35.9% | 1.21 → 1.15 | 98.7% / 74.4% / 3.7% / 15.3% / 56.4% / 95.4% |
| `11n-seg` | 66.0% → 56.7% (`-9.3`) | 44.1% → 42.0% | 2.36 → **1.77** | 98.7% / 15.0% / 4.7% / 65.6% / 44.7% / 54.4% |
| `11n-pose` | 55.2% → 55.2% (`-0.0`) | 4.8% → 9.1% | 1.72 → 2.33 | 97.4% / **90.4%** / 4.7% / 2.9% / 41.4% / 24.5% |
| `26n-det` | 64.3% → 64.3% (`+0.0`) | 39.1% → 39.1% | 1.29 → 1.20 | 98.7% / 72.5% / 3.9% / 25.3% / **67.3%** / **97.5%** |
| `26n-seg` | 56.3% → 52.5% (`-3.7`) | 42.2% → 42.1% | 2.60 → **1.68** | 98.7% / 5.5% / 4.9% / 56.7% / 38.1% / 56.0% |
| `26n-pose` | 65.8% → 63.8% (`-1.9`) | 11.3% → 10.8% | 1.61 → 2.25 | 97.6% / 76.4% / **27.1%** / 39.3% / 49.2% / 29.5% |

### C. `model_benchmark` 6타겟 모드별 전·후 비교 (`yolo26n-pose`)
| Mode | Pos 가중 `True Lock %` (`Base → V2`) | Macro `True Lock %` (`Base → V2`) | Macro `False Lock %` (`Base → V2`) | V2 타겟별 `GT True Lock %` (`T0 / T1 / T2 / T3 / T4 / T5`) |
|---|---:|---:|---:|---|
| `1:ByteTrack` | 26.5% → 26.5% (`+0.0`) | 24.7% → 24.7% | 0.3% → 0.3% | 17.1% / 13.3% / 3.3% / 56.8% / 56.2% / 1.2% |
| `3:BoT-SORT` | 18.2% → 18.2% (`+0.0`) | 18.2% → 18.2% | 0.4% → 0.4% | 17.1% / 13.5% / 3.7% / 10.5% / 59.7% / 4.6% |
| **`2:Task-ReID`** | 69.7% → **73.9%** (**`+4.2`**) | 62.0% → **67.4%** (**`+5.4`**) | 23.9% → **20.4%** | **97.8%** / 89.9% / 23.2% / **67.2%** / 47.5% / 78.8% |
| **`4:BoT+ReID`** | 70.1% → **73.6%** (**`+3.5`**) | 62.9% → **67.2%** (**`+4.4`**) | 22.0% → 30.8% | **97.8%** / **90.3%** / 23.6% / 65.0% / 47.1% / **79.7%** |
| `5:Adaptive` | 65.8% → 63.8% (`-1.9`) | 54.4% → 53.2% | **11.3% → 10.8%** | 97.6% / 76.4% / **27.1%** / 39.3% / 49.2% / 29.5% |

### D. `pose_estimate2_gt` · `one_man_gt` 전·후 비교
- **`pose_estimate2_gt`** ([summary.md](results/pose_estimate2_gt/summary.md)): `26n-pose + 5:Adaptive` `GT True Lock 96.1% → 96.1%` 유지 (`Fail Episodes 16 → 14`), `v8n-seg` `96.0% → 96.0%`, `11n-seg` `95.8% → 95.8%`, `26n-seg` `96.0% → 96.0%`. 단, `Seg geom_fast`는 근접/화면하단 잘림 판정으로 `M/L Valid %`가 `100% → 4.3%~12.5%`로 감소했습니다.
- **`one_man_gt`** ([summary.md](results/one_man_gt/summary.md)): `all9` 전 모델이 `99.3% ~ 100.0%` 그대로 유지되면서, Pose 3종의 `Fallback %`가 `10.1%~19.2% → 1.1%~2.1%`로 크게 감소했고 Seg 3종의 `ReID` 지연시간은 `1.20~1.30ms → 0.71~0.80ms`로 약 `38~45%` 단축되었습니다.

### E. 핵심 인사이트 및 최종 채택 결정
1. **Pose `edges` 효과 확인**: BBox 폴백률이 8-GT 평균 `17.0%~21.4% → 2.4%~3.5%`로 급감했고, **`v8n-pose`는 8-GT 전체 1위(`83.3%`, `+4.4%p`), `model_benchmark` 6타겟 기준 `+8.4%p` 상승**을 기록했습니다. `26n-pose`에서도 `Task-ReID`(`+4.2%p`), `BoT+ReID`(`+3.5%p`), `T2`(`3.3% → 27.1%`)가 크게 개선되었습니다.
2. **Seg `geom_fast` 트레이드오프 → `baseline` 기본값 복원**: `ReID` 연산 시간은 `~25~35%` 단축되었으나, 화면 경계 잘림(`clipped_bottom`/`clipped_top`) 및 `128px` 다운스케일 조건이 원거리·부분 가림 타겟(`T2`, `T4`, `T5`)의 `mid/lower` 유효율과 재식별 일관성을 낮춰 `Seg` 정확도가 `-3.7%~-10.5%p` 하락했습니다. 이에 **`Seg` 기본값은 `baseline`으로 복원**했습니다(`--seg-mode geom_fast` 옵션은 유지).
3. **최종 기본 모델 변경 (`yolo26n-pose` → `yolov8n-pose`)**: 8종 GT 종합 정확도(`83.3%` 1위), 다수 타겟(`71.3%`, `False Lock 5.4%`), GPU 추론 속도 모두에서 우위를 확인해 기본 `tracker_family_idx=1`(`yolov8n-pose.pt`, `pose_mode='edges'`, `edge_score_mode='max'`)으로 전환했습니다.
