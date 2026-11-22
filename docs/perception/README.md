# Perception Benchmark & Human-in-the-Loop GT Guide

`person_tracker`의 모델(`v8n/11n/26n × Det/Seg/Pose`)·추적 알고리즘(`1~5`)·3D LiDAR Localizer 오프라인 벤치마크 결과와, 노트북에서 이어서 작업하기 위한 Human-in-the-Loop GT 어노테이션·실행 가이드입니다.

> **개인정보 / 용량 원칙**: 실제 카메라 프레임(스냅샷 PNG), rosbag, `.pt` 가중치, `.pptx`/`.drawio`는 Git에 올리지 않고 정량 요약(`summary.md`, `summary_*.csv`), 성능 차트(`charts/*.png`), BBox 좌표 기반 GT CSV(`gt/*.csv`)만 관리합니다.

---

## 1. 폴더 구성

```text
docs/perception/
├── README.md                  # 이 문서 (노트북 이어하기 · 사용법 · 결과 요약)
├── gt/
│   └── pose_estimate2_gt.csv  # Human-in-the-Loop 클릭 교정 GT (2,403f 중 2,300f 라벨링)
└── results/
    ├── pose_estimate2_gt/     # [NEW] GT 적용 정량 평가 결과 (all9 + 5모드 + LiDAR)
    ├── model_benchmark/       # 카메라 모델 9종 연산 분해 · 전경 분리 비교 (2,184f)
    ├── pose_estimate2/        # 다인 교차 혼잡 시나리오 (2,403f, auto-lock 기준)
    ├── one_man/               # 단일 인물 이탈·재등장 시나리오 (1,870f)
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
