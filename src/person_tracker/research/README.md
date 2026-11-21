# Person Tracker — 오프라인 Bag 벤치마크 & 연구 가이드

이 디렉터리는 실시간 주행 노드를 건드리지 않고, 녹화한 **ROS 2 Bag(`.mcap` / `.db3`)** 을 이용해
**YOLO 모델 세대(`v8n/11n/26n`) × 태스크(`Det/Seg/Pose`) × 추적 알고리즘(`1~5`) × 3D LiDAR Localizer** 성능을
오프라인으로 자동 비교·시각화하기 위한 도구 공간입니다.

---

## 1. 다른 PC에서 바로 벤치마크 실행하기 (빠른 세팅)

고성능 PC(GPU/다코어 CPU)에서 GitHub 코드와 Google Drive의 bag 파일을 받아 바로 실행하는 순서입니다.

### 1) 저장소 클론 및 브랜치 체크아웃
```bash
git clone git@github.com:ROKEY-10-Arti/arti.git ~/project/arti
cd ~/project/arti
git checkout feat/perception-benchmark
```

### 2) Python venv & ROS 2 빌드
```bash
# 1) Python 의존성 설치 (프로젝트 공용 venv 스크립트 사용)
bash scripts/make_venv.sh

# 2) ROS 2 Jazzy 환경 로드 및 person_tracker 패키지 빌드
source /opt/ros/jazzy/setup.bash
colcon build --packages-select person_tracker
source install/setup.bash
```
> **필수 패키지 확인**: `matplotlib`, `opencv-python`, `ultralytics`, `PyYAML`, `rosbag2_py`가 설치되어 있어야 합니다.  
> YOLO 가중치(`.pt`)는 최초 실행 시 `ultralytics`가 자동 다운로드합니다.

### 3) Google Drive에서 받은 Bag으로 벤치마크 실행

#### 단일 Bag 빠르게 실행 (`quick`: `v8n-pose`, `11n-pose`, `26n-det`, `26n-seg`, `26n-pose`)
```bash
source /opt/ros/jazzy/setup.bash && source ~/project/arti/install/setup.bash
python3 ~/project/arti/src/person_tracker/research/bag_benchmark.py \
  --bags ~/bag/model_benchmark \
  --models quick \
  --modes 1,3,5 \
  --reid-scale \
  --out ~/bench_results
```

#### 여러 Bag 일괄 실행 (발표용 전체 9모델 × 5알고리즘, GPU 사용)
```bash
source /opt/ros/jazzy/setup.bash && source ~/project/arti/install/setup.bash
python3 ~/project/arti/src/person_tracker/research/bag_benchmark.py \
  --bags ~/bag/model_benchmark ~/bag/complete_bag/test_full ~/bag/follow_with_cam \
  --models all9 \
  --modes 1,2,3,4,5 \
  --device cuda:0 \
  --reid-scale \
  --out ~/bench_results_all9
```

> **네트워크 안전성**: `bag_benchmark.py`는 시작 시 자동으로 `ROS_DISCOVERY_SERVER` / `ROS_SUPER_CLIENT`를 해제하고
> `ROS_DOMAIN_ID=98`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 격리하므로 실제 로봇과 충돌하지 않습니다.

---

## 2. 주요 CLI 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--bags PATH [PATH ...]` | *(필수)* | 평가할 ROS 2 bag 폴더 경로 (1개 이상 공백으로 나열) |
| `--out DIR` | `./bench_results/<시각>` | 결과 저장 루트 폴더 (`<out>/<bag_name>/`에 자동 분류) |
| `--models` | `quick` | 프리셋(`quick`, `versions`, `tasks`, `all9`) 또는 콤마 목록(`26n-det,26n-seg,26n-pose`) |
| `--modes` | `1,3,5` | 비교할 추적 알고리즘 번호 (`1:ByteTrack`, `2:Task-ReID`, `3:BoT-SORT`, `4:BoT+ReID`, `5:Adaptive`) |
| `--device` | 자동 (`cuda:0`/`cpu`) | 추론 장치 지정 (`cuda:0`, `cpu` 등) |
| `--max-frames N` | `0` (전체) | 앞 N프레임만 평가 (빠른 동작 확인용) |
| `--stride K` | `1` (매 프레임) | K프레임마다 1장씩 샘플링 |
| `--lock-point x,y` | 자동 잠금 | 첫 타겟 선택 좌표 (예: 중앙 `320,240`; 비우면 첫 검출 인물 자동 잠금) |
| `--snap-frames` | 자동 선정 | Det/Seg/Pose·버전 비교 중심 프레임 번호 직접 지정 (예: `40,120,250`) |
| `--seq-delta Δ` | `10` | 전·중·후 3프레임 시퀀스 비교 간격 (`f-Δ`, `f`, `f+Δ`) |
| `--reid-scale` | 꺼짐 | `follow_test`의 REID 완화 파라미터(`ema_min_sim=0.52` 등) 적용 |
| `--no-localizer-compare` | 비교 켜짐 | LiDAR `Raw` vs `Kinematic+Map+EMA` 비교 생략 |
| `--no-snapshots` | 저장 켜짐 | 대시보드 및 비교 스냅샷 PNG 저장 생략 |
| `--gt PATH` | 비활성 | `gt_annotator.py`로 만든 정답 CSV 기반 진짜 Recall / Hijack / Miss / 개입 횟수 집계 |
| `--gt-iou THRESH` | `0.5` | `--gt` 매칭 시 정답으로 인정하는 최소 BBox IoU 임계값 |

---

## 3. 자동 생성되는 정리 리포트 & 지표 의미

실행이 끝나면 `<out>/<bag_name>/` 아래에 다음 항목들이 **5개 테마별 표 + 7종 차트 + 4종 시각 비교 이미지**로 정리됩니다.

### A. 통합 리포트 (`summary.md` & `summary_*.csv`)
`summary.md` 하나에 아래 내용이 이미지 미리보기와 함께 순서대로 정리됩니다.
- **0. 핵심 관찰 요약 (`Auto Insights`)**: 최고 FPS, 최고 진성 유지율, 최고 변별 마진, Pose 폴백률, Adaptive/LiDAR 개선치 자동 요약
- **1. 속도·후처리 분해 표**: `Pre`, `Infer Mean / P95`, `Post+Trk (NMS 포함)`, `ReID Extract`, `Total Mean / P95`, `Max FPS`
- **2. 특징 추출 & 부위 가용성 표**:
  - `U / M / L Valid %`: 실제로 상의·허벅지·정강이 특징이 추출된 비율 (근접/가림 시 Pose 관절 소실률 확인용)
  - `Pose Fallback %`: 관절 부족으로 Pose가 BBox 중앙 크롭으로 폴백한 비율
  - `Mask FG %`: BBox 면적 중 실제 전경 마스크가 차지하는 비율 (배경 포함 정도 비교)
  - `Avg Tgt Sim ± Std (Min)` 및 `U / M / L / F Avg Sim`
- **3. 재식별 변별력 · 오인식 방어 표**:
  - `True Lock %`: 전체 프레임 중 유사도 `>= 0.40`으로 실제 타겟을 유지한 비율
  - `Hijack %`: 유사도 `< 0.40`인 타인을 타겟으로 오탐 잠금한 비율 (기존 `ByteTrack` 등의 ID 탈취 포착)
  - `Direct Sw` vs `Recovered (Avg f)`: 비-SEARCH 직접 탈취 횟수와 이탈(`SEARCH`) 후 재등장 복귀(`RECOVERED`) 횟수·평균 소요 프레임 분리 집계
  - `Other Avg / Max`, `Sim Margin Mean / Min`
- **4. 추적 알고리즘 비교 표 (`1~5`)** / **5. 3D LiDAR Localizer 비교 표**

### B. 프레임별 전체 시계열 원본 (`timeseries/*.csv`)
- 매 프레임마다 40개 이상 지표(`tgt_sim`, `tgt_u/m/l/f_sim`, `other_max_sim`, `sim_margin`, `u/m/l_valid`, `pose_fallback`, `mask_fg_pct`, `true_locked`, `hijacked`, `bbox_move_px`, `pre/infer/post_track/reid/total_ms`, `loc_*`)를 전부 저장합니다.

### C. 비교 차트 (`charts/`, 최대 7종)
- `chart_models_latency_fps.png`: `YOLO Infer` / `Post+Track` / `ReID` 3단 지연 분해 & Max FPS
- `chart_models_tracking.png`: 모델별 `True Lock %` vs `Hijack %` & 평균 유사도
- `chart_models_part_sim.png`: 부위별(`Total / Upper / Mid / Lower / Full`) 유사도 비교
- `chart_models_parts_and_fg.png`: `U / M / L Valid %` & `Pose Fallback %` vs `Mask FG %` & `Sim Margin`
- `chart_models_timeseries.png`: 동일 시간축 기준 모델별 `Target Sim` / `Sim Margin` / `Frame Latency(ms)`
- `chart_modes_comparison.png`: 알고리즘별 `True Lock %` vs `Hijack %` (`0.40` 임계선 포함)
- `chart_localizer_distance.png`: 3D LiDAR 거리 추정 `Raw` vs `Kinematic+Map+EMA` 비교

### D. 시각 검증 스냅샷 (`snapshots/`)
1. **상태 전환 대시보드 (`<ref>_<STATE>_fXXXX.png`)**: `SAFE(f=15)`, `GUARD`, `SEARCH`, `RECOVERED`, `Occluded` 발생 시 통합 대시보드 저장
2. **동일 프레임 마스크 + 304D HSV 히스토그램 비교 (`compare_tasks_hist_<tag>_fXXXX.png`)**:
   - 기준 모델(`26n-pose`)에서 **혼잡(`crowd`) → 복귀/이탈(`recovered`/`search`) → 최대 움직임(`motion`) → 1/4·1/2·3/4** 순으로 자동 선정한 동일 프레임에서 `26n-det / 26n-seg / 26n-pose`의 마스크 + 304D HSV 히스토그램 대조
3. **실제 추출 픽셀 비교 (`compare_pixels_<tag>_fXXXX.png`)**:
   - `[Overlay | Upper | Mid | Lower | Full FG]` 순서로 **배경을 검정으로 지운 실제 추출 픽셀만** 나란히 보여줘 `Det`의 배경 유입 vs `Seg`의 실루엣 분리 vs `Pose` 관절/폴백 차이를 직접 검증
4. **버전 비교 (`compare_versions_<tag>_fXXXX.png`)**: `v8n-pose` / `11n-pose` / `26n-pose` 동일 프레임 비교
5. **움직임 전·중·후 3프레임 그리드 (`compare_seq_<tag>_fXXXX.png`)**: `[BEFORE (f-Δ) | CENTER (f) | AFTER (f+Δ)] × (Det/Seg/Pose)`

---

## 4. 실시간 수동 오프라인 재생 (`run_offline_bag_test.sh`)

```bash
# [터미널 1] 오프라인 트래커 실행 (자동 격리 DOMAIN_ID=99)
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh tracker

# [터미널 2] 녹화본 재생 (--clock)
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh play ~/bag/complete_bag/test_full

# [터미널 3] (선택) RViz2 동시 확인
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh rviz
```

---

## 5. Human-in-the-Loop GT 클릭 어노테이션 & 정량 평가 (`gt_annotator.py` + `--gt`)

트래커의 `similarity >= 0.40` 휴리스틱 대신 실제 정답(Ground Truth) 기준으로 오인식(Hijack)·놓침(Miss)·재잠금 개입 횟수를 측정하려면:

```bash
# 1) 보조 트래커(26n-seg)를 재생하면서 오인식 순간만 마우스로 클릭해 GT CSV 생성
python3 src/person_tracker/research/gt_annotator.py \
  --bag ~/bag3/pose_estimate2 \
  --out ~/bench_gt/pose_estimate2_gt.csv

# 2) 생성된 GT CSV를 넘겨 모델·알고리즘별 진짜 GT Recall / Hijack / Miss / 개입 횟수 집계
python3 src/person_tracker/research/bag_benchmark.py \
  --bags ~/bag3/pose_estimate2 \
  --models all9 \
  --modes 1,2,3,4,5 \
  --gt ~/bench_gt/pose_estimate2_gt.csv \
  --gt-iou 0.5 \
  --reid-scale
```
- **Left-Click**: 실제 타겟 인물 클릭 지정(GT 기록 + 보조 트래커 즉시 재잠금) / **Left-Drag**: 미검출 구간 수동 BBox
- **Right-Click / `N`**: 타겟 부재·완전 가림(`gt_present=0`) 표시 / **`S`**: 저장 / **`Q`**: 저장 후 종료
