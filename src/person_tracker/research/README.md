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
| `--snap-frames` | 자동 선정 | Det/Seg/Pose 비교 중심 프레임 번호 직접 지정 (예: `40,120,250`) |
| `--seq-delta Δ` | `10` | 전·중·후 3프레임 시퀀스 비교 간격 (`f-Δ`, `f`, `f+Δ`) |
| `--reid-scale` | 꺼짐 | `follow_test`의 REID 완화 파라미터(`ema_min_sim=0.52` 등) 적용 |
| `--no-localizer-compare` | 비교 켜짐 | LiDAR `Raw` vs `Kinematic+Map+EMA` 비교 생략 |
| `--no-snapshots` | 저장 켜짐 | 대시보드 및 비교 스냅샷 PNG 저장 생략 |

---

## 3. 자동 생성되는 출력 파일 및 스냅샷 기준

실행이 끝나면 `<out>/<bag_name>/` 아래에 다음 파일들이 생성됩니다.

### A. 요약 표 (발표 자료 바로 복붙용)
- `summary.md`: 모델 비교(추론/후처리/ReID 분리 시간, Max FPS, 부위별 U/M/L/F 유사도, Margin), 알고리즘 비교, LiDAR 비교 Markdown 표
- `summary_models.csv`, `summary_modes.csv`, `summary_localizer.csv`

### B. 프레임별 전체 시계열 원본 데이터 (`timeseries/`)
- `model_<name>.csv`, `mode_<idx>_<name>.csv`, `localizer_<cfg>.csv`
- 매 프레임마다 아래 지표를 전부 기록합니다:
  - `frame_idx`, `t_sec`, `num_persons`, `locked`, `target_id`, `adaptive_state`, `ema_status`, `search_reason`
  - `tgt_sim`, `tgt_u_sim`, `tgt_m_sim`, `tgt_l_sim`, `tgt_f_sim`, `other_max_sim`, `other_avg_sim`, `sim_margin`
  - `bbox_x1..y2`, `bbox_cx`, `bbox_cy`, `bbox_area_ratio`, `bbox_move_px`
  - `pre_ms`, `infer_ms`, `post_track_ms`, `reid_ms`, `total_ms`
  - `loc_source`, `loc_dist_m`, `loc_base_x/y`, `loc_map_x/y`

### C. 비교 차트 (`charts/`)
- `chart_models_latency_fps.png`: 모델별 `YOLO Infer` / `Post+Track` / `ReID` 지연시간 누적 막대 & Max FPS
- `chart_models_tracking.png`: 모델별 타겟 잠금 유지율(`Locked %`) & 평균 유사도
- `chart_models_part_sim.png`: 모델/태스크별 부위별 평균 유사도(`Total`, `Upper`, `Mid`, `Lower`, `Full`) — Det 배경 섞임 vs Seg/Pose 차이 비교
- `chart_models_timeseries.png`: 동일 시간축 기준 모델별 `Target Sim` / `Sim Margin` / `Frame Latency(ms)` 동시 비교
- `chart_modes_comparison.png`: 추적 알고리즘(`1~5`)별 유지율·ID Switch 및 시간별 유사도 곡선
- `chart_localizer_distance.png`: 3D LiDAR 거리 추정 `Raw` vs `Kinematic+Map+EMA` 비교

### D. 스냅샷 선정 기준 및 생성 이미지 (`snapshots/`)
1. **상태 전환 대시보드 (`<ref_model>_<STATE>_fXXXX.png`)**:
   - 기준 모델(`26n-pose`)에서 초기 안정(`SAFE` @ `f=15`, 1장), 혼잡(`GUARD`, 최대 2장), 이탈(`SEARCH`, 최대 2장), 복귀(`RECOVERED`, 최대 2장), 라이다 가림(`Occluded`, 최대 2장) 발생 시 카메라 + ReID 패널 + 벤치마크 패널 통합 대시보드를 저장합니다.
2. **동일 프레임 마스크 + 304D HSV 히스토그램 비교 (`compare_tasks_hist_<tag>_fXXXX.png`)**:
   - 기준 모델 시계열에서 **① 혼잡/타인 유사도 최고점(`crowd`) → ② 이탈/복귀(`search`/`recovered`) → ③ 타겟 이동량 최대(`motion`) → ④ 부족 시 1/4·1/2·3/4 균등 지점** 순으로 최대 3개 중심 프레임을 자동 선정합니다 (`--snap-frames`로 직접 지정 가능).
   - 선정된 동일 프레임에서 `26n-det` / `26n-seg` / `26n-pose` 각각의 **상단(마스크·스켈레톤 + Sim/U/M/L/F/지연시간 배너) + 하단(304D HSV Anchor/Recent/Other 히스토그램 히트맵 + 120f 시계열)** 을 나란히 비교합니다.
3. **움직임·이벤트 전·중·후 3프레임 그리드 (`compare_seq_<tag>_fXXXX.png`)**:
   - 위 중심 프레임 `f`에 대해 **`[BEFORE (f-Δ) | CENTER (f) | AFTER (f+Δ)]`** 3프레임을 행(`26n-det`, `26n-seg`, `26n-pose`) × 열(전/중/후) 그리드로 합성해 움직임·교차 전후의 마스크와 유사도 변화를 한눈에 비교합니다.

---

## 4. 실시간 수동 오프라인 재생 (`run_offline_bag_test.sh`)

자동 벤치마크 외에 실시간 GUI 창에서 직접 모델/모드 버튼(`Q/W/E`, `A/S/F`, `1~5`)과 ReID 트랙바를 눌러보며 확인하고 싶을 때 사용합니다.

```bash
# [터미널 1] 오프라인 트래커 실행 (자동 격리 DOMAIN_ID=99)
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh tracker

# [터미널 2] 녹화본 재생 (--clock)
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh play ~/bag/complete_bag/test_full

# [터미널 3] (선택) RViz2 동시 확인
bash ~/project/arti/src/person_tracker/research/run_offline_bag_test.sh rviz
```
