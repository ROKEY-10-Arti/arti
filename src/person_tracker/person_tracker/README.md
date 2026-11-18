# `person_tracker` 핵심 파이프라인 코드 가이드

> [!NOTE]
> 아래 모든 파일·함수 링크는 상대 경로 + 줄 번호 형식(`./파일명.py#L줄번호`)으로 작성되어 있어, **GitHub 웹 UI**와 **VS Code Markdown 미리보기(`Ctrl+Shift+V`)** 양쪽에서 클릭 시 해당 함수 정의 줄로 바로 이동합니다.

---

## 1. 디렉토리 및 모듈 구성

| 파일 | 역할 | 설계도 위치 |
|---|---|---|
| [`gesture_test_node.py`](./gesture_test_node.py#L65) | 메인 ROS 2 노드 (`GestureTestNode`), 영상 입력·추적·제스처 등록·위치 추정·GUI 통합 제어 | 전체 루프(`handle_frame`) |
| [`multi_tracker_core.py`](./multi_tracker_core.py#L84) | YOLO26n-pose + ByteTrack 1차 추적 및 Adaptive ReID (`COEX` / `SAFE` / `GUARD` / `SEARCH`) | 사람 탐지 · 인식 판별 |
| [`feature_extractors.py`](./feature_extractors.py#L368) | 304D HSV 히스토그램 + Pose 17관절 해부학 부위 마스킹(`upper`/`mid`/`lower`/`full`) 및 유사도 비교 | 외형 특징 추출 · EMA |
| [`gesture_core.py`](./gesture_core.py#L145) | 손목 정사각 ROI, MediaPipe 21관절 분류, 2단계 얼굴 오검출 필터, 1:1 손목 매칭, `Fist->Open` 상태머신 | 등록 모듈 |
| [`lidar_person_localizer.py`](./lidar_person_localizer.py#L90) | 카메라 방위각 + 2D LiDAR 정합 거리 추정 + `base_link`/`map` TF 좌표 변환 및 토픽 발행 | 인식 모듈 거리·좌표 변환 |
| [`reid_visualizer.py`](./reid_visualizer.py#L34) | `ReID Monitor` 실시간 그래프, 부위 막대, 304D HSV 히트맵 (`Anchor` / `Recent EMA` / `Compare`) | 디버그 시각화 |
| [`camera_publisher_node.py`](./camera_publisher_node.py#L1) | 로컬 웹캠/DroidCam을 `/robot1/top_cam/image_raw/compressed` ROS 토픽으로 발행 | 영상 입력 퍼블리셔 |
| [`top_cam_tf_node.py`](./top_cam_tf_node.py#L1) | 상단 카메라 정적 TF (`oakd_rgb_camera_frame` -> `top_cam_link` -> `top_cam_optical_frame`) 발행 | `/tf_static` 퍼블리셔 |
| [`env_bootstrap.py`](./env_bootstrap.py#L1) | `rokey_venv` 파이썬 경로 부트스트랩 및 네임스페이스 토픽·모델 경로 해석 | 공통 유틸 |

- 전체 draw.io 설계도: [`docs/follow/person_tracker_flow.drawio`](../../../docs/follow/person_tracker_flow.drawio)

---

## 2. ROS 2 메시지 인터페이스

| 구분 | 토픽 / 소스 | 메시지 타입 | 설명 |
|---|---|---|---|
| **입력 (영상)** | `/<ns>/top_cam/image_raw/compressed` (또는 `video_source`) | `sensor_msgs/CompressedImage` (`Image` 선택 가능) | 상단 웹캠 BGR 영상 (웹캠 직접 입력 시 `640x480`) |
| **입력 (라이다)** | `/<ns>/scan` | `sensor_msgs/LaserScan` | 2D LiDAR 거리 스캔 (`rplidar_link`) |
| **입력 (TF)** | `/<ns>/tf`, `/<ns>/tf_static` | `tf2_msgs/TFMessage` | `map -> odom -> base_link -> top_cam_link -> rplidar_link` |
| **출력 (타겟 맵 좌표)** | `/<ns>/target_person/point_map` | `geometry_msgs/PointStamped` | `map` 기준 타겟 관람객 좌표 (`frame_id: map`) |
| **출력 (로봇 기준 좌표)** | `/<ns>/target_person/point_base` | `geometry_msgs/PointStamped` | 로봇 `base_link` 기준 타겟 상대 좌표 |
| **출력 (RViz 마커)** | `/<ns>/target_person/marker` | `visualization_msgs/Marker` | 타겟 위치 실린더 마커 (`LiDAR`=초록, `CamEst`=주황) |
| **출력 (제스처 상태)** | `/<ns>/gesture/status` | `std_msgs/String` | `SCANNING`, `FIST_ARMED`, `TRIGGERED`, `LOCKED`, `LOST_WAIT` |

---

## 3. 전체 실행 흐름

```text
[영상 입력: _compressed_cb / _raw_cb / _poll_local_frame]
   │
   ▼
GestureTestNode.handle_frame()  (./gesture_test_node.py#L511)
   │
   ├─▶ MultiModelTrackerCore.process_frame()  (./multi_tracker_core.py#L424)
   │      └─ YOLO26n-pose + ByteTrack으로 전 인원 bbox, track_id, 17관절 keypoints 추출
   │
   ├─ [미등록: target_id is None] ──▶ 등록 모듈 (gesture_core.py)
   │      ├─ bw < 44px 원거리 제외
   │      ├─ build_person_hand_rois()         (./gesture_core.py#L514)  손목 연장선 정사각 ROI
   │      ├─ detect_in_roi()                  (./gesture_core.py#L168)  MediaPipe 21관절 검출
   │      ├─ classify_hand_gesture()          (./gesture_core.py#L88)   FIST / OPEN_PALM 판정
   │      ├─ find_best_hand_owner()           (./gesture_core.py#L676)  얼굴 오검출 2단계 제외 + 손목 1:1 매칭
   │      ├─ pick_primary_observation()       (./gesture_core.py#L239)  대표 손 1개 선정
   │      ├─ FistOpenStateMachine.update()    (./gesture_core.py#L299)  Fist(4f) -> Open(4f) 전이
   │      └─ select_target_by_id()            (./multi_tracker_core.py#L404) 타겟 ID & Anchor 고정
   │
   └─ [등록됨: target_id is not None] ──▶ 인식 모듈 (Adaptive ReID + Localizer)
          ├─ PoseKeypointFeatureExtractor.extract()       (./feature_extractors.py#L379)
          ├─ compute_dual_similarity_detail()             (./feature_extractors.py#L194)
          ├─ MODE_ADAPTIVE (COEX / SAFE / GUARD / SEARCH) (./multi_tracker_core.py#L620)
          ├─ 수동 초기화 [R] 클릭 시 _reset_target()       (./gesture_test_node.py#L294)
          └─ LidarPersonLocalizer.localize_target()       (./lidar_person_localizer.py#L368)
                 └─ 방위각 + LiDAR p25 거리 -> cam_xyz -> base_xyz -> map_xyz 발행
```

---

## 4. 모듈별 주요 함수 인덱스 (클릭 이동)

### ① [`gesture_test_node.py`](./gesture_test_node.py) — 진입점 · 루프 제어
- [`main()`](./gesture_test_node.py#L766): ROS 2 노드 초기화, `spin_once()` + 유휴 GUI 갱신(`refresh_idle_gui`) 루프
- [`GestureTestNode.__init__()`](./gesture_test_node.py#L75): 파라미터 선언, 제스처/트래커/로컬라이저/시각화 객체 초기화 및 입력 소스 연결
- [`_ensure_reid_window()`](./gesture_test_node.py#L200): `ReID Monitor` 실시간 튜닝 트랙바(`Match%`, `Coex%`, `Margin%`, `ConfirmF`) 생성
- [`_start_local_capture()`](./gesture_test_node.py#L241) / [`_capture_loop()`](./gesture_test_node.py#L257) / [`_poll_local_frame()`](./gesture_test_node.py#L271): `video_source` 지정 시 V4L2 비동기 캡처 스레드 + 최신 1프레임 소비
- [`_compressed_cb()`](./gesture_test_node.py#L278) / [`_raw_cb()`](./gesture_test_node.py#L287): ROS 이미지 토픽 수신 디코딩
- [`_reset_target()`](./gesture_test_node.py#L294): 타겟 잠금, 제스처 상태머신 및 ReID 그래프 일괄 초기화
- [`_estimate_candidate_spatial()`](./gesture_test_node.py#L502): ReID 후보 점수 계산용 3D 좌표 콜백 (토픽 미발행)
- [`handle_frame()`](./gesture_test_node.py#L511): **매 프레임 메인 파이프라인** (추적 → 제스처 스캔 → 위치 추정·발행 → GUI 출력)
- [`_on_mouse_click()`](./gesture_test_node.py#L316) / [`_process_key()`](./gesture_test_node.py#L340): 상단 UI 클릭, 수동 타겟 지정, `[Y]/[R]/[1~4]/[D]/[V]/[P]` 단축키 처리

### ② [`multi_tracker_core.py`](./multi_tracker_core.py) — 추적 · Adaptive ReID 상태머신
- [`MultiModelTrackerCore.__init__()`](./multi_tracker_core.py#L119): 기본 `yolo26n-pose.pt` + `MODE_ADAPTIVE` + `COEX`/`SAFE`/`SEARCH` 임계값 설정
- [`_lock_target_bank()`](./multi_tracker_core.py#L363) / [`_activate_target()`](./multi_tracker_core.py#L387) / [`select_target_by_id()`](./multi_tracker_core.py#L404): 타겟 확정 시 `anchor_bank`·`recent_bank` 동시 고정 및 상태 카운터 초기화
- [`_is_crowded_near_target()`](./multi_tracker_core.py#L280): `IoU > 0.02` 또는 근접 인원 여부로 `SAFE` / `GUARD` 분기 판별
- [`_compute_spatial_score()`](./multi_tracker_core.py#L297) / [`_score_candidate()`](./multi_tracker_core.py#L350): 직전 3D 위치 기반 보행 반경(`0.65 + 1.35 * dt`) 가우시안 공간 점수 및 `0.65*외형 + 0.35*공간` 결합 점수 산출
- [`process_frame()`](./multi_tracker_core.py#L424): YOLO 추적 → 특징 비교 → `MODE_ADAPTIVE` 처리
  - `COEX` 등록([`L622`](./multi_tracker_core.py#L622)): BBox 비중첩(`IoU < 0.05`)·원거리(`|dx| > 0.9*tw`) 비타겟 ID 기억
  - `SAFE` 이탈 검사([`L656`](./multi_tracker_core.py#L656)): 다수 인원(`len >= 2`) 시 `sim < 0.48` 4연속이면 `target_id = -1`로 해제 후 `SEARCH` 진입 / 1인 고신뢰(`>= 0.72`) 시 `recent_bank` EMA 갱신
  - `SEARCH` 복귀([`L674`](./multi_tracker_core.py#L674)): `eligible` 필터(`COEX`는 `>= coexist_min_sim`일 때만 통과) → `margin` 격차(`0.06`) → `2프레임` 연속 확정 시 `RECOVERED`
  - `GUARD` 혼잡 보호([`L769`](./multi_tracker_core.py#L769)): `exclude_boxes`로 겹친 타인 제거 + `0.60*외형 + 0.40*공간 + 0.04*관성` 재선정

### ③ [`feature_extractors.py`](./feature_extractors.py) — 304D HSV · Pose 신체 부위 마스킹
- [`PartFeatureBank`](./feature_extractors.py#L30): `full`, `upper`, `mid`, `lower` 4종 벡터 컨테이너
- [`BaseFeatureExtractor.calc_hsv_hist()`](./feature_extractors.py#L60): `0.72 * (18x16 H-S)` + `0.28 * (16 V)` L1 정규화 304차원 히스토그램 생성
- [`BaseFeatureExtractor.hist_sim()`](./feature_extractors.py#L83): Bhattacharyya 거리 기반 `max(0, 1 - dist)` 유사도
- [`BaseFeatureExtractor._apply_exclude_boxes()`](./feature_extractors.py#L91): 타인 BBox 교차 영역을 마스크에서 0으로 제거
- [`BaseFeatureExtractor.compare_banks_detail()`](./feature_extractors.py#L125): 가용 부위 가중 평균(`0.44 / 0.36 / 0.20`)과 `full_score` 중 최댓값 계산
- [`BaseFeatureExtractor.compute_dual_similarity_detail()`](./feature_extractors.py#L194): `anchor_bank` vs `recent_bank` 비교 후 더 높은 종합·부위 유사도 선택
- [`BaseFeatureExtractor.update_recent_bank()`](./feature_extractors.py#L236): `alpha = 0.04` EMA 갱신
- [`PoseKeypointFeatureExtractor.extract()`](./feature_extractors.py#L379): `ALLOW_CROSS_PART = False`, 어깨·골반(`upper`), 골반·무릎(`mid`), 무릎·발목(`lower`) 해부학 마스크 생성 (관절 부족 시 [`BBoxFeatureExtractor.extract()`](./feature_extractors.py#L256) 폴백)

### ④ [`gesture_core.py`](./gesture_core.py) — 손 제스처 ROI · 얼굴 오검출 필터 · 상태머신
- [`build_person_hand_rois()`](./gesture_core.py#L514) / [`_clamp_square_roi()`](./gesture_core.py#L497): 팔꿈치→손목 `+25%` 연장선 정사각 ROI 생성 (손목 미검출 시 [`build_person_gesture_roi()`](./gesture_core.py#L452) 상반신 폴백)
- [`HandGestureDetector.detect_in_roi()`](./gesture_core.py#L168): ROI 크롭 → MediaPipe `HandLandmarker` 21관절 검출 → 원본 좌표 복원
- [`classify_hand_gesture()`](./gesture_core.py#L88): `palm_scale = dist(0, 9)` 기준 손가락 거리비로 `FIST` / `OPEN_PALM` / `UNKNOWN` 판정
- [`is_false_face_detection()`](./gesture_core.py#L561): 얼굴 영역(코·눈·귀 0~4 반경) 내 손 중 Pose 손목(9,10) 근접이 아닌 얼굴 오검출 제거
- [`match_hand_to_person_wrist()`](./gesture_core.py#L610) / [`find_best_hand_owner()`](./gesture_core.py#L676): 크기 하한(`bw >= 44`, `palm_scale >= 12`) + 얼굴 필터 + Pose 손목 1:1 최단거리 주인 배정
- [`HandGestureDetector.pick_primary_observation()`](./gesture_core.py#L239): 동일 인물의 다중 손 중 명확도·크기 기준 대표 손 1개 선택
- [`MultiTargetGestureManager.update()`](./gesture_core.py#L422) / [`FistOpenStateMachine.update()`](./gesture_core.py#L299): `IDLE -> FIST_ARMED(4f) -> TRIGGERED(Open 4f)` 전이 (`grace_frames=3` 유예)

### ⑤ [`lidar_person_localizer.py`](./lidar_person_localizer.py) — 방위각 · LiDAR 거리 · 맵 좌표 변환
- [`LidarPersonLocalizer.__init__()`](./lidar_person_localizer.py#L101): YAML/파라미터 로드, `tf`/`tf_static`/`scan` 구독 및 `point_base`/`point_map`/`marker` 퍼블리셔 생성
- [`_on_scan()`](./lidar_person_localizer.py#L259) / [`_on_tf_msg()`](./lidar_person_localizer.py#L251) / [`_on_tf_static_msg()`](./lidar_person_localizer.py#L255): 센서 메시지 수신 및 `tf_buffer` 적재
- [`_get_map_from_base_matrix()`](./lidar_person_localizer.py#L289): 1순위 `AMCL`(`map->base_link` TF), 2순위 `Init+Odom` 변환 행렬 산출
- [`localize_target()`](./lidar_person_localizer.py#L368): 핀홀 방위각(`alpha_c`) → LiDAR 섹터 25백분위(`p25`) 거리 정합(미수신 시 `CamEst` 폴백) → `cam_xyz` → `base_xyz` → `map_xyz` 계산
- [`_publish_target_outputs()`](./lidar_person_localizer.py#L495): `target_person/point_base`, `target_person/point_map`, `target_person/marker` 발행

### ⑥ [`reid_visualizer.py`](./reid_visualizer.py) — ReID 디버그 모니터
- [`ReIDDebugVisualizer.render()`](./reid_visualizer.py#L117): 상단 State/`Search Status`, 120f 유사도 시계열 그래프, 인원별 `Tot/U/M/L/F` 막대, `Anchor`/`Recent EMA`/`Compare` 304D 히트맵 및 `A-R Sim` 렌더링
- [`_render_hsv_vector_patch()`](./reid_visualizer.py#L65): 304D 벡터를 `18x16 H-S` 컬러 히트맵(`sqrt` 정규화) + `16-bin V` 밝기 막대로 변환

---

## 5. 실행 및 테스트 방법

```bash
# 집/로컬 테스트 시 실습실 Discovery Server 비활성화 (양쪽 터미널 모두 실행)
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
ros2 daemon stop
```

- **로컬 웹캠 단독 테스트 (로컬라이저 끔)**:
  ```bash
  ros2 run person_tracker gesture_test_node --ros-args -r __ns:=/robot1 -p video_source:=0 -p use_localizer:=false
  ```
- **rosbag 재생 테스트 (인식 + LiDAR/TF 좌표 계산)**:
  ```bash
  # 터미널 1
  ros2 bag play ~/bag/rosbag2_2026_11_17-21_21_09 --loop
  # 터미널 2 (과거 bag 내 target_person 토픽과 겹치지 않게 발행 여부 확인 시 publish_target_topics 조절)
  ros2 run person_tracker gesture_test_node --ros-args -r __ns:=/robot1 -p use_localizer:=true
  ```
- **GUI 단축키**: `[Y]` YOLO+ROI 전환 · `[R]` 타겟 초기화 · `[1/2]` Fist 프레임 · `[3/4]` Open 프레임 · `[D]` CPU/GPU 전환 · `[V]` ReID 모니터 창 토글 · `[P]` 타겟 토픽 발행 토글
