# `person_tracker` 핵심 파이프라인 코드 가이드

> [!NOTE]
> 아래 모든 파일·함수 링크는 상대 경로 + 줄 번호 형식(`./파일명.py#L줄번호`)으로 작성되어 있어, **GitHub 웹 UI**와 **VS Code Markdown 미리보기(`Ctrl+Shift+V`)** 양쪽에서 클릭 시 해당 함수 정의 줄로 바로 이동합니다.

---

## 1. 디렉토리 및 모듈 구성

| 파일 | 역할 | 설계도 위치 |
|---|---|---|
| [`gesture_test_node.py`](./gesture_test_node.py#L65) | 메인 ROS 2 노드 (`GestureTestNode`), 영상 입력·추적·제스처 등록·위치 추정·GUI 통합 제어 | 전체 루프(`handle_frame`) |
| [`multi_tracker_core.py`](./multi_tracker_core.py#L85) | YOLO26n-pose + ByteTrack 1차 추적 및 Adaptive ReID (`COEX` / `SAFE` / `GUARD` / `SEARCH`) | 사람 탐지 · 인식 판별 |
| [`feature_extractors.py`](./feature_extractors.py#L368) | 304D HSV 히스토그램 + Pose 17관절 해부학 부위 마스킹(`upper`/`mid`/`lower`/`full`) 및 유사도 비교 | 외형 특징 추출 · EMA |
| [`gesture_core.py`](./gesture_core.py#L145) | 손목 정사각 ROI, MediaPipe 21관절 분류, 2단계 얼굴 오검출 필터, 1:1 손목 매칭, `Fist->Open` 상태머신 | 등록 모듈 |
| [`lidar_person_localizer.py`](./lidar_person_localizer.py#L90) | 카메라 방위각 + 2D LiDAR 정합 거리 추정 + `base_link`/`map` TF 좌표 변환 및 토픽 발행 | 인식 모듈 거리·좌표 변환 |
| [`reid_visualizer.py`](./reid_visualizer.py#L34) | `ReID Monitor` 실시간 그래프, 부위 막대, 304D HSV 히트맵 (`Anchor` / `Recent EMA` / `Compare`) | 디버그 시각화 |
| [`camera_publisher_node.py`](./camera_publisher_node.py#L1) | 로컬 웹캠/DroidCam을 `/robot1/top_cam/image_raw/compressed` ROS 토픽으로 발행 | 영상 입력 퍼블리셔 |
| [`top_cam_tf_node.py`](./top_cam_tf_node.py#L1) | 상단 카메라 정적 TF (`oakd_rgb_camera_frame` -> `top_cam_link` -> `top_cam_optical_frame`) 발행 | `/tf_static` 퍼블리셔 |
| [`env_bootstrap.py`](./env_bootstrap.py#L1) | `rokey_venv` 파이썬 경로 부트스트랩 및 네임스페이스 토픽·모델 경로 해석 | 공통 유틸 |

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

## 3. 모듈 입출력 요약

```mermaid
flowchart LR
    CAM["/robot1/top_cam/image_raw/compressed [CompressedImage]"] -->|"frame: np.ndarray"| REG["등록 모듈 (target_id is None): YOLO26n-pose + 2단계 손목 매칭 + Fist->Open"]
    REG -->|"target_id, anchor_bank (304D HSV)"| REC
    CAM -->|"frame: np.ndarray"| REC["인식 모듈 (target_id != None): Pose ReID + 방위각·LiDAR 정합 + TF 변환"]
    SCAN["/robot1/scan [LaserScan]"] --> REC
    TF["/robot1/tf, /tf_static [TFMessage]"] --> REC
    REC -->|"point_map, point_base [PointStamped], marker [Marker]"| OUT["주행 / RViz 출력"]
```

---

## 4. 모듈별 동작 흐름 · 입출력 인터페이스 다이어그램 (클릭 이동)

각 모듈의 **순수 동작 흐름(핵심 함수)과 입출력 인터페이스**만 간결하게 정리했습니다.

---

### ① 등록 모듈 동작 흐름 (`target_id is None`)

```mermaid
flowchart TD
    IN["Input: /robot1/top_cam/image_raw/compressed [CompressedImage]"]
    DET["사람 탐지: process_frame() - YOLO26n-pose + ByteTrack (bw >= 44px)"]
    ROI["손목 ROI & 손 분류: build_person_hand_rois() -> detect_in_roi() -> classify_hand_gesture()"]
    FLT["2단계 얼굴 오검출 제외 & 손목 1:1 매칭: find_best_hand_owner() + is_false_face_detection() -> pick_primary_observation()"]
    SM["제스처 상태머신 (Fist 4f -> Open 4f): MultiTargetGestureManager.update() -> FistOpenStateMachine.update()"]
    LOCK["타겟 고정: select_target_by_id() -> _lock_target_bank()"]
    OUT["Output: target_id(int), anchor_bank(PartFeatureBank 304D), /robot1/gesture/status [String]"]

    IN -->|"frame: np.ndarray(BGR)"| DET
    DET -->|"person(bbox, keypoints[17,3])"| ROI
    ROI -->|"List[HandGestureObservation]"| FLT
    FLT -->|"primary: HandGestureObservation"| SM
    SM -->|"prog.just_triggered == True"| LOCK
    LOCK --> OUT
```

- **함수 바로 이동 (등록 모듈 순서)**:
  - 입력·탐지: [`_compressed_cb()`](./gesture_test_node.py#L317) / [`_raw_cb()`](./gesture_test_node.py#L326) / [`_poll_local_frame()`](./gesture_test_node.py#L310) → [`handle_frame()`](./gesture_test_node.py#L550) → [`process_frame()`](./multi_tracker_core.py#L438)
  - 손 ROI·분류: [`build_person_hand_rois()`](./gesture_core.py#L514) ([`_clamp_square_roi()`](./gesture_core.py#L497) / [`build_person_gesture_roi()`](./gesture_core.py#L452)) → [`detect_in_roi()`](./gesture_core.py#L168) → [`classify_hand_gesture()`](./gesture_core.py#L88)
  - 얼굴 제외·손목 1:1 매칭: [`find_best_hand_owner()`](./gesture_core.py#L676) → [`match_hand_to_person_wrist()`](./gesture_core.py#L610) ([`is_false_face_detection()`](./gesture_core.py#L561)) → [`pick_primary_observation()`](./gesture_core.py#L239)
  - 상태머신·잠금: [`MultiTargetGestureManager.update()`](./gesture_core.py#L422) → [`FistOpenStateMachine.update()`](./gesture_core.py#L299) → [`select_target_by_id()`](./multi_tracker_core.py#L418) → [`_activate_target()`](./multi_tracker_core.py#L401) → [`_lock_target_bank()`](./multi_tracker_core.py#L377)

---

### ② 인식 모듈 동작 흐름 (`target_id != None`)

```mermaid
flowchart TD
    IN["Input: /robot1/top_cam/image_raw/compressed [CompressedImage]"]
    TRK["사람 추적: process_frame() - YOLO26n-pose + ByteTrack"]
    EXT["Pose 부위 추출 & 비교: PoseKeypointFeatureExtractor.extract() (3부위) -> compute_dual_similarity_detail() (304D HSV)"]
    REID["Adaptive ReID 판별: MODE_ADAPTIVE (COEX · SAFE 4f 이탈 · GUARD · SEARCH)"]
    SCAN["Input: /robot1/scan [LaserScan] - _on_scan()"]
    TF["Input: /robot1/tf, /tf_static [TFMessage] - _on_tf_msg() / _lookup()"]
    DIST["방위각 & 거리 측정: localize_target() (핀홀 alpha_c + p25 클러스터 평균 / CamEst 폴백)"]
    COORD["3D 좌표 변환 (CAM -> base -> map): localize_target() -> _get_map_from_base_matrix() (AMCL / Init+Odom)"]
    PUB["위치 발행: _publish_target_outputs()"]
    OUT["Output: /robot1/target_person/point_map, point_base [PointStamped], marker [Marker]"]

    IN -->|"frame: np.ndarray(BGR)"| TRK
    TRK -->|"detections: List[UnifiedTrackedPerson]"| EXT
    EXT -->|"PartFeatureBank(304D), similarity"| REID
    REID -->|"target(bbox)"| DIST
    SCAN -->|"latest_scan [LaserScan]"| DIST
    TF -->|"T_cam_lidar"| DIST
    DIST -->|"CAM(x,y,z)"| COORD
    TF -->|"T_base_cam, T_map_base"| COORD
    COORD -->|"MAP(x,y,z)"| PUB
    PUB --> OUT
```

- **함수 바로 이동 (인식 모듈 순서)**:
  - 외형 추출·유사도: [`PoseKeypointFeatureExtractor.extract()`](./feature_extractors.py#L379) ([`BBoxFeatureExtractor.extract()`](./feature_extractors.py#L256)) → [`calc_hsv_hist()`](./feature_extractors.py#L60) → [`PartFeatureBank`](./feature_extractors.py#L30) → [`compute_dual_similarity_detail()`](./feature_extractors.py#L194) → [`compare_banks_detail()`](./feature_extractors.py#L125) / [`hist_sim()`](./feature_extractors.py#L83)
  - Adaptive ReID 상태머신: [`MODE_ADAPTIVE`](./multi_tracker_core.py#L640) (`COEX` [`L641`](./multi_tracker_core.py#L641) · `SAFE` [`L674`](./multi_tracker_core.py#L674) · `SEARCH` [`L704`](./multi_tracker_core.py#L704) · `GUARD` [`L794`](./multi_tracker_core.py#L794)), [`update_recent_bank()`](./feature_extractors.py#L236), [`_apply_exclude_boxes()`](./feature_extractors.py#L91), [`_is_crowded_near_target()`](./multi_tracker_core.py#L294), [`_compute_spatial_score()`](./multi_tracker_core.py#L311), [`_score_candidate()`](./multi_tracker_core.py#L364)
  - 센서 수신·좌표 변환·발행: [`LidarPersonLocalizer.__init__()`](./lidar_person_localizer.py#L101), [`_on_scan()`](./lidar_person_localizer.py#L259), [`_on_tf_msg()`](./lidar_person_localizer.py#L251), [`_on_tf_static_msg()`](./lidar_person_localizer.py#L255), [`_estimate_candidate_spatial()`](./gesture_test_node.py#L541) → [`localize_target()`](./lidar_person_localizer.py#L368) → [`_get_map_from_base_matrix()`](./lidar_person_localizer.py#L289) → [`_publish_target_outputs()`](./lidar_person_localizer.py#L495)
  - 수동 제어·리셋: [`_on_mouse_click()`](./gesture_test_node.py#L355), [`_process_key()`](./gesture_test_node.py#L379), [`_reset_target()`](./gesture_test_node.py#L333)

---

### ③ 공통 디버그 시각화 (`ReID Monitor` & GUI)
- [`ReIDDebugVisualizer.render()`](./reid_visualizer.py#L117) / [`_render_hsv_vector_patch()`](./reid_visualizer.py#L65) / [`_ensure_reid_window()`](./gesture_test_node.py#L225): 120f 유사도 시계열 그래프, 인원별 `Tot/U/M/L/F` 막대, `Anchor` / `Recent EMA` / `Compare` 304D 히트맵 및 `A-R Sim`·`EMA` 상태 렌더링 (기동 진입점: [`main()`](./gesture_test_node.py#L805), [`__init__()`](./gesture_test_node.py#L75))

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
- **주요 ReID/EMA ROS 2 파라미터** (`--ros-args -p <이름>:=<값>`): `ema_min_sim` (`0.60`), `ema_alpha` (`0.04`), `ema_require_single` (`true`), `tracker_conf` (`0.15`), `tracker_iou` (`0.55`), `reid_match_thresh` (`0.50`), `search_min_sim` (`0.48`), `coexist_min_sim` (`0.74`), `recover_margin` (`0.06`), `recover_confirm_frames` (`2`), `id_switch_thresh` (`0.48`), `safe_mismatch_limit` (`4`)
- **GUI 단축키**: `[Y]` YOLO+ROI 전환 · `[R]` 타겟 초기화 · `[1/2]` Fist 프레임 · `[3/4]` Open 프레임 · `[D]` CPU/GPU 전환 · `[V]` ReID 모니터 창 토글 · `[P]` 타겟 토픽 발행 토글
