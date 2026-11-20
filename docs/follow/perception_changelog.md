# `person_tracker` 인식·추적 모듈 개선 요약 (포트폴리오 / 발표 정리용)

> 대상 브랜치: `feature/follow-gesture` (`main` `b550271` 통합 및 후속 개선)  
> 역할: Robot1 자유관람 관람객 **제스처 등록 · 다부위 ReID 추적 · 2D LiDAR/TF 기반 3D 위치 추정**

---

## 1. 문제 해결 및 핵심 개선 내역

| 구분 | 기존 문제 | 해결 방법 | 수정 파일 · 커밋 |
|---|---|---|---|
| **① 제스처 기반 타겟 등록 안정화** | 전체 화면 손 검출 시 다인 환경에서 얼굴 오검출·타인 손 오인식 발생 | Pose 손목(9·10번) 우선 정사각 ROI 생성 + 얼굴 관절 제외 필터 + 1:1 최단거리 주인 귀속 + `Fist(4f) → Open(4f)` 상태머신 적용 | `gesture_core.py`, `gesture_test_node.py` (`509ac8c`) |
| **② 유실·재식별(ReID) 튜닝 및 파라미터화** | `recent_bank` EMA 갱신 기준(`0.72`)이 높아 조명/자세 변화 적응이 제한적이고, 임계값이 코드에 고정 | `ema_min_sim`을 `0.60`으로 완화하고 `tracker_*`·`reid_*`·`ema_*` 15종을 ROS 2 파라미터화. `ReID` 창 헤더에 `EMA` 실시간 상태 출력 | `multi_tracker_core.py`, `gesture_test_node.py`, `reid_visualizer.py` (`0b0301c`) |
| **③ 부분 가림·근접 등록 보완 (`UPGRADE`/`BACKFILL`)** | 근접 촬영 시 관절 부족으로 BBox 3등분 폴백이 얼굴까지 옷 특징으로 오등록, 등록 때 비었던 부위(`None`)는 이후 전신이 보여도 미활용 | 어깨만 보일 때 **어깨선 아래 상의만 추출**; BBox 폴백 시 `is_fallback=True` 표기 후 안전 구간에서 Pose 2프레임 연속 검출 시 **`UPGRADE`**, `None` 부위는 안전 구간 2프레임 연속 관측 시 **`BACKFILL`** | `feature_extractors.py`, `multi_tracker_core.py` (`5e41dbb`) |
| **④ 이미지 ↔ LiDAR ↔ TF 시간 동기화** | 추론 시간(50~100ms) 후 최신 `scan`·`TF()`를 조회해 로봇 회전 시 방위각·`map→odom` 변환 오차 발생 가능 | 영상 `header.stamp`를 파이프라인 끝까지 전달하고, 40개 `/scan` 링 버퍼 중 최단 시차 스캔(`scan_sync_slop_sec=0.25s`) + `frame_stamp` 기준 TF 조회 및 출력 타임스탬프 정합 | `gesture_test_node.py`, `lidar_person_localizer.py` (`5e41dbb`) |
| **⑤ 디버그 시각화 고도화** | `gesture_test_node`는 BBox만 표시돼 마스크 샘플링 영역 확인이 어려웠고, ReID 히트맵은 2열 혼합 배치 | 실제 추출 마스크(`upper`/`mid`/`lower`) **반투명(`0.34`) + 외곽선**·Pose 마커 표시(`[M]` 토글); ReID 304D 히트맵을 **행별 1줄(`Anchor` / `Recent EMA` / `Other or Target`) × 4열(`Upper`/`Mid`/`Lower`/`Full`)**로 재구성 | `feature_extractors.py`, `gesture_test_node.py`, `reid_visualizer.py` (`5e41dbb`) |
| **⑥ 공용 노드 베이스 분리 & 모터 안전 제어** | `multi_tracker_node`와 `gesture_test_node`의 ROS 입출력/TF·시각화 중복, 관측 전용 시에도 `/start_motor` 자동 호출 가능 | 공용 **`BaseTrackerNode`** + 선택적 **`TrackerStateStore`**(`enable_state_store`) 분리 (`main`에서 `front_tracker_node`도 동일 베이스 상속); **`auto_start_lidar_motor`** 파라미터로 관측 전용 시 자동 모터 기동 차단 | `base_tracker_node.py`, `tracker_state.py`, `lidar_person_localizer.py` (`466ea7e`, `315ab8a`) |
| **⑦ 양손 교차 제스처 오등록 방지** | 한 사람 양손 중 왼손은 주먹(`FIST_ARMED`), 오른손은 편 손(`OPEN_PALM`)일 때 대표 손 1개만 뽑는 구조 때문에 손이 번갈아 잡히면 즉시 등록(`TRIGGERED`) | Pose 손목(`9:L`, `10:R`) 및 폴백 좌/우 슬롯별 **독립 `FistOpenStateMachine`(키 `(subject_id, hand_label)`)** 관리로 서로 다른 손 상태 혼합 원천 차단 | `gesture_core.py`, `gesture_test_node.py` (`1096d63`) |
| **⑧ 실행 중 영상 소스 동적 전환** | `BaseTrackerNode`가 `__init__()` 1회만 `video_source`/`image_topic`을 읽어 실행 중 웹캠 ↔ 토픽 전환 불가 | `add_on_set_parameters_callback(_on_set_parameters)` + `_stop_video_input()`/`_configure_video_input()` 분리로 `ros2 param set` 시 스레드·구독 안전 정리 후 즉시 전환 | `base_tracker_node.py` (`c777d97`) |
| **⑨ 호스트 시계 오차 보정 & 최전방 LiDAR 클러스터링** | 로봇-PC1 간 시계 오차(`~+760ms`) 시 과거 스캔 매칭·`p25` 백분위수가 다리 ray(2~3개)를 건너뛰고 뒤쪽 벽면(`3.88~4.3m`) 선택 (`test_full` 동일ID LiDAR 튐 33회) | 동시 도착 wall-clock 기반 시계 오차 자동 추정·보정(`compensate_clock_offset`) + Pose 몸통 관절 기반 섹터 축소(`use_pose_sector`) + 최전방 유효 다중 ray 클러스터 선택(`front_valid`, ≥2 rays) 적용 (`test_full` 동일ID LiDAR 튐 33→5회) | `lidar_person_localizer.py`, `base_tracker_node.py` |

---

## 2. 파일 구조 변화 및 리팩터링 방향

- **모듈 역할 분리 (`src/person_tracker/person_tracker/`)**:
  - `base_tracker_node.py`: 공용 ROS 2 영상 입력(`video_source` / 토픽 구독)·`MultiModelTrackerCore`·`LidarPersonLocalizer`·오버레이·`TrackerStateStore` 훅 제공 (`run_tracker_node`)
  - `tracker_state.py`: ROS 비의존 선택적 상태 스냅샷(`TrackerState`) + 이벤트 콜백(`TrackerStateStore`)
  - `gesture_core.py`: 손 ROI, MediaPipe 21관절 분류, 얼굴 오검출 제외, 손 슬롯(`L`/`R`)별 `Fist → Open` 상태머신
  - `feature_extractors.py`: `BaseFeatureExtractor._prepare_part_masks()` / `extract_part_masks()` 통합 템플릿(Detect/Seg/Pose 중복 제거), 304D HSV(`18x16 H-S` + `16 V`), `backfill_part()`
  - `multi_tracker_core.py`: YOLO26n-pose + ByteTrack + Adaptive ReID(`COEX`/`SAFE`/`GUARD`/`SEARCH`) + `UPGRADE`/`BACKFILL`
  - `lidar_person_localizer.py`: `/scan` 링 버퍼 시간 정합 + `frame_stamp` 기반 TF 변환 + `auto_start_lidar_motor` 제어 + `point_base`/`point_map`/`marker` 발행
  - `reid_visualizer.py`: 120프레임 시계열 그래프, 부위 점수 막대, 행별 4부위 304D 히트맵
  - `gesture_test_node.py`: `BaseTrackerNode` 상속 후 손 슬롯별 제스처 등록 전략 + ReID 모니터 창 담당 (`arti_follow` `tracker:=gesture` 기본 노드)
  - `front_tracker_node.py` *(on `main`)*: `BaseTrackerNode` 상속 정면 자동/클릭 등록 노드 (`tracker:=front`)
