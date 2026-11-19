# `person_tracker` 인식·추적 모듈 개선 요약 (포트폴리오 / 발표 정리용)

> 대상 브랜치: `updating_branch` (최신 커밋 [`5e41dbb`](../../src/person_tracker/person_tracker/README.md))  
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

---

## 2. 파일 구조 변화 및 리팩터링 방향

- **모듈 역할 분리 (`src/person_tracker/person_tracker/`)**:
  - `gesture_core.py`: 손 ROI, MediaPipe 21관절 분류, 얼굴 오검출 제외, `Fist → Open` 상태머신
  - `feature_extractors.py`: `BaseFeatureExtractor._prepare_part_masks()` / `extract_part_masks()` 통합 템플릿(Detect/Seg/Pose 중복 제거), 304D HSV(`18x16 H-S` + `16 V`), `backfill_part()`
  - `multi_tracker_core.py`: YOLO26n-pose + ByteTrack + Adaptive ReID(`COEX`/`SAFE`/`GUARD`/`SEARCH`) + `UPGRADE`/`BACKFILL`
  - `lidar_person_localizer.py`: `/scan` 링 버퍼 시간 정합 + `frame_stamp` 기반 TF 변환 + `point_base`/`point_map`/`marker` 발행
  - `reid_visualizer.py`: 120프레임 시계열 그래프, 부위 점수 막대, 행별 4부위 304D 히트맵
  - `gesture_test_node.py`: 위 모듈을 통합한 ROS 2 실행 노드
- **향후 공용화 설계 (진행 예정)**:
  - `multi_tracker_node.py`와 `gesture_test_node.py`의 공통 ROS 2 입출력·시각화·위치추정 골격을 `BaseTrackerNode`로 추출하고, **순수 상태 객체(`TrackerState` / 스냅샷) + 콜백 훅**을 분리해 ROS 2 분산 실행과 Python `import` 직결 실행을 모두 지원하도록 정리 예정
