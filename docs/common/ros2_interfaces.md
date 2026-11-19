# ArtI 프로젝트 — ROS 2 통신 명세서

> 원본: Notion "ROS2 통신 명세서" · 환경: ROS 2 Jazzy / Fast DDS Discovery Server

## 1. 시스템 구성

| 장비 | 담당 역할 | 네임스페이스 |
|---|---|---|
| PC3 | 키오스크 UI, 중앙 관제, SQLite DB | `/arti` |
| PC1 | Robot1 제어, 자유관람·관람객 추종 | `/robot1` |
| PC2 | Robot4 제어, 코스관람·자율주행 | `/robot4` |

**설계 원칙**
- PC3는 로봇 호출·관람 시작 명령을 전송하고 진행 상태를 수신한다.
- PC1·PC2는 로봇 이동, 관람객 등록, 관람 진행을 수행한다.
- 작품 위치·구역 정보는 각 로봇 PC의 로컬 YAML에서 관리한다.
- 작품 설명은 PC3의 SQLite DB에서 관리한다.
- `session_id`, `visitor_id`는 PC3 ↔ 로봇 PC 통신에 사용하지 않는다.
- 로봇 1대당 하나의 관람 작업만 수행한다.

## 2. 통신 인터페이스 (Action 2종 + Service 1종)

| 기능 | 인터페이스 | 방식 | 방향 |
|---|---|---|---|
| Robot1 호출·등록 | `/robot1/arti/prepare_tour` | Action | PC3 → PC1 |
| Robot4 호출·등록 | `/robot4/arti/prepare_tour` | Action | PC3 → PC2 |
| Robot1 자유관람 시작 | `/robot1/arti/start_tour` | Action | PC3 → PC1 |
| Robot4 코스관람 시작 | `/robot4/arti/start_tour` | Action | PC3 → PC2 |
| 작품 설명 조회 | `/arti/description` | Service | PC1·PC2 → PC3 |

Robot1·Robot4는 동일 Action 타입을 쓰고 네임스페이스로 구분한다.

## 3. 메시지 타입

### 3.1 PrepareTour.action
순서: 호출 → 언독 → 만남 장소 이동 → 관람객 인식·등록 → 준비 완료

| state | 의미 |
|---|---|
| UNDOCKING | 도킹 해제 중 |
| MOVING | 만남 장소 이동 중 |
| ARRIVED | 만남 장소 도착 |
| REGISTERING | 관람객 인식·등록 중 |
| REGISTERED | 관람객 등록 완료 |

완료 조건: 등록 완료 후 `success=true`. PC3는 성공 확인 후 관람 시작 버튼 활성화.

### 3.2 StartTour.action

| 관람 방식 | 대상 | artwork_ids |
|---|---|---|
| 자유관람 | Robot1 | `[]` |
| 코스관람 | Robot4 | `[1, 3, 4]` |

| state | 의미 |
|---|---|
| MOVING | 다음 작품으로 이동 중 |
| FOLLOW | 관람객 추종 중 |
| EXPLAINING | 작품 설명 중 |
| ARTWORK_COMPLETED | 작품 설명 완료 |
| RETURNING | 복귀 중 |

- 관련 작품이 없으면 `artwork_id = 0`.
- 작품 설명 완료 시 `ARTWORK_COMPLETED` + 해당 `artwork_id` 전달.
- 종료는 Result, 취소는 Action Cancel.

### 3.3 Description.srv
작품 ID는 ROS 통신에서 `int32`로 통일, PC3에서 필요 시 DB 문자열 ID로 변환.


## 4. 작품 위치 및 구역 정보

| 데이터 | 위치 | 용도 |
|---|---|---|
| gallery.yaml / gallery.pgm | PC1·PC2 | 지도 및 Localization |
| init_pose.yaml | PC1·PC2 | 로봇별 초기 위치 |
| artworks.yaml | PC1·PC2 | 작품 좌표, 구역 범위, 안내 구역 |
| SQLite Art | PC3 | 작품 설명·콘텐츠 |

- Nav2 주행 목표 좌표는 작품 구역 중심과 구분해 관리한다.
- 전시 배치 변경 시 PC1·PC2의 YAML을 동일하게 갱신한다.

## 5. 기존 로봇 상태 Topic (상시 수신용)

| Topic | 타입 | 용도 |
|---|---|---|
| `/robot{1,4}/battery_state` | sensor_msgs/BatteryState | 배터리 |
| `/robot{1,4}/dock_status` | irobot_create_msgs/DockStatus | 도킹 상태 |
| `/robot{1,4}/odom` | nav_msgs/Odometry | 이동 상태 |

## 6. 통합 구현 체크리스트
- [x] PC1: PrepareTour Action Server 구현 (언독·이동·관람객 등록까지, visitor_follower, 2026-11-18)
- [ ] PC2: PrepareTour Action Server 구현
- [x] PC1: StartTour에서 준비 단계 중복 실행 제거 (prepare_tour 성공 뒤에만 받음)
- [ ] PC3: 각 로봇 Action Client 연결
- [ ] PC3: `/arti/description` Service Server 구현
- [x] PC1: Description Service Client 구현 (arti_audio `guide_audio_node`, 2026-11-18)
- [ ] PC2: Description Service Client 구현
- [ ] 작품 ID 변환 규칙 통일
- [ ] 두 로봇의 지도·구역 YAML 버전 일치 확인
- [ ] 준비 실패·관람 실패·취소·중복 명령 처리 검증
