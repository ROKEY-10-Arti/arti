# Arti 키오스크 개발 안내

키오스크는 PC3에서 관람객 화면과 관리자 화면을 제공합니다. Flask는 브라우저의 요청을 받아 SQLite에 관람 기록을 저장하고, ROS 2 Jazzy로 로봇을 호출합니다.

## 빠른 실행

아래 명령은 저장소의 `apps/kiosk` 디렉터리에서 실행합니다.

| 목적 | 실행 명령 | 접속 주소 |
| --- | --- | --- |
| 실제 로봇 연동 | `bash scripts/run-dev.sh` | `http://127.0.0.1:5000/` |
| 화면과 버튼 흐름 확인 | `python3 scripts/run-simulator.py` | `http://127.0.0.1:5003/`, 수동 신호 리모컨 `/remote` |

### 첫 실행 준비

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

실제 로봇과 연동할 PC에서는 ROS 2 Jazzy와 `colcon`을 설치한 뒤 공용 인터페이스를 빌드합니다. 이후 새 터미널에서는 가상환경을 활성화하고 실행 명령을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build-interfaces.sh
```

### 실기와 시뮬레이터 설정

- **실기** — 관제 PC의 Fast DDS Discovery Server를 먼저 실행합니다. `run-dev.sh`는 Jazzy, 빌드된 인터페이스, [중앙 ROS 접속 설정](config/central_ros.json)을 불러옵니다. 서버 실행 방법은 [팀 ROS 환경 설정](../../docs/common/setup.md)을 참고하세요.
- **Robot1 자유 관람** — PC1의 `PrepareTour`·`StartTour` 액션 서버를 사용합니다. [로봇 관람 실행 안내](../../docs/follow/README.md)
- **Robot4 코스 관람** — 실기 연동에는 PC2의 두 액션 서버가 필요합니다. 현재 [PC2 코스 노드](../../src/arti_pc2_nav/arti_pc2_nav/a_course_node.py)는 토픽으로 관람 요청을 받습니다.
- **시뮬레이터** — `/remote`에서 도착, 등록 완료 등의 신호를 보내고 별도 SQLite DB에 테스트 데이터를 저장합니다.
- **관리자** — 각 실행 모드의 `/data`에서 작품·로봇 상태·로그를 확인합니다. 첫 관리자 계정은 서버가 실행 중인 PC에서 `/admin/setup`을 열어 생성합니다. 시스템 상태 화면은 `/sysmon`입니다.

## 한눈에 보는 구조

```text
관람객·관리자 브라우저
  │ HTTP 요청 · SSE 상태 수신
  ▼
Flask (app.py)
  ├── SQLite (database.py, schema.sql)
  │     작품 카탈로그 · 관람 기록 · 운영 로그
  └── ROS 2 (robot_preparation.py, fleet.py)
        관람 액션 · 작품 설명 서비스 · 배터리/도킹 구독
        ↕ Fast DDS
      Robot1 / Robot4 및 각 로봇 PC
```

버튼 동작을 따라가려면 [static/js/app.js](static/js/app.js) → [static/js/services.js](static/js/services.js) → [app.py](app.py) → [robot_preparation.py](robot_preparation.py) 순서로 보면 됩니다.

## 폴더 구조

```text
apps/kiosk/
├── app.py          Flask 진입점과 관람 세션 API
├── index.html      관람객 화면 진입점
├── templates/      관리자 로그인·데이터·시스템 상태와 시뮬레이터 리모컨
├── static/
│   ├── js/          화면 제어, API 호출, 실시간 상태 구독
│   ├── css/         화면별 스타일
│   ├── art/         작품 및 로봇 상태 이미지
│   └── media/       이용 안내와 갤러리 소개 이미지
├── config/         중앙 ROS 접속 설정과 로봇 네임스페이스
├── data/           초기 작품·코스 카탈로그
├── schema.sql      SQLite 테이블 정의
├── scripts/        서버 실행, ROS 환경 로딩, 인터페이스 빌드, 시뮬레이터
├── ros2_ws/        공용 arti_interfaces를 빌드하는 로컬 워크스페이스
├── tests/          Python·JavaScript 테스트와 로컬 ROS 통합 테스트
└── instance/       실행 시 생성되는 DB와 앱별 설정 파일
```

### 파일별 역할

| 기능 | 파일 |
| --- | --- |
| 관람객 UI와 HTTP 요청 | [static/js/app.js](static/js/app.js), [static/js/services.js](static/js/services.js) |
| 준비 상태 수신과 관람 시작 후 화면 복귀 | [static/js/robot-preparation.js](static/js/robot-preparation.js), [static/js/visit-handoff.js](static/js/visit-handoff.js) |
| 코스 선택·작품 카탈로그·지도 | [course-selection.js](static/js/course-selection.js), [catalog-feed.js](static/js/catalog-feed.js), [gallery-map.js](static/js/gallery-map.js) |
| 관람 세션 API와 목록 조회 | [app.py](app.py), [session_query.py](session_query.py) |
| ROS 액션 클라이언트, 설명 서비스, 준비 상태 스트림 | [robot_preparation.py](robot_preparation.py) |
| 배터리·도킹 상태 수집과 가용 상태 계산 | [fleet.py](fleet.py), [robot_availability.py](robot_availability.py), [config/fleet.json](config/fleet.json) |
| 운영 데이터 초기화와 스키마 이전 | [database.py](database.py), [schema.sql](schema.sql), [data/catalog.json](data/catalog.json) |
| 관리자 인증·데이터 조회 및 수정 | [admin_auth.py](admin_auth.py), [managed_data.py](managed_data.py), [templates/data.html](templates/data.html) |
| 로봇 로그 수신 및 조회 | [robot_logs.py](robot_logs.py), [log_query.py](log_query.py) |
| 시스템 상태 화면과 지표 API | [system_monitor.py](system_monitor.py), [templates/system_monitor.html](templates/system_monitor.html) |
| 시뮬레이터와 수동 신호 | [signal_simulator.py](signal_simulator.py), [scripts/run-simulator.py](scripts/run-simulator.py) |

## 관람 버튼과 서버 흐름

1. **로봇 호출** — 브라우저가 `POST /api/sessions`로 관람 세션을 만들고 `POST /api/sessions/<id>/preparation`을 보냅니다. Flask가 관람 방식에 따라 로봇을 배정하고 `prepare_tour` Goal을 전송합니다.
2. **준비 상태 표시** — `GET /api/sessions/<id>/preparation/events`의 서버 전송 이벤트(SSE)로 최신 준비 상태를 받습니다. 준비 액션이 성공하면 **관람 시작하기** 버튼이 활성화됩니다.
3. **관람 시작하기** — 브라우저가 `PATCH /api/sessions/<id>`에 `{"status":"active"}`를 보냅니다. Flask가 준비 상태를 확인하고 `start_tour` Goal을 전송합니다.
4. **관람 진행·종료** — 완료한 작품과 관람 결과는 `TourSession`에 반영됩니다. 사용자가 진행 중인 작업을 취소하면 키오스크가 ROS 액션 취소를 요청합니다.

실제 Goal 전송 코드는 [robot_preparation.py](robot_preparation.py)의 `RosPreparationTransport.action()` 안에 있는 `send_goal_async()`입니다. 브라우저 Network 탭에서는 HTTP 요청을, ROS에서는 각 액션의 `/_action/status`와 `/_action/feedback` 토픽을 확인할 수 있습니다.

## ROS 연동

키오스크 ROS 노드는 `/arti` 네임스페이스에서 실행됩니다. 빌드 스크립트는 공용 `src/common/arti_interfaces`를 참조합니다.

| 인터페이스 | 방향 | 데이터 |
| --- | --- | --- |
| `/<robot>/arti/prepare_tour` (`PrepareTour`) | 키오스크 → 로봇 | `start: true`로 호출하고, 준비 액션이 성공한 뒤 관람 시작 버튼을 활성화합니다. |
| `/<robot>/arti/start_tour` (`StartTour`) | 키오스크 → 로봇 | 자유 관람 `/robot1`에는 `artwork_ids: []`, 코스 관람 `/robot4`에는 정수 작품 ID 목록을 보냅니다. |
| `/arti/description` (`Description`) | 로봇 → 키오스크 | 정수 작품 ID로 SQLite의 `Art` 테이블에서 설명을 조회합니다. |
| `/<robot>/battery_state`, `/<robot>/dock_status` | 로봇 → 키오스크 | 두 로봇의 배터리와 도킹 상태를 구독합니다. |

키오스크는 `ARTWORK_COMPLETED` 피드백의 작품 ID를 방문 기록에 저장합니다. 관람 종료는 액션 결과로 확인하고, 진행 중인 작업 취소는 ROS 2 Action Cancel을 사용합니다. 세션 ID와 관람객 ID는 키오스크 내부에서 관리하며, 작품 위치와 구역은 로봇 PC의 로컬 YAML에서 관리합니다.

## 연결이 안 될 때

키오스크를 실행한 채 별도 터미널에서 같은 ROS 설정을 불러옵니다. 아래 예시는 Robot1 자유 관람입니다.

```bash
source scripts/central-ros-env.bash
echo "$ROS_DOMAIN_ID" "$ROS_DISCOVERY_SERVER"
ros2 topic list --no-daemon --spin-time 5
ros2 action list
```

1. Discovery Server 주소가 관제 PC와 일치하는지 확인합니다.
2. `/robot1/battery_state`와 `/robot1/dock_status`가 보이는지 확인합니다.
3. 두 관람 액션이 보이면 버튼을 누르고 `/data`의 **로그 DB · Log**에서 Goal 전송과 수락 여부를 확인합니다. ROS CLI 목록이 이전 환경을 반영한다면 같은 터미널에서 `ros2 daemon stop` 후 목록을 다시 조회합니다.

## 운영 데이터와 로그

| 데이터 | 확인 위치 |
| --- | --- |
| 작품 설명 및 메타데이터 | SQLite `Art`, 관리자 화면 `/data` |
| 관람 시작·종료 시각, 상태, 방문 작품 | SQLite `TourSession`, 세션 API `/api/sessions` |
| 키오스크 관람 액션 로그와 HTTP로 수신한 로봇 로그 | SQLite `Log`, 관리자 화면 `/data`, 로그 API `/api/logs` |
| 현재 활동과 배터리·도킹 | 관리자 화면 `/data`의 실시간 로봇 상태 |

운영 DB는 `instance/arti.sqlite3`, 시뮬레이터 DB는 `instance/simulator/arti.sqlite3`입니다. 키오스크는 관람 액션의 Goal 전송·수락·피드백 변경·취소 요청·결과를 `Log`에 저장합니다. 대시보드의 현재 활동은 관람 액션 피드백으로 갱신합니다. 작품 설명 조회는 서비스 응답으로 처리하고, 설명 진행과 완료는 관람 액션 피드백으로 기록합니다.

```bash
sqlite3 -readonly -header -column instance/arti.sqlite3
```

## 검증

`apps/kiosk`에서 실행합니다.

```bash
python3 -m unittest tests.test_robot_preparation tests.test_central_ros
node --test tests/test_robot_preparation.mjs tests/test_visit_handoff_flow.mjs
```

[tests/ros_preparation_check.py](tests/ros_preparation_check.py)는 로컬 ROS 도메인에서 모의 Robot1·Robot4를 실행해 액션, 설명 서비스, 피드백, 종료와 취소를 검증합니다.

## 공동 개발

- ROS 액션·서비스 필드를 변경할 때는 저장소의 [공용 인터페이스](../../src/common/arti_interfaces)를 기준으로 PC3와 로봇 PC 구현을 함께 확인합니다.
- 운영 DB와 관리자 키가 들어 있는 `instance/`, `.venv/`, ROS 빌드 결과는 PC별로 생성되며 [Git에서 제외](.gitignore)합니다.

### 커밋 메시지 양식

`<유형>(kiosk): <변경 내용 한 줄>`

- 유형은 `feat`(기능 추가), `fix`(오류 수정), `docs`(문서), `test`(테스트) 등을 사용합니다.
- 예시 `docs(kiosk): 폴더 구조와 ROS 연동 흐름 정리`
