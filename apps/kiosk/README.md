# Arti
PC3용 ROS 2 Jazzy 키오스크 및 중앙 관제 웹앱입니다. Flask와 SQLite를 사용합니다.
로봇 네임스페이스는 config/fleet.json, Discovery 서버 설정은 config/central_ros.json에 있습니다.

## 실행
저장소의 `apps/kiosk` 디렉터리에서 실행합니다.
```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
bash scripts/build-interfaces.sh
bash scripts/run-dev.sh
```

키오스크는 http://127.0.0.1:5000/ , 관리자 화면은 /data 입니다.
계정과 운영 DB는 각 PC의 instance 디렉터리에 생성됩니다.

## ROS 연동

키오스크 ROS 노드는 `/arti` 네임스페이스에서 실행됩니다. 빌드 스크립트는 공용 `src/common/arti_interfaces`를 참조합니다.

| 인터페이스 | 방향 | 데이터 |
| --- | --- | --- |
| `/<robot>/arti/prepare_tour` (`PrepareTour`) | 키오스크 → 로봇 | `start: true`로 호출하고, 준비 액션이 성공한 뒤 관람 시작 버튼을 활성화합니다. |
| `/<robot>/arti/start_tour` (`StartTour`) | 키오스크 → 로봇 | 자유관람 `/robot1`에는 `artwork_ids: []`, 코스관람 `/robot4`에는 정수 작품 ID 목록을 보냅니다. |
| `/arti/description` (`Description`) | 로봇 → 키오스크 | 정수 작품 ID로 SQLite의 `Art` 테이블에서 설명을 조회합니다. |
| `/<robot>/battery_state`, `/<robot>/dock_status`, `/<robot>/odom` | 로봇 → 키오스크 | 두 로봇의 배터리, 도킹, 이동 상태를 구독합니다. |

키오스크는 `ARTWORK_COMPLETED` 피드백의 작품 ID를 방문 기록에 저장합니다. 관람 종료는 액션 결과로 확인하고, 진행 중인 작업 취소는 ROS 2 Action Cancel을 사용합니다. 세션 ID와 관람객 ID는 키오스크 내부에서 관리하며, 작품 위치와 구역은 로봇 PC의 로컬 YAML에서 관리합니다.

자유관람 로봇 실행은 [로봇 관람 안내](../../docs/follow/README.md)를 참고하세요.
코스관람 실기 연동에는 PC2의 동일한 액션 서버가 필요합니다.
현재 저장소의 `arti_pc2_nav` 코스 노드는 토픽으로 관람 요청을 받습니다.

UI만 확인하려면 `python3 scripts/run-simulator.py`를 실행하세요.
화면은 포트 5003, 수동 신호는 `/remote`에 있습니다.
