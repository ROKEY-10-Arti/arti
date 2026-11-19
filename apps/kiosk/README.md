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

키오스크는 `/robot1` 자유관람과 `/robot4` 코스관람에
`/<robot>/arti/prepare_tour` 및 `/<robot>/arti/start_tour`를 사용합니다.
빌드 스크립트는 저장소의 공용 `src/common/arti_interfaces`를 참조합니다.
로봇은 `/arti/description`에 정수 작품 ID를 보내 키오스크 DB의 설명을 조회합니다.

자유관람 로봇 실행은 [로봇 관람 안내](../../docs/follow/README.md)를 참고하세요.
코스관람 실기 연동에는 PC2의 동일한 액션 서버가 필요합니다.
현재 main의 `arti_pc2_nav` 코스 노드는 토픽으로 관람 요청을 받습니다.

UI만 확인하려면 `python3 scripts/run-simulator.py`를 실행하세요.
화면은 포트 5003, 수동 신호는 `/remote`에 있습니다.
