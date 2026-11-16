# arti

TurtleBot 4 두 대로 갤러리 관람객을 안내하는 큐레이터 로봇 (ROKEY 10기 지능1 팀 프로젝트). 기능 개요는 `README.md`, 개발용 요구사항(수치·제약·기능→패키지 매핑)은 `docs/common/requirements.md` 참고. 원본 기획은 팀 노션.

## 진행 방식

- 사용자는 로보틱스를 배우는 중이다. 데이터가 어떻게 오가는지 이해하는 것이 목표다.
- 명령어는 한 번에 하나씩, 짧은 설명과 함께. 새 토픽/서비스/액션은 `ros2 topic info/echo`, `ros2 interface show`, `rqt_graph`로 직접 확인하게 한다.
- 팀 저장소다. `main`에 직접 push하지 않고 브랜치 → PR로 올린다. push/PR은 사용자 확인 후에.
- 로봇(TurtleBot 4) SSH 접속은 사용자가 그 작업에 명시적으로 허락할 때만. 이동 명령은 보내지 않는다.
- OAK-D RGB는 raw 말고 `/compressed` 토픽 사용. 사람 인식용 탑CAM은 로봇 위 휴대폰 카메라.
- 관람객 영상은 저장하지 않는다(개인정보 요구사항). 카메라 토픽 rosbag 녹화는 팀 합의 후에만.
- 로봇·지도·작품 위치·관람 구역은 코드에 하드코딩하지 말고 YAML 파라미터로 둔다(확장성 요구사항).
- 답변은 한국어, 수식은 LaTeX 없이 평문/코드 블록.

## 트러블슈팅 기록

- 에러를 해결하면 (또는 해결하지 못해도) `docs/troubleshooting/<폴더>/YYYY-MM-DD-slug.md`에 남긴다. 양식은 `TEMPLATE.md`, 폴더 기준과 목록은 `docs/troubleshooting/README.md`.
- 기록을 추가하면 README 목록에도 한 줄을 추가한다.

## 환경

- Ubuntu 24.04, ROS 2 Jazzy. `~/turtlebot4_ws`는 `.bashrc`에서 이미 source됨.
- 로봇 네임스페이스: `/robot1`, `/robot4` (`env.bash`의 `ARTI_ROBOT1_NS`, `ARTI_ROBOT2_NS`).
- 저장소 루트(`~/project/arti`) = colcon 워크스페이스 루트. `pb`로 빌드.
- Python venv: `~/venvs/arti_venv` (`scripts/make_venv.sh`, `--system-site-packages`). pip 의존성은 `requirements.txt`에 버전 고정. pytest·colcon은 pip로 설치하지 않는다(ROS와 충돌).
- `.envrc`(direnv, 커밋됨)가 들어올 때 `env.bash` → arti venv → `install/setup.bash`를 자동 로드한다. `ws arti`도 같은 효과. 팀원 세팅 절차는 `docs/common/setup.md`, 주행 띄우기는 `docs/common/navigation.md`.
- `env.bash`는 로컬 전용(gitignore). `env.bash.example`을 복사해서 `ROS_DOMAIN_ID`, 로봇 네임스페이스 설정.

## 패키지 구성

역할(노션 간트 차트)별 폴더. 두 역할 이상이 쓰는 것은 `src/common/`. 규칙은 `src/README.md`.

| 폴더 | 패키지 | 빌드 | 역할 | 상태 |
|---|---|---|---|---|
| common | arti_interfaces | ament_cmake | 커스텀 msg/srv/action (역할 간 데이터는 여기로만) | 빈 패키지 |
| common | arti_bringup | ament_python | 로봇별 launch·파라미터 (localization, nav2, /robot1·/robot4) | 빈 패키지 |
| common | arti_common | ament_python | 공용 지도 `maps/rokey_map`, 공용 좌표 config, 유틸 | 지도 포함 |
| follow | arti_follow | ament_python | 사용자 추종: 등록·재식별, 위치 추정, 80cm 추종, 분실 처리 | 빈 패키지 |

예정 역할 폴더 (필요해지면 추가): `course/`(코스주행, 현재 다른 담당), `docent/`(작품 해설), `fleet/`(배정·통행 순서·도킹·비상 정지), `marking/`(AprilTag 작품 마킹), 키오스크/관제 UI(Figma 설계 중).
역할 패키지끼리 직접 import하지 않는다.
문서도 같은 구분: `docs/common/`, `docs/follow/` (`docs/README.md`). 역할 설계(노드·토픽·파라미터)가 정해지면 해당 역할 README 표에 적는다.

## 미정

- 팀 `ROS_DOMAIN_ID` (현재 로봇 discovery 설정은 1, `env.bash.example`은 0 → `docs/troubleshooting/env/2026-11-16-domain-id-mismatch.md`)
- package.xml maintainer (현재 자리표시자 `arti@todo.invalid`)
