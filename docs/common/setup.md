# 개발 환경 세팅 (팀원용)

Ubuntu 24.04 + ROS 2 Jazzy + TurtleBot 4 패키지가 설치된 PC를 기준으로 한다.
위에서부터 한 단계씩 진행하고, 단계마다 **확인** 명령으로 결과를 본다.

## 1. `~/.bashrc`에 추가

`~/.bashrc` 맨 아래에 붙인다. 이미 비슷한 줄이 있으면 중복으로 넣지 않는다.

```bash
# --- ROS 2 Jazzy + TurtleBot 4 ---
source /opt/ros/jazzy/setup.bash
[ -f ~/turtlebot4_ws/install/setup.bash ] && source ~/turtlebot4_ws/install/setup.bash

# --- 로봇과 통신 (ROS_DOMAIN_ID, Discovery Server) ---
# 파일 내용은 `cat /etc/turtlebot4_discovery/setup.bash`로 확인
[ -f /etc/turtlebot4_discovery/setup.bash ] && source /etc/turtlebot4_discovery/setup.bash

# --- direnv: 프로젝트 폴더에 들어가면 환경 자동 로드 ---
eval "$(direnv hook bash)"    # 반드시 다른 설정보다 뒤에 둔다

# --- 편의 명령 ---
alias pb='colcon build --symlink-install && source install/setup.bash'   # 워크스페이스 루트에서 빌드
alias ros-restart='ros2 daemon stop; ros2 daemon start'                   # 토픽이 안 보일 때 먼저 시도
```

적용과 확인:

```bash
source ~/.bashrc
echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION    # 예: 1 rmw_fastrtps_cpp
```

## 2. direnv 설치

```bash
sudo apt install direnv
direnv version
```

## 3. 저장소 받기

```bash
mkdir -p ~/project && cd ~/project
git clone https://github.com/ROKEY-10-Arti/arti.git
cd arti
```

## 4. 로컬 환경변수 파일 만들기

`env.bash`는 사람마다 다를 수 있으므로 git에 올리지 않는다.

```bash
cp env.bash.example env.bash
nano env.bash
```

- `ROS_DOMAIN_ID`는 팀 공통 값 **1**이다 (로봇 discovery 설정과 같음). 예시 파일 값 그대로 두면 된다.
  다르면 로봇 토픽이 보이지 않는다 → [기록](../troubleshooting/env/2026-11-16-domain-id-mismatch.md)
- `ARTI_DISCOVERY_SERVERS`에 로봇 discovery server를 `서버ID:IP`로 적는다. robot4 IP가 정해지면 추가한다.
  저장소 폴더에서는 이 값으로 `ROS_DISCOVERY_SERVER`, `RMW_IMPLEMENTATION`, `ROS_SUPER_CLIENT`가 자동 설정된다 (`scripts/ros_network.bash`).
  값을 바꾼 뒤에는 `ros2 daemon stop; ros2 daemon start` (= `ros-restart`)를 한 번 해야 반영된다.
- 로봇 네임스페이스는 `/robot1`, `/robot4`다 (`ARTI_ROBOT1_NS`, `ARTI_ROBOT2_NS`). 예시 파일 값 그대로 두면 된다.

## 5. arti 전용 venv 만들기

```bash
bash scripts/make_venv.sh      # ~/venvs/arti_venv 생성 + requirements.txt 설치
```

- 시스템 패키지(ROS, rclpy)를 볼 수 있게 `--system-site-packages`로 만든다.
- pip로 받을 패키지는 `requirements.txt`에 버전을 고정해서 추가하고, 스크립트를 다시 실행한다.
- **pytest, colcon은 pip로 설치하지 않는다** (ROS와 충돌 → [기록](../troubleshooting/env/2026-11-16-colcon-test-pytest-venv.md)).
- 위치를 바꾸려면 `env.bash`에 `export ARTI_VENV=...`를 넣는다.

## 6. direnv 허용

저장소에 들어 있는 `.envrc`가 `env.bash` → arti venv → `install/setup.bash` 순서로 자동으로 불러온다.
보안을 위해 처음 한 번, 그리고 `.envrc`가 바뀔 때마다 허용해야 한다.

```bash
direnv allow
```

## 7. 빌드와 확인

```bash
pb
cd ~ && cd ~/project/arti           # 다시 들어오면 direnv가 환경을 다시 불러온다
echo $ROS_DOMAIN_ID $ARTI_ROBOT1_NS
echo $ROS_DISCOVERY_SERVER            # 예: ;192.168.107.101:11811;
which python3                       # ~/venvs/arti_venv/bin/python3
ros2 pkg list | grep arti            # arti_bringup, arti_interfaces 가 보이면 성공
```

## 문제가 생기면

- `direnv: error .envrc is blocked` → `direnv allow`
- 로봇 토픽이 안 보임 → `ROS_DOMAIN_ID` 확인 → `ros-restart`
- 해결한 내용은 [`troubleshooting/`](../troubleshooting/README.md)에 기록한다.
