---
title: robot4 OAK-D 끄기 (웹캠만 쓰는데 OAK-D가 돌아 토픽·부하만 늘어남)
date: 2026-11-20
author: flypig341
status: 해결
packages: [turtlebot4_bringup]
nodes: [oakd, oakd_container]
robot: robot4
---

## 증상

코드를 실행하면 ping이 크게 오르고 discovery가 느려졌다. robot4는 사람 인식에 노트북 웹캠만 쓰는데, 로봇 OAK-D가 부팅 30 s 뒤 켜져 `/robot4/oakd/...` 토픽 수십 개를 낸다.

## 원인

TurtleBot4 기본 bringup(`standard.launch.py`)이 OAK-D를 항상 켠다. 로봇 부팅은 `turtlebot4.service` → `/usr/sbin/turtlebot4-start` → `/etc/ros/jazzy/turtlebot4.d`로 만든 launch → `standard.launch.py`.

## 해결

로봇에 SSH로 들어가 OAK-D를 켜는 `TimerAction` 블록만 주석 처리 (LiDAR·바퀴·조이스틱은 그대로).

```bash
ssh ubuntu@192.168.107.104
cp /opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py ~/standard.launch.py.bak    # 백업
sudo nano /opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py
#   "# Delay the OAK-D startup" 아래 TimerAction( ... oakd_launch_file ... ), 블록 각 줄 앞에 #
python3 -c "import ast; ast.parse(open('/opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py').read()); print('OK')"
#   (py_compile은 __pycache__ 쓰기 권한 때문에 Permission denied → ast.parse로 문법만 확인)
sudo systemctl restart turtlebot4      # 로봇 노드 1~2분 끊김
```

- 되돌리기: `sudo cp ~/standard.launch.py.bak /opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py`
- `sudo apt upgrade`로 `turtlebot4_bringup`이 업데이트되면 원래대로 돌아감 → 업데이트 뒤 다시 확인
- 파일을 안 건드리고 잠깐만 끄려면: `ros2 service call /robot4/oakd/stop_camera std_srvs/srv/Trigger "{}"` (재시작하면 다시 켜짐)

## 확인 방법

```bash
ros2 daemon stop
ros2 node list | grep robot4     # rplidar_composition·turtlebot4_node는 있고 oakd, oakd_container는 없음
```
노드가 안 보이면 robot4 서버로 직접 확인: `ROS_DISCOVERY_SERVER=";;;;192.168.107.104:11811;" ros2 node list --no-daemon`
