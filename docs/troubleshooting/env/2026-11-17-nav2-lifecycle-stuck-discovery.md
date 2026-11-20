---
title: nav2가 활성화되지 않음 (lifecycle_manager가 controller_server 서비스를 오래 못 찾고, configure 응답이 사라짐)
date: 2026-11-17
author: jsy0605
status: 확인 중
packages: [turtlebot4_navigation]
nodes: [lifecycle_manager_navigation, controller_server, planner_server]
robot: robot1
---

## 증상

`ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1` 후 활성화가 끝나지 않음.

```
[robot1.lifecycle_manager_navigation]: Waiting for service controller_server/get_state...   (약 80초 반복)
[robot1.lifecycle_manager_navigation]: Configuring planner_server                            (여기서 멈춤)
[WARN] [robot1.planner_server.rclcpp]: failed to send response to /robot1/planner_server/change_state (timeout): client will not receive response
```

planner_server 자체는 configure를 끝냈는데(`Planner Server has GridBased planners available`) 응답이 lifecycle_manager에 가지 않음.
그 사이 `local_costmap`에서 `Invalid frame ID "odom" ... frame does not exist`도 보임 (robot1 Create3 노드가 보였다 사라짐).

## 재현 방법

```bash
export ROS_DISCOVERY_SERVER=";192.168.107.101:11811;"   # robot1 서버에만 붙음
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
```
robot1 ping이 87~303 ms로 튀던 때 (공유기 ping 4~67 ms, Host PC ping 44~382 ms, PC 신호 -48 dBm).

## 원인

Discovery Server 모드에서는 같은 PC 노드끼리도 서버를 거쳐 서로를 찾는다. 서버가 robot1(Wi-Fi 너머)이라
Wi-Fi가 느리면 서비스 endpoint 매칭이 늦어지고, 서비스 응답을 보낼 때 아직 상대 reader와 연결이 안 돼서 응답이 버려진다.
turtlebot4 nav2.launch.py는 `use_composition:=False` 고정이라 노드 12개가 각자 participant → 매칭할 것이 많다.

측정 (같은 PC에서 talker → `ros2 topic echo --once`, 처음 받을 때까지):

| discovery | 걸린 시간 |
|---|---|
| robot1 서버만 (`;192.168.107.101:11811;`) | 40초 넘게 못 받음 (timeout) |
| Host PC 서버만 (`192.168.107.54:11811;`) | 20.3초 |
| 서버 없음 (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) | 1.3초 |

→ 같은 PC 노드끼리도 Wi-Fi 너머 서버를 거치느라 수십 초 걸림. robot1 서버가 특히 느림.

## 해결

미해결. Nav2는 기존 `~/turtlebot4_ws`의 turtlebot4_navigation으로 띄우기로 함 (저장소 쪽 launch 추가는 되돌림).
- 근본 원인은 Wi-Fi 지연 → 로봇 쪽(`wlan0` 송신량, 신호, 다른 PC의 robot1 RViz) 확인 필요
- 참고: 노드를 한 프로세스(component container, `use_composition:=True`)에 모으면 Nav2 내부 연결은 서버를 안 거친다.
  turtlebot4 nav2.launch.py는 이 값이 False로 고정

## 확인 방법

```bash
ros2 lifecycle get /robot1/controller_server   # active [3]
ros2 lifecycle get /robot1/bt_navigator        # active [3]
```
lifecycle_manager 로그에 `Managed nodes are active`.

## 참고

- [localization이 "Configuring amcl"에서 멈춤](2026-11-17-localization-stuck-discovery.md) — 같은 `failed to send response ... (timeout)`
- Wi-Fi 지연 자체는 로봇 쪽(`wlan0` 송신량, 신호, 다른 PC의 robot1 RViz) 확인 필요
