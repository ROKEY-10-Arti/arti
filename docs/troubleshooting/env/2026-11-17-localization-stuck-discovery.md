---
title: localization이 "Configuring amcl"에서 멈춤 (Discovery Server에 붙지 못해 노드끼리 못 찾음)
date: 2026-11-17
author: jsy0605
status: 해결
packages: []
nodes: [amcl, map_server, lifecycle_manager_localization]
robot: robot1
---

## 증상

gallery 지도로 localization을 띄웠는데 AMCL이 활성화되지 않음. map_server는 지도를 정상으로 읽음.

```
[robot1.lifecycle_manager_localization]: Configuring amcl
[WARN] [robot1.amcl.rclcpp]: failed to send response to /robot1/amcl/change_state (timeout): client will not receive response, at ./src/rmw_response.cpp:153, at ./src/rcl/service.c:400
```

`ros2 node list`(daemon 끄고 `--no-daemon`) 결과가 비어 있음 — 같은 PC에서 띄운 map_server·amcl도 안 보임. `ros2 topic list`에 `/robot1/...` 없음.

## 재현 방법

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$HOME/arti/maps/gallery.yaml
ros2 daemon stop
ros2 node list --no-daemon      # 비어 있음
```

## 원인

Discovery Server 모드에서는 같은 PC 노드끼리도 서버를 거쳐야 서로를 찾는다.
이 PC의 `ROS_DISCOVERY_SERVER`는 `192.168.107.54:11811;192.168.107.101:11811;` (관제 PC = Host PC `192.168.107.54` = 서버 0, robot1 `192.168.107.101` = 서버 1)인데 둘 다 응답이 없는 것으로 보임.

확인한 것:
- 이 PC IP: `192.168.107.11` (같은 날 앞서는 `192.168.108.11` — 다른 대역에 붙어 있던 적 있음)
- `192.168.107.54`(Host PC): ping 됨. 서버가 실제로 떠 있는지는 미확인
- `192.168.107.101`(robot1): ARP는 REACHABLE인데 ping 응답 없음
- 이 PC에서도 `fast-discovery-server --server-id 0`이 떠 있었음 → 아무도 이 PC(.11)를 서버로 쓰지 않으므로 쓸모없음. Host PC는 .42
- 저장소에 `env.bash`가 없었음 → `ARTI_ROBOT_NS` 비어 있음 (`ROS_DISCOVERY_SERVER`는 `.bashrc`의 `/etc/turtlebot4_discovery/setup.bash` 값)

## 해결

robot1 쪽이 다시 응답하면서 해결 (아래 마지막 항목).
- `env.bash`를 guest로 만듦: `ARTI_PC_ROLE=guest`, `ARTI_HOST_PC_IP=192.168.107.54`, `ARTI_DISCOVERY_SERVERS="1:192.168.107.101"`
  → `ROS_DISCOVERY_SERVER=192.168.107.54:11811;192.168.107.101:11811;`, `ARTI_ROBOT_NS=/robot1`
- 이 PC의 discovery server 끄기 (해당 터미널에서 Ctrl+C)
- 남은 것: robot1이 켜져 있고 응답하는지 확인

추가 확인 (같은 날):
- 셸에 예전 값(`127.0.0.1:11811;192.168.107.101:11811;;;192.168.107.54:11811;` — 이 PC가 서버 0, .42가 4번 칸)이 남아 있어서
  다른 PC에서 테스트 토픽 `/arti_weird_banana`를 구독해도 `Subscription count: 0`이었음
- `env.bash`를 다시 source해서 `192.168.107.54:11811;192.168.107.101:11811;`로 맞춘 뒤 `ros2 topic list --no-daemon --spin-time 8`:
  `/arti_weird_banana`와 `/robot4/...`가 보임 → **서버 0(.42) 연결 OK**. `/robot1/...`은 0개 → robot1 쪽은 아직
- 최종: `env.bash`를 `/etc/turtlebot4_discovery/setup.bash`와 같은 값으로 맞춤 → guest, `192.168.107.54:11811;192.168.107.101:11811;`
- 최종 값으로 다시 확인: `/robot4/...` 88개 보임(서버 0 OK), `/robot1/...` 0개.
  robot1(.101)은 ARP REACHABLE(MAC `d8:3a:dd:b8:70:35`, Raspberry Pi)인데 ping·22번 포트 모두 응답 없음 → 로봇 쪽(전원/부팅/IP 변경) 확인 필요
- `--no-daemon`은 기본 대기 시간이 짧아서 Discovery Server 모드에서는 결과가 비어 보일 수 있다. `--spin-time 5` 이상 줄 것
- 이후 robot1(.101) ping·22번 포트 응답 복구. `ROS_DISCOVERY_SERVER=192.168.107.54:11811;192.168.107.101:11811;` 그대로
  `ros2 topic list --no-daemon --spin-time 10` → `/robot1/...` 43개(`scan`, `tf`, `odom`, `cmd_vel`, `oakd/...`), `/robot4/...` 106개
- 주의: 셸에서 `unset ROS_DISCOVERY_SERVER`(또는 `ROS_SUPER_CLIENT`)를 하면 오히려 robot1이 안 보인다.
  로봇이 Discovery Server 모드라 멀티캐스트로는 못 찾는다. 값을 바꾼 뒤에는 `ros2 daemon stop`으로 daemon도 새로 띄울 것

## 확인 방법

```bash
ros2 daemon stop
ros2 node list          # 이 PC의 map_server, amcl이 보이면 서버 0 연결 OK
ros2 topic pub -r 1 /arti_weird_banana std_msgs/msg/String "{data: hi}"   # 다른 PC에서 echo로 받으면 PC 간 OK
ros2 topic list | grep robot1   # /robot1/scan, /robot1/tf 가 보이면 로봇 연결 OK
```

## 참고

- `scripts/ros_network.bash`, `env.bash.example`
- [env.bash의 ROS_DOMAIN_ID가 로봇 설정을 덮어씀](2026-11-16-domain-id-mismatch.md)
