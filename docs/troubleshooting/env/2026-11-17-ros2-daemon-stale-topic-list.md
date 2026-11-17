---
title: ros2 topic list에 로봇 토픽이 안 나옴 (ros2 daemon이 예전 설정으로 떠 있음)
date: 2026-11-17
author: choongman.choi
status: 해결
packages: []
nodes: [ros2-daemon]
robot: robot1
---

## 증상

로봇(robot1)에 ping은 되는데 `ros2 topic list`에 로봇 토픽이 하나도 안 나온다.

```
$ ros2 topic list
/parameter_events
/rosout
```

## 재현 방법

```bash
# 네트워크 설정(env.bash, ROS_DISCOVERY_SERVER 등)이 바뀌기 전에 ros2 daemon이 이미 떠 있던 상태에서
ros2 topic list                # /parameter_events, /rosout 만 나옴
ros2 topic list --no-daemon    # /robot1/... 토픽이 정상으로 나옴
```

## 원인

`ros2 topic list`는 백그라운드 `ros2-daemon`에게 물어본다. daemon은 **처음 실행될 때의 환경변수**
(`ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT` 등)로 계속 떠 있으므로, 그 뒤에 터미널의 네트워크 설정이
바뀌어도 반영되지 않는다. `--no-daemon`으로는 보이고 기본 명령으로는 안 보이면 이 경우다.

같이 확인한 것: 이 PC는 `ARTI_PC_ROLE=host`인데 `scripts/run_discovery_server.sh`(ID 0)가 꺼져 있었다.
robot1 토픽은 로봇 쪽 discovery server(ID 1)로 보이므로 이번 증상의 원인은 아니지만,
Guest PC와 통신하려면 켜 둬야 한다.

## 해결

```bash
ros2 daemon stop      # 다음 ros2 명령 때 현재 터미널 환경으로 새로 뜬다
ros2 topic list       # 처음 한두 번은 디스커버리 중이라 비어 있을 수 있음 → 몇 초 뒤 다시
```

## 확인 방법

`ros2 topic list | grep -c robot1` → 43 (재시작 전 0).

## 참고

- `scripts/ros_network.bash`, `docs/common/setup.md`
- [env.bash의 ROS_DOMAIN_ID가 로봇 설정을 덮어씀](2026-11-16-domain-id-mismatch.md)
