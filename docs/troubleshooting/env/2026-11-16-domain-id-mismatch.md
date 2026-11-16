---
title: env.bash의 ROS_DOMAIN_ID가 로봇 설정을 덮어써서 로봇 토픽이 안 보임
date: 2026-11-16
author: choongman.choi
status: 해결
packages: []
nodes: []
robot: 공통
---

## 증상

`ws arti`(또는 direnv로 `env.bash` 로드) 후 로봇 토픽이 보이지 않을 수 있다.
에러 메시지는 따로 없고 `ros2 topic list`에 로봇 토픽이 빠진다.

## 재현 방법

```bash
echo $ROS_DOMAIN_ID      # 1  (로봇 discovery 설정 값)
ws arti
echo $ROS_DOMAIN_ID      # 0  ← env.bash가 덮어씀
ros2 topic list          # /robot1/... 토픽이 안 보임
```

## 원인

- `/etc/turtlebot4_discovery/setup.bash`는 로봇과 통신하도록 `ROS_DOMAIN_ID=1`, `ROS_DISCOVERY_SERVER`를 설정한다.
- `env.bash.example`은 자리표시자로 `ROS_DOMAIN_ID=0`을 갖고 있다.
- 이 파일을 그대로 복사한 `env.bash`가 나중에 source되면서 값이 0으로 바뀐다.
- 도메인 ID가 다르면 DDS가 서로를 찾지 못하므로 노드와 토픽이 보이지 않는다.

## 해결

`env.bash`에서 `ROS_DOMAIN_ID`를 로봇과 같은 값으로 맞춘다.
아니면 그 줄을 주석 처리해서 discovery 설정 값을 그대로 쓴다.

```bash
nano env.bash    # export ROS_DOMAIN_ID=1
```

팀 공통 값을 1로 정하고 `env.bash.example`도 1로 고쳤다 (2026-11-16). 그 전에 `env.bash`를 복사한 사람은 직접 1로 바꿔야 한다.

## 확인 방법

```bash
echo $ROS_DOMAIN_ID      # 1
ros2 topic list | grep robot1
```

## 참고

- `/etc/turtlebot4_discovery/setup.bash` (alias `vd`로 확인)
