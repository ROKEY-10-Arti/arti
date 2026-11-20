---
title: ros2 topic list에 /parameter_events, /rosout만 나옴 (다른 Wi-Fi에 연결됨)
date: 2026-11-18
author: choongman.choi
status: 해결
packages: []
nodes: []
robot: robot1
---

## 증상

`ros2 topic list`를 하면 로봇·관제 PC 토픽이 하나도 안 나오고 이 PC 것만 보인다.

```
/parameter_events
/rosout
```

ROS 환경 변수는 정상이었다.

```
ROS_DOMAIN_ID=1
ROS_DISCOVERY_SERVER=192.168.107.54:11811;192.168.107.101:11811;
```

## 재현 방법

```bash
# 로봇 네트워크(192.168.107.x)가 아닌 다른 Wi-Fi(DT_Campus_1)에 연결된 상태에서
ros2 topic list
```

## 원인

노트북이 다른 Wi-Fi(`DT_Campus_1`, IP `192.168.123.90`)에 자동으로 연결돼 있었다.
Discovery Server(관제 PC `192.168.107.54`)와 robot1(`192.168.107.101`)에 ping도 안 가서 discovery가 전혀 안 됐다.

## 해결

1. Wi-Fi를 로봇 네트워크(192.168.107.x)로 바꿨다.
2. 데몬을 재시작했다. 데몬에 예전 discovery 상태가 남아 있기 때문이다.
   ```bash
   ros2 daemon stop && ros2 daemon start
   ```

## 확인 방법

```bash
hostname -I                    # 192.168.107.x 인지
ping -c2 192.168.107.54        # 관제 PC
ping -c2 192.168.107.101       # robot1
ros2 topic list                # /robot1/..., /arti/... 토픽이 보임
```

참고: 재연결한 뒤 IP는 `.11`이 아니라 `192.168.107.151`로 DHCP 할당됐다. guest PC라서 지금 구성에서는 문제없다.

## 참고

- [ros2 daemon이 예전 설정으로 떠 있음](2026-11-17-ros2-daemon-stale-topic-list.md)
