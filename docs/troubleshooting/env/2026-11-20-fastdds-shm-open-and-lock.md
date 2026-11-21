---
title: "[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7020: open_and_lock_file failed"
date: 2026-11-20
author: flypig341
status: 해결
packages: []
nodes: []
robot: 해당없음
---

## 증상

`ros2 action send_goal` 등을 실행하면 맨 앞에 에러가 찍힌다. 명령 자체는 계속 진행된다.

```
2026-11-20 12:13:08.746 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7020: open_and_lock_file failed -> Function open_port_internal
```

## 원인

launch·노드를 Ctrl+C·강제 종료를 여러 번 하면서 Fast DDS 공유 메모리 파일이 `/dev/shm`에 남았다 (이날 234개). 같은 PC 통신은 UDP로 대신하므로 동작에는 문제없다.

## 해결

ROS 프로그램을 모두 끈 뒤:
```bash
fastdds shm clean
```

## 확인 방법

`ls /dev/shm | grep -c fastrtps` 개수가 줄고 에러가 안 나옴.
