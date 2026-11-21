---
title: 언독했는데 키오스크로 안 가고 60 s 뒤 재도킹 (AMCL이 scan을 전부 버림 "earlier than all the data in the transform cache")
date: 2026-11-20
author: flypig341
status: 미해결
packages: [arti_pc2_nav, arti_bringup]
nodes: [amcl, a_course_node]
robot: robot4
---

## 증상

호출 → 언독 → `initialpose` 발행까지는 됐는데 Nav2 STARTUP 요청이 안 나오고, 60 s 뒤 재도킹했다.

```
[a_course_node]: 언독 완료 → initialpose 발행 (구독자 2, map -0.12, 0.03, -179°)
[a_course_node]: {"state": "REDOCK", "detail": "not_ready: AMCL,TF,Nav2; 키오스크로 가지 않고 바로 재도킹"}
[amcl]: Message Filter dropping message: frame 'rplidar_link' at time 1795145149.001 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

- AMCL 로그에 위 메시지가 **활성화 직후 첫 scan부터** 계속(95번) 나오고, `createLaserObject`가 한 번도 없음 → scan을 한 번도 못 씀
- 같은 시각 `/robot4/scan`, `/robot4/tf`(odom→base_link)는 노트북에 잘 들어옴 (지연 0.01 s / 0.04~0.8 s), 로봇·노트북 시계도 맞음
- 정상이던 launch(같은 날 12:06)에서는 이 메시지 0번

## 재현 방법

확실한 재현 방법 모름. 서버 42만 쓰는 상태에서 launch를 여러 번 다시 띄우던 중 한 번 남.

## 원인

모름. AMCL의 TF 저장소(cache)에 scan 시각보다 **나중** 데이터만 있는 상태로 시작한 것으로 보인다.
의심: 서버 경유로 TF가 몰려서 늦게 오거나, 시각이 튀는 TF가 한 번 들어와 cache가 꼬임.

## 해결

우회: launch를 다시 띄움. 액션을 보내기 전에 확인한다.

```bash
grep -c "Message Filter dropping" ~/.ros/log/$(ls -t ~/.ros/log | grep amcl_ | head -1)   # 계속 늘면 다시 launch
```

## 확인 방법

```bash
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot4/tf -r /tf_static:=/robot4/tf_static
# 언독 뒤 값이 나오면 AMCL 정상. 'Invalid frame ID "map"'이 계속이면 이 문제
```

## 참고

- [Discovery Server가 느릴 때](../env/2026-11-20-discovery-server-slow-new-nodes.md)
