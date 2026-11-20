---
title: prepare_tour가 dock_status_unavailable로 실패 (같은 때 /arti/description도 못 받음, Discovery 일시 끊김 의심)
date: 2026-11-20
author: 이정욱
status: 미해결
packages: [arti_follow, arti_audio]
nodes: [visitor_follower, guide_audio_node]
robot: robot1
---

## 증상

`follow_test.launch.py`를 켜고 `/robot1/arti/prepare_tour`를 보냈더니 undock 전에 실패했다.
16:40, 16:41 두 번 연속. 16:20 실행에서는 정상 undock 됐다.

```
[ERROR] [robot1.visitor_follower]: dock_status를 받지 못함 (3 s) → undock 요청을 보내지 않음 (ros2 topic echo /robot1/dock_status --once 확인)
[ERROR] [robot1.visitor_follower]: prepare_tour 실패 → result success=false, message='dock_status_unavailable', goal.abort() / state=READY
```

노드는 호출 약 30초 전부터 켜져 있었는데도 `dock_status`를 한 번도 받지 못했다.
같은 실행에서 `guide_audio_node`도 PC3 서비스를 못 받았다.

```
[WARN] [robot1.guide_audio_node]: 작품 설명 못 받음 (1: no_description) → 미리 합성 건너뜀
```

## 재현 방법

```bash
ros2 launch arti_follow follow_test.launch.py
ros2 action send_goal /robot1/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}" --feedback
```

다시 해 보니 재현되지 않았다 (아래 확인 방법).

## 원인

확정하지 못함. 정황:

- 코드·QoS 문제는 아니다. 퍼블리셔 `create3_repub`는 RELIABLE, 구독(`qos_profile_sensor_data`)은 BEST_EFFORT라 호환된다.
  16:20 실행과 노드 코드도 같다.
- 실패한 두 실행에서는 로봇에서 오는 토픽(`dock_status`)과 PC3에서 오는 서비스(`/arti/description`)가 **둘 다** 안 들어왔다.
  둘 다 Discovery Server(PC3 `192.168.107.42`)를 거치는 PC 간 통신이다.
  → 그 시각 Discovery Server 재시작, PC3 쪽 문제, Wi-Fi 끊김 중 하나로 의심.

## 해결

없음 (조금 뒤 저절로 정상). 다시 나면 launch를 다시 켜기 전에 아래를 먼저 확인한다.

## 확인 방법

조금 뒤 (16:4x) 이 PC에서 확인한 결과, 모두 정상:

```bash
ros2 topic info -v /robot1/dock_status                  # Publisher count: 1 (create3_repub)
ros2 topic echo /robot1/dock_status --field header.stamp.sec   # 약 1 Hz로 계속 나옴
ros2 service type /arti/description                     # arti_interfaces/srv/Description
ping -c2 192.168.107.42                                 # 응답 있음
```

ROS_SUPER_CLIENT=False(노드와 같은 조건)로 BEST_EFFORT 구독해도 8초에 8개 받음.

다시 나면 그 순간에 위 명령을 실행해서, 로봇 토픽과 PC3 서비스가 둘 다 안 보이는지 확인하고
PC3에서 `scripts/run_discovery_server.sh`가 계속 떠 있었는지 물어본다.

## 참고

- [localization이 "Configuring amcl"에서 멈춤 (Discovery Server 연결 안 됨)](2026-11-17-localization-stuck-discovery.md)
- 로그: `~/.ros/log/python_179989_1795160501753.log` (visitor_follower), `python_179990_*.log` (guide_audio_node)
