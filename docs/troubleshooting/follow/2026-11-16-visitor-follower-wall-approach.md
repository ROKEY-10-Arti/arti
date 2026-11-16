---
title: visitor_follower 주행 테스트 중 로봇이 앞 벽 쪽으로 감
date: 2026-11-16
author: jsy0605
status: 미해결
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

가짜 인식(`ros2 topic pub`, 탑캠 정면 고정 박스)과 `match_angle:=35.0`, `max_linear:=0.15`로 `visitor_follower`를 띄우고
사람이 앞에서 걸었는데, 로봇이 앞에 있는 벽 쪽으로 갔다.

노드 로그 (`~/.ros/log/python3_80866_1794816217256.log`, 일부):

```
[INFO] [robot1.visitor_follower]: 대상 잠금: x 0.93, y -0.17 m
[INFO] [robot1.visitor_follower]: [FOLLOW] 대상 x 0.83, y +0.04 (거리 0.83 m, +3°) 후보 1명 → v 0.00, w +0.07
[INFO] [robot1.visitor_follower]: [FOLLOW] 대상 x 1.13, y +0.05 (거리 1.14 m, +3°) 후보 4명 → v 0.15, w +0.07
[INFO] [robot1.visitor_follower]: 상태 FOLLOW → TURN 후보 2명이 비슷함
[INFO] [robot1.visitor_follower]: [FOLLOW] 대상 x 0.95, y -0.42 (거리 1.04 m, -24°) 후보 2명 → v 0.14, w -0.63
[WARN] [robot1.visitor_follower]: depth 끊김 (0.5 s) → 정지
[INFO] [robot1.visitor_follower]: 상태 FOLLOW → TURN 직전 위치에서 너무 멂 (끼어든 사람?)
[INFO] [robot1.visitor_follower]: [FOLLOW] 대상 x 0.69, y +0.02 (거리 0.69 m, +1°) 후보 2명 → v 0.00, w +0.04
[INFO] [robot1.visitor_follower]: [FOLLOW] 대상 x 0.66, y -0.01 (거리 0.66 m, -1°) 후보 2명 → v 0.00, w -0.02
```

## 재현 방법

```bash
ros2 topic pub -r 10 /robot1/visitor/detection arti_interfaces/msg/VisitorDetection "{found: true, id: 1, confidence: 0.9, center_x: 640.0, center_y: 360.0, width: 400.0, height: 700.0, image_width: 1280, image_height: 720}"
ros2 run arti_follow visitor_follower --ros-args -r __ns:=/robot1 --params-file src/arti_follow/config/visitor_follower.yaml -p match_angle:=35.0 -p max_linear:=0.15
```

## 원인

확정 못 함. 로그·depth 화면에서 확인한 사실과 의심:

- 노드가 낸 전진 속도는 최대 0.15 m/s였고, 노드가 도는 동안(…217~…254 s) Nav2는 목표가 없었다.
  Nav2 목표(bt_navigator "Begin navigating")는 노드 종료 약 40초 뒤(…294 s~)부터 들어왔다 — 그때 움직임이었다면 Nav2 쪽
- 사람 한 명인데 후보가 2~4명으로 잡히고, 대상 좌표가 프레임마다 좌우로 0.2~0.4 m씩 튐
  → depth가 잡음이 많고 빈 곳이 많아(같은 날 depth 화면 확인) **벽·바닥 조각이 다리 폭(5~70 cm) 후보로 잡힌 것**으로 의심
- 가짜 박스는 항상 정면이고 `match_angle` 35°라 depth 화각 전체가 매칭 범위 → 벽 조각도 대상이 될 수 있음
- 안전 정지(앞 0.4 m)는 depth 점만 본다. 가까운 무늬 없는 벽은 depth 값이 안 나와서 안전 정지가 동작하지 않을 수 있다
- depth가 2~3초마다 0.5 s 넘게 끊김 (Wi-Fi) → 정지/출발 반복

## 해결

아직 없음. 다음 테스트에서 `rqt_image_view /robot1/follow/debug_image`로 초록 선(대상)이 다리에 있는지, 벽·바닥에 있는지 확인한다.
후보 대책: LiDAR(`/scan`)로 안전 정지, 다리 후보 조건 강화(`min_points` 증가), 실제 인식 연동 후 `match_angle` 8°로 재시험.

대책 적용 (같은 날, 노드를 새로 작성): 사람 좌표를 depth 대신 **LiDAR**로 찾고, 주행은 LiDAR + depth 장애물을 보고
**돌아서 가는 회피 주행(DWA + 격자 경로)**으로 바꿨다. 무늬 없는 벽도 LiDAR가 정확히 잡으므로 다리 오인·충돌 원인이 사라질 것으로 본다.
가상 LiDAR 시험만 통과했고 실제 주행 확인 전이라 상태는 미해결로 둔다.
이후 좌표 파악은 인식 모듈로 넘어갔고(`visitor/pose`), 이 노드는 받은 좌표로 주행만 한다. 다리 오인 문제는 인식 모듈 쪽 몫이 됐다.

## 확인 방법

미해결.

## 참고

- 같은 날 화면이 안 뜬 문제: [2026-11-16-debug-image-not-shown.md](2026-11-16-debug-image-not-shown.md)
