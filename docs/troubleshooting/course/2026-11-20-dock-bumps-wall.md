---
title: 관람 끝나고 도킹 접근점에서 Dock 시작 2.7 s 만에 실패, 도크 앞 벽에 부딪힘
date: 2026-11-20
author: flypig341
status: 해결
packages: [arti_pc2_nav]
nodes: [a_course_node]
robot: robot4
---

## 증상

```
[a_course_node]: {"state": "HOME_ZONE_ENTERED", "detail": "Nav2 목표 취소 요청"}
[a_course_node]: {"state": "HOME_ARRIVED", "detail": "충전 스테이션 접근 위치 도착; Dock 액션 실행"}
[a_course_node]: {"state": "FAILED", "detail": "Dock 액션 실패/거절/취소"}       ← 2.7 s 뒤, 그 자리 정지
```
Nav2로 접근점까지는 왔고, Create 3 Dock 동작 중에 앞 벽에 범퍼가 닿았다.

## 원인

접근점(`dock.approach_pose`) **반경 0.30 m 원에 들어서자마자** 멈춘다(11/18부터 있던 동작, Nav2 미세 조정 실패 방지용).
출구(북쪽)에서 내려오면 로봇이 남쪽·비스듬히 서서 **도크가 정면에 없다**. Create 3 Dock은 앞쪽 IR로 도크 신호를 보며 다가가는데,
정면에 없으면 찾다가 앞 벽에 부딪혀 hazard로 Dock을 중단한다. `approach_pose.yaw: 0`은 원에서 바로 멈추므로 적용되지 않았다.

## 해결

`[코스 관람 변경 ⑨]`: 정지 확인 뒤 `FACE_DOCK` 상태에서 **정면이 충전 스테이션(`dock.station`)을 보게** Nav2 spin → Dock.
이미 보고 있으면(20° 이하) 바로 Dock, 회전 실패·20 s 초과여도 Dock 시도. `dock_face:=false`로 끔.
같이 고침: 노드의 `tick` 상태 목록에 `FACE_KIOSK`가 빠져 있어 회전 시간 초과(20 s)가 동작하지 않던 것.
(`course_engine.py` `_dock_turn`, `_start_docking`, `spin_result`, 테스트 `test_face_dock_before_docking`)

## 확인 방법

```
[a_course_node]: {"state": "HOME_ARRIVED", "detail": "충전 스테이션 접근 위치 도착; 도크 쪽으로 9x° 회전 후 Dock"}
[a_course_node]: {"state": "DOCKED", ...}
```
