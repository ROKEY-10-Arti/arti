---
title: Nav2가 장애물을 거의 피해 가지 않고 앞에서 멈칫하거나 멈춤
date: 2026-11-20
author: flypig341
status: 미해결
packages: [arti_pc2_nav]
nodes: [controller_server, planner_server, collision_monitor]
robot: robot4
---

## 증상

코스 관람 중 경로 앞에 장애물(사람·물건)이 있으면 옆으로 비켜 가지 않고 다가가다 멈추거나 멈칫멈칫한다.

```
[collision_monitor]: Robot to stop due to invalid source. Either due to data not published yet, or to lack of new data received within the sensor timeout, or if impossible to transform data to base frame
```
(한 번 주행에 19번)

## 원인 (설정 분석, `config/nav2_real.yaml`)

장애물 **인식**은 정상 (costmap의 scan drop 0번). 회피를 덜 하게 설정돼 있다.

1. DWB critic 가중치: `PathDist 32, PathAlign 32, GoalDist 24, GoalAlign 24` vs `BaseObstacle 0.02` → 부딪히는 궤적만 버리고 경로를 고집 (Nav2·TurtleBot4 기본값)
2. `inflation_radius 0.22` (로봇 반지름 0.189) + `cost_scaling_factor 8.0` → 장애물에서 3 cm만 비용 → 경로가 장애물 바로 옆·틈으로 지나감
3. global costmap 갱신 1 Hz, BT 재계획 1 Hz → 새 장애물 반영이 1~2 s 늦음
4. collision_monitor는 감속·정지만 하고, scan이 `source_timeout 1.0` s 넘게 늦으면(와이파이) 멈춤
5. LiDAR 한 평면만 봄 (OAK-D 꺼짐), local costmap 3×3 m

## 해결

아직 적용 안 함. 후보 (한 번에 하나씩 실주행 확인, 작품 4 반원 r=0.43에 들어갈 수 있는지도 확인):

| 항목 | 지금 | 후보 |
|---|---|---|
| `inflation_radius` (local·global) | 0.22 | 0.35~0.45 |
| `cost_scaling_factor` | 8.0 | 3~4 |
| global `update_frequency` | 1.0 | 3~5 |
| `BaseObstacle.scale` | 0.02 | 조금씩 올리기 |
| collision_monitor `source_timeout` | 1.0 | 1.5~2.0 |

## 확인 방법

RViz에서 local/global costmap과 경로를 띄우고 장애물을 놓아 경로가 돌아가는지 본다.
