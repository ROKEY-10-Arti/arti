---
title: 도킹 중 호출(prepare_tour)이 거절되고, 2D Pose Estimate가 안 먹고, Nav2가 "Aborting bringup"으로 멈춤 (도킹 중 LiDAR 꺼짐)
date: 2026-11-20
author: flypig341
status: 해결
packages: [arti_pc2_nav, arti_bringup]
nodes: [a_course_node, amcl, lifecycle_manager_navigation, planner_server]
robot: robot4
---

## 증상

로봇을 도크에 붙인 채 launch를 띄우고 키오스크 호출(prepare_tour)을 보내면 세 가지가 번갈아 났다.

1. 호출이 언독 전에 거절됨
   ```
   Result: success: false
   message: AMCL map 위치 정보 없음 (RViz 2D Pose Estimate 필요)
   ```
2. RViz 2D Pose Estimate를 찍어도 AMCL이 위치를 안 냄 (`amcl_pose`, map→odom TF 없음)
3. Nav2가 켜지다 멈춤
   ```
   [planner_server]: Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform
   [lifecycle_manager_navigation]: Failed to change state for node: planner_server
   [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
   ```

## 재현 방법

```bash
# 로봇은 도크에 붙인 채
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=...
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot4 params_file:=...
ros2 action send_goal /robot4/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}"
```

## 원인

도킹 중에는 TurtleBot4 절전(power_saver)으로 **LiDAR 모터가 멈춘다** → `/robot4/scan`이 없다.
AMCL은 scan이 들어올 때마다 위치를 계산해 `amcl_pose`와 map→odom TF를 내므로, scan이 없으면 2D Pose Estimate를 찍어도 결과를 안 낸다.
map TF가 없으니 Nav2 global_costmap이 활성화를 못 끝내고 bond 시간 초과로 bringup을 포기한다.
코스 노드는 출발 전 점검에서 AMCL 위치·Nav2까지 요구해서 도킹 상태에서는 항상 거절했다.

## 해결

명세서 순서(호출 → 언독 → 만남 장소 이동) 그대로, **언독한 뒤에** 위치 잡기와 Nav2 켜기를 하도록 바꿨다. 코드에 `[코스 관람 변경 ②③④⑤]`로 표시.

- ② 언독 직후 `initialpose` 발행: robot4를 언독시켜 RViz로 맞춘 위치를 `arti_common/config/init_pose.yaml` `/robot4.undock: {x: -0.12, y: 0.03, yaw_deg: -179}`에 저장. launch(arti_bringup, `start_point:=undock`)와 코스 노드(`_send_initial_pose`)가 이 값을 쓴다 → 2D Pose Estimate 불필요
- ③ 점검 두 단계: 도킹 중엔 도크로 돌아올 수 있는지(도크 상태·배터리·odom·Dock 서버·음성)만 보고 언독, 언독 뒤 scan·AMCL·TF·Nav2가 3 s 연속 OK일 때 출발 (`_check_ready(before_undock=True)`, `_tick` (3))
- ④ 언독 후 60 s 안에 준비 안 되거나 그 사이 취소되면 Nav2 없이 Dock 액션으로 그 자리 재도킹 (`TourMachine.redock`)
- ⑤ Nav2 `autostart:=false`로 띄우고, scan·TF가 들어오면 코스 노드가 `lifecycle_manager_navigation/manage_nodes` STARTUP (`_activate_nav2`)

## 확인 방법

```
[a_course_node]: 언독 완료 → initialpose 발행 (구독자 3, map -0.12, 0.03, -179°)
[a_course_node]: scan·TF 들어옴 → Nav2 STARTUP 요청 (lifecycle_manager_navigation/manage_nodes)
[a_course_node]: Nav2 STARTUP 완료 (active)          ← 약 10 s
[a_course_node]: prepare_tour 끝: success (registered)
```

## 참고

- `src/arti_pc2_nav/README.md` "초기 대비 바뀐 점" ②~⑤
- [Discovery Server가 느릴 때](../env/2026-11-20-discovery-server-slow-new-nodes.md)
