---
title: 관람객이 화면에 보이는데 놓침·제자리 회전 (인식 좌표가 벽 점으로 튐) → LiDAR 다리 후보로 이어 감
date: 2026-11-20
author: 이정욱
status: 우회
packages: [arti_follow, person_tracker]
nodes: [visitor_follower, gesture_test_node]
robot: robot1
---

## 증상

관람객이 탑캠 인식 창에 보이는데도 로봇이 `FOLLOW → SEARCH`로 넘어가 제자리에서 빙빙 돈다.
관람객이 가까이 오거나 벽·가운데 벽 모서리 근처에 있을 때 특히 잦다. visitor_follower 로그:

```
[WARN] [robot1.visitor_follower]: 관람객 좌표 버림: 벽 안·지도 밖 (-1.49, 3.69), 상태 SEARCH
[INFO] [robot1.visitor_follower]: 관람객 놓침 → 마지막 위치까지 이동 (1.42 m)
```

같은 순간 인식 노드는 타겟을 계속 내고 있었다 (ID 유지, 화면에 사람 있음):

```
[타겟 위치 ID=95 | LiDAR (+6ms)] base: (+0.36,-0.17)m (0.40m) | map: X=-1.49m, Y=+3.69m [AMCL]
```

19:27 실행 한 번에 놓침 5번, 추적 ID 8번 바뀜, 19:48 실행에서 벽 좌표 버림 40번 넘게.

## 재현 방법

```bash
ros2 launch arti_bringup localization.launch.py
ros2 launch arti_follow tour.launch.py
# prepare_tour → start_tour 뒤 관람객이 로봇 앞 0.5 m까지 다가가거나 벽에 붙어 선다
```

## 원인

- 인식 노드(`person_tracker` `lidar_person_localizer.py`)는 사람 박스 방향(`hfov_deg` 54°로 계산한 각도) 범위에서
  **가장 앞쪽 LiDAR 묶음**을 사람 거리로 쓴다. 그 묶음이 벽인지 사람인지는 보지 않는다.
- 버려진 좌표 9개를 지도에서 확인하니 모두 벽 선 위(벽 속 5~10 cm) = 라이다가 벽에 맞은 점.
- 방향이 어긋나는 원인으로 의심: 2026-11-20에 노트북(탑캠)을 로봇 위로 옮겼는데 `phone_camera.yaml`의
  위치·화각(x 0.05, z 0.275, yaw 0, hfov 54°)은 그대로. 노트북 웹캠(MSI Pulse 16 AI 내장) 실제 화각은 아직 안 잼.
- visitor_follower는 벽 속 좌표를 버리므로(정상) 좌표가 하나도 안 남아 lost_timeout 뒤 SEARCH.

## 해결 (우회)

visitor_follower가 LiDAR로 사람(다리) 후보를 직접 찾아, 카메라 좌표가 끊긴 동안 이어서 따라간다
(누가 관람객인지는 여전히 카메라 재식별만 정한다). `src/arti_follow/arti_follow/motion.py`, `visitor_follower.py`,
파라미터는 `config/visitor_follower.json`의 `lidar_people` 묶음.

1. `find_people()` (leg detector와 같은 원리): scan을 묶음으로 나눔(이웃 점 10 cm) → 폭 3~25 cm →
   양옆 ray보다 앞으로 튀어나옴 → 다리 둘이 45 cm 안이면 사람 하나
2. 지도 벽에서 15 cm 안 후보 버림 (`person_map_clear`), 같은 자리에 3 s 넘게 있는 후보(모서리·받침대)는
   고정 물체로 배워서 뺌 (`StaticClutter`, 관람객 1 m 안에서는 배우지 않음)
3. FOLLOW 중 카메라 좌표가 0.3 s 넘게 안 오면 칼만 예측 0.5 m 안의 후보로 위치 갱신, 카메라가 3 s 넘게 없으면 그만
   (`lidar_assist_*`, 후보 둘이 비슷하게 가까우면 안 씀)
4. 추종 중 관람객 본인의 다리 점은 장애물에서 뺌 (`person_obstacle_clear` 0.3 m)
5. RViz: `/robot1/follow/lidar_people` (Marker, odom 초록 공)

같은 날 함께 바꾼 것 (놓침 줄이기): `lost_timeout` 1.5 → 2.5 s(예측은 1 s까지), 추종 중 사람을 화면 안에 두는 벌점
(`w_view`), 놓친 직후 가던 쪽으로 먼저 돌아보기(`search_look_*`), 재식별 기준(`follow_test.launch.py` `REID_SCALE`).
커밋 462aee3 (main) + 모서리 필터는 그 뒤 커밋.

## 확인 방법

- 가짜 scan 시험 (노드 밖): 벽 + 다리 둘 + 큰 물체 → 사람 후보 하나만 (처음엔 다리 사이로 보이는 벽 조각도 나와서
  '양옆보다 앞' 조건 추가). 모서리 1개 + 걷는 사람 + 서 있는 관람객 → 모서리만 약 3~4 s 뒤부터 빠짐.
- 2026-11-20 저녁 로봇 시험 (21:23 실행): 사용자 확인 "leg detector 확실히 효과가 있었다".
  벽 좌표가 계속 버려지는 중에도 `관람객 놓침` 뒤 1~3 s 안에 `SEARCH → FOLLOW`로 돌아오는 경우가 많았고,
  관람 끝까지 LOST 없이 출구 → 도크 복귀까지 감.

## 남은 문제

- 근본 원인(인식 노드가 벽을 고름)은 그대로: 웹캠 화각·위치를 재서 `phone_camera.yaml`에 넣기,
  인식 노드에서 지도 벽·다리 모양 후보를 쓰도록 하기 (person_tracker 담당과 상의).
- 가운데 벽 위쪽 끝(약 (-1.45, 3.95)) 근처에서 SEARCH 이동이 막힘 (`3 s 동안 경로 거리 안 줄어듦`). 놓친 자리가 벽에
  붙어 있어 로봇 중심이 갈 수 없는 곳(금지선 0.30 m 안)이 목표가 됨.

## 참고

- [같은 날: 라이다 scan이 Wi-Fi로 끊겨 뚝뚝 멈춤](2026-11-20-scan-gap-wifi-stutter.md)
- 로그 (로컬): `~/.ros/log/python_28987_1795170461496.log`(19:27), `python_35365_1795171686170.log`(19:48),
  `python_66842_1795177427411.log`(21:23)
