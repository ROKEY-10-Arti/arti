---
title: 추종 중·SEARCH 때 로봇이 제자리에서 저절로 돌고 회전 속도가 들쑥날쑥, 빨리 걸으면 놓침
date: 2026-11-18
author: jsy0605
status: 확인 중
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

1. 관람객이 캠에서 벗어나면 최종 인식 좌표로 이동한 뒤 10초 회전해야 하는데, 그 자리에서 5초쯤 돌다가 멈춤
2. 회전 속도가 들쑥날쑥하고, 너무 빨리 돌 때는 돌다가 사람을 못 잡음
3. 천천히 걸으면 잘 따라오는데 조금 빨리 걸으면 못 따라오고, 어떤 때는 너무 가까이 다가옴

## 재현 방법

```bash
ros2 launch arti_follow tour.launch.py      # 2026-11-18 bag: ~/arti_bags/follow/20261118_{110919,112125,114122,115116}
```

## 원인 (bag 분석)

- **Create 3는 작은 제자리 회전 명령을 크게 돈다.** `cmd_vel`이 v=0, w=+0.010인데 odom은 0.25~0.31 rad/s로 계속 돎(bag 4개 공통, v=0이고 0<|w|<0.05인 구간의 실제 |w| 중앙값 0.25~0.31). w=0이면 바로 멈춤.
- **DWA가 그 작은 w를 만든다.** 각속도 후보가 `linspace(w0-0.3, w0+0.3, 15)`라 w0=0.01이면 정확히 0이 후보에 없다. 비용이 같으면 덜 도는 쪽을 고르니 0.01이 계속 뽑힌다 → 서 있어야 할 때 0.25 rad/s로 돌다가 탑캠이 사람을 놓친다(놓침 24번 중 4번).
- 0.6 rad/s 넘게 빨리 돌 때도 탑캠 인식을 놓친다(24번 중 5번). `max_angular` 1.5, `angular_accel` 3.0.
- SEARCH 이동: 도착 거리 0.5 m는 놓칠 때 관람객과의 거리(0.8 m 안팎)와 비슷해 거의 안 움직이고 회전으로 넘어감. 이동 시간 상한 0(무제한)이라 사람이 그 자리에 서 있으면 영원히 못 도착할 수 있음.
- `lead_time` 0.5 s: 기록된 좌표로 칼만 필터를 다시 돌려 보니 0.5 s 앞 예측 오차 p90 0.68~0.74 m(0.3 s면 0.53~0.58 m). 멈출 때 지난 속도로 앞을 잡아 더 다가간다. 감속 0.4 m/s^2면 0.3 m/s에서 11 cm 더 감.
- 관람객 속도 0.4~0.6 m/s > TurtleBot 4 최고 약 0.3 m/s → 빨리 걸으면 물리적으로 거리가 벌어짐.
- 122~131 s처럼 오래 서 있던 구간은 작품 설명 중(`artwork_zone/state: explaining`) → 정상.

## 해결

`src/arti_follow/arti_follow/visitor_follower.py` (주행 알고리즘은 그대로):
- DWA 각속도 후보에 정확히 0 추가, `drive()`에서 v≈0이고 |w| < `min_spin_cmd`(0.1)면 w=0
- SEARCH 회전: 2π/`search_time`(0.63 rad/s) 일정 속도로 odom 기준 360° 돌면 끝(`search_spin_extra` 3 s 여유), 그다음 `wait_time`
- 파라미터: `lost_timeout` 3.0→1.5, `search_move_time` 0→5.0, `search_arrive` 0.5→0.35, `max_angular` 1.5→0.8, `angular_accel` 3.0→1.5,
  `linear_accel` 0.5→0.8, `linear_decel` 0.4→0.8, `lead_time` 0.5→0.3, `kf_accel_std` 1.0→2.0, `kf_meas_std` 0.15→0.2

## 확인 방법

- 오프라인 시뮬레이션(ROS_DOMAIN_ID 99, 가짜 위치): 놓침 1.5 s 뒤 SEARCH → 0.628 rad/s로 약 10 s 한 바퀴 → WAIT 10 s → LOST
- 로봇 테스트 후 bag에서: SEARCH 중 odom |w| ≈ 0.63 일정, v=0이면서 0<|w|<0.1인 `cmd_vel` 없음, 거리 0.65 m 미만 비율 (이전 114122: 29/180)

## 참고

- 후진하지 않으므로 관람객이 로봇 쪽으로 다가오면 0.8 m보다 가까워지는 것은 남는다
