---
title: 사람 좌표는 오는데 로봇이 안 움직임 (localization 꺼져서 map→odom 없음)
date: 2026-11-16
author: jsy0605
status: 해결
packages: [arti_follow]
nodes: [visitor_follower, amcl, map_server]
robot: robot1
---

## 증상

웹캠 인식으로 사람을 클릭했는데 로봇이 안 움직임. `follow/state`가 계속 `IDLE`.
테스트 로그(`~/arti_logs/follow/20261116_211235/`):

```
[INFO] [입력] target_person/point_map: 15.0 Hz, 간격 최대 0.08 s ...
[INFO] [출력] cmd_vel 간격 최대 0.102 s, 이유별 주기 수 {'idle': 20}
[WARN] [TF] 그 시각 값 없음(fallback)/실패 횟수 {'tf_fail:map': 30, ...}
```

## 재현 방법

```bash
# localization을 끈 상태에서
ros2 launch arti_follow follow_test.launch.py
```

## 원인

- 인식 모듈은 사람 좌표를 `map` 기준으로 주고, `visitor_follower`는 `map→odom` TF로 odom 좌표로 바꿔 쓴다.
  `map→odom`은 AMCL이 발행한다.
- 19:31에 켜 둔 localization(`map_server`, `amcl`, `lifecycle_manager`)이 21:04:54에 Ctrl+C(exit code -2)로 같이 꺼졌고
  다시 켜지 않았다 (터미널 정리하다 같이 꺼진 것으로 보임). → 좌표를 받을 때마다 변환 실패 → 버림 → IDLE.
- localization을 다시 켠 뒤에도 **초기 위치(2D Pose Estimate)를 주기 전에는** AMCL이 `map→odom`을 내지 않는다.

## 해결

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$HOME/arti/maps/gallery.yaml
# RViz에서 2D Pose Estimate로 초기 위치 지정
ros2 topic echo --once /robot1/amcl_pose
ros2 launch arti_follow follow_test.launch.py
```
테스트 중에는 localization 터미널을 끄지 않는다.

## 확인 방법

`ros2 lifecycle get /robot1/amcl` → `active`, `amcl_pose` 수신 후 테스트(`20261116_211907/`)에서
FOLLOW 933주기(약 93 s), 최대 v 0.30 m/s.

## 참고

- 테스트 로그의 `[TF] ... tf_fail:map` 이 보이면 이 경우다.
- 관련: [2026-11-16-tracker-namespace-black-webcam.md](2026-11-16-tracker-namespace-black-webcam.md)
