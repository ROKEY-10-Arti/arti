---
title: 웹캠 인식 창이 까맣고 사람 좌표가 /robot4로 나옴 (인식 모듈 네임스페이스)
date: 2026-11-16
author: jsy0605
status: 해결
packages: [person_tracker, arti_follow]
nodes: [multi_person_tracker_node, top_camera_publisher_node, visitor_follower]
robot: robot1
---

## 증상

1. `person_tracker` bringup으로 웹캠 인식을 띄웠는데 `visitor_follower`가 `IDLE`에서 안 움직임.
   `/robot1/target_person/point_map` 발행자 0개.
2. 인식 노드를 /robot1로 맞춘 뒤에는 인식 창(웹캠)이 까만 화면으로만 나옴.

```
$ ros2 node info /multi_person_tracker_node
  Subscribers:
    /robot4/scan, /robot4/tf, /top_cam/image_raw/compressed
  Publishers:
    /robot4/target_person/point_map
```

## 재현 방법

```bash
ros2 launch person_tracker bringup.launch.py namespace:=/robot1 use_localization:=false run_init_pose:=false video_source:=0
ros2 topic info /robot1/target_person/point_map   # Publisher count: 0
```

## 원인

- `bringup.launch.py`는 `multi_tracker_node`를 **네임스페이스 없이** 띄운다. 이때 노드는 `config/phone_camera.yaml`의
  `namespace`로 토픽 이름을 정한다.
- (1) 소스 yaml은 `/robot1`로 고쳐져 있었지만 다시 빌드하지 않아 `install/` 쪽 yaml은 `/robot4` 그대로였다.
- (2) 영상 토픽은 `env_bootstrap.resolve_image_topic()`이 yaml을 `~/turtlebot4_ws/src/...` 또는 모듈 옆 `config/`에서 찾는데,
  설치본에서는 둘 다 없어서 네임스페이스 없는 `/top_cam/image_raw/compressed`를 구독했다.
  카메라는 `/robot1/top_cam/image_raw/compressed`로 보내므로 영상이 한 장도 안 와서 대기(까만) 화면만 나왔다.
  웹캠 자체는 정상 (`/dev/video0` 640x480, 밝기 평균 약 125).

## 해결

- `colcon build --packages-select person_tracker`로 설치본 yaml을 `/robot1`로 맞춤.
- `src/arti_follow/launch/follow_test.launch.py`에서 bringup을 거치지 않고 카메라 launch + `multi_tracker_node`를
  **`namespace=/robot1`로 직접** 띄움 → 영상·scan·tf·좌표 토픽이 모두 `/robot1/...`.

## 확인 방법

```
$ ros2 node info /robot1/multi_person_tracker_node
  Subscribers: /robot1/scan, /robot1/tf, /robot1/tf_static, /robot1/top_cam/image_raw/compressed
$ ros2 topic hz /robot1/top_cam/image_raw/compressed
average rate: 14.982
```
실제 테스트(2026-11-16 21:19)에서 `target_person/point_map` 15 Hz 수신, 약 93 s FOLLOW.

## 참고

- 인식 모듈 가상환경의 numpy 2.x 때문에 `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` 경고와
  ImportError traceback이 찍히지만 카메라·인식 노드는 정상 동작함 (README는 `numpy<2` 요구, 인식 담당과 정리 필요).
