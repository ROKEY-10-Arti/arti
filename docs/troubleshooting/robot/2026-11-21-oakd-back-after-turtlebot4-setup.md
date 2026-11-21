---
title: robot1에서 꺼 둔 OAK-D가 다시 켜짐 (turtlebot4-setup이 launch 링크를 되돌림)
date: 2026-11-21
author: 이정욱
status: 해결
packages: [turtlebot4_bringup]
nodes: [oakd, oakd_container]
robot: robot1
---

## 증상

11/20 오전에 robot1이 OAK-D 없이 켜지게 바꿔 두었는데, 11/21에 보니 `/robot1/oakd` 노드가 다시 떠 있었다.

```
$ ros2 node list | grep oakd
/robot1/oakd
/robot1/oakd_container
$ ros2 topic info /robot1/oakd/rgb/image_raw/compressed -v
Publisher count: 1
Node name: oakd
```

## 재현 방법

```bash
# robot1에서
turtlebot4-setup        # 설정을 저장/적용하면
ls -l /etc/ros/jazzy/turtlebot4.d/
# standard.launch.py -> /opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py  (링크로 돌아감)
```

## 원인

11/20 방법: `/etc/ros/jazzy/turtlebot4.d/standard.launch.py`(원래 패키지 파일로 가는 링크)를
OAK-D 부분만 뺀 일반 복사본으로 바꿈.

같은 날 17:15에 robot1에서 `turtlebot4-setup`이 실행되었고(`~/.bash_history`), 이것이 `/etc` 쪽 시작 설정을
다시 만들면서 링크를 패키지 원본으로 되돌렸다. 그래서 다음 시작부터 OAK-D가 다시 켜졌다.

## 해결

`turtlebot4-setup`이 링크를 다시 만들어도 상관없게, 링크가 가리키는 **패키지 파일 자체**를 고쳤다.

- 파일: robot1 `/opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py`
- 바꾼 것: OAK-D를 30초 뒤 켜는 `TimerAction` 블록(원본 87–100줄)만 삭제. 나머지(robot, joy, rplidar, description, diagnostics)는 그대로.
- 원본 백업: robot1 `~/arti_backup/standard.launch.py.orig` (수정 전 파일과 `cmp`로 같음 확인)
- `/etc/ros/jazzy/turtlebot4.d/standard.launch.py`는 기본 상태(패키지 파일 링크) 그대로 둠.
- 적용: `sudo systemctl restart turtlebot4` (이전 노드가 늦게 꺼져 약 30초 걸림)

주의:
- `apt upgrade`로 `ros-jazzy-turtlebot4-bringup`이 업데이트되면 패키지 파일이 원본으로 돌아간다. 그때는 같은 블록을 다시 지운다.
- 되돌리기: `sudo cp ~/arti_backup/standard.launch.py.orig /opt/ros/jazzy/share/turtlebot4_bringup/launch/standard.launch.py && sudo systemctl restart turtlebot4`
- `oakd_rgb_camera_frame` TF는 URDF에서 나오므로 `top_cam_tf`는 영향 없음.

## 확인 방법

서비스 재시작 후 1분(원래 OAK-D 시작 시점 30초 지남) 뒤:

```
$ ros2 node list | grep -E "oakd|rplidar|turtlebot4_node"
/robot1/rplidar_composition
/robot1/turtlebot4_node
$ ros2 topic info /robot1/scan | grep Publisher
Publisher count: 1
```

`oakd` 노드 없음, LiDAR·turtlebot4_node 정상.

## 참고

- [2026-11-20 관람 중 로봇이 뚝뚝 멈춤 (scan 끊김)](../follow/2026-11-20-scan-gap-wifi-stutter.md) — OAK-D를 끈 이유(CPU·Wi-Fi 부담 줄이기)
