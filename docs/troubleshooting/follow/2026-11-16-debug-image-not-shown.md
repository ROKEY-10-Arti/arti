---
title: rqt_image_view에 visitor_follower 디버그 화면이 안 나옴
date: 2026-11-16
author: jsy0605
status: 해결
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

`ros2 run rqt_image_view rqt_image_view /robot1/follow/debug_image`로 열었는데 그림이 나오지 않았다.

## 재현 방법

```bash
ros2 run arti_follow visitor_follower --ros-args -r __ns:=/robot1 --params-file src/arti_follow/config/visitor_follower.yaml
ros2 run rqt_image_view rqt_image_view /robot1/follow/debug_image
```

## 원인

노드가 디버그 그림을 압축 토픽(`follow/debug_image/compressed`, CompressedImage)으로만 발행했고,
"구독자가 있을 때만 그린다"는 조건을 그 압축 토픽 기준으로 확인했다.
rqt_image_view는 토픽 이름만 주면 원본(raw) 전송 방식으로 `follow/debug_image`를 구독한다
→ 노드 입장에선 압축 토픽 구독자가 0명 → 그리지 않음.

## 해결

노드와 rqt가 같은 PC라 압축할 필요가 없으므로, `follow/debug_image`를 원본 `sensor_msgs/Image`(bgr8)로 발행하도록 바꿨다
(`src/arti_follow/arti_follow/visitor_follower.py`).

## 확인 방법

```bash
ros2 topic echo /robot1/follow/debug_image --once --field encoding   # bgr8
```

## 참고

- [2026-11-16-visitor-follower-wall-approach.md](2026-11-16-visitor-follower-wall-approach.md)
