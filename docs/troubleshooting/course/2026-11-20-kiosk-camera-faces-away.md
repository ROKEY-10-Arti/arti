---
title: 키오스크 앞에서 웹캠이 관람객 반대쪽을 봄 (반원 진입 즉시 정지, 뒤쪽 웹캠) / 고정 각도로 돌렸더니 반만 돎
date: 2026-11-20
author: flypig341
status: 해결
packages: [arti_pc2_nav]
nodes: [a_course_node]
robot: robot4
---

## 증상

키오스크에 도착하면 로봇이 키오스크를 정면으로 보고 섰다. robot4 웹캠은 뒤를 보므로(`camera_yaw:=180`) 관람객이 잘 안 잡혔다.
처음 고친 버전(고정 목표 방향 `kiosk.goal_pose.yaw` 91°로 회전)에서는 반만 돌았다.

```
[a_course_node]: {"state": "KIOSK_FACING", "detail": "-85° 회전 (웹캠을 키오스크 쪽으로)"}
[behavior_server]: Turning -1.48 for spin behavior.
```

## 원인

- 반원에 **들어서자마자** Nav2를 취소하고 멈추므로 `goal_pose.yaw`까지 돌지 않는다. 로봇은 들어오던 방향(키오스크 쪽)을 보고 선다.
- 고정 방향 91°는 로봇이 Nav2 목표점(반원 위쪽 끝)에 섰을 때 기준이다. 실제로는 반원 **옆(동쪽)**에서 멈춰서, 그 자리에서 북쪽을 봐도 등은 키오스크가 아니라 남쪽 벽을 본다.

## 해결

`[코스 관람 변경 ⑦]`: 정지 확인 뒤 `FACE_KIOSK` 상태에서 **멈춘 자리 기준으로 키오스크(반원 중심)의 반대쪽**을 보게 Nav2 spin. 등(웹캠)이 항상 키오스크를 본다.
20° 이하 차이면 안 돌고, 회전 실패·20 s 초과면 그대로 등록. `kiosk.goal_pose.yaw`도 -1.5472 → 1.5944(반대)로 바꿈.
(`course_engine.py` `_kiosk_turn`, `a_course_node.py` `_start_spin`, 테스트 `test_kiosk_turn_back_to_kiosk_from_side`)

## 확인 방법

```
[a_course_node]: 키오스크 방향 맞추기: -17x° 회전 (Nav2 spin)
[behavior_server]: spin completed successfully
```
로봇 등이 키오스크를 보고 등록(주먹→손바닥)이 됨.
