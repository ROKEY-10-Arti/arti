---
title: 작품 앞에서 해설 시작 0.7 s 만에 "해설 중 현재 작품 반원 이탈 또는 위치 정보 소실"로 멈춤
date: 2026-11-20
author: flypig341
status: 해결
packages: [arti_pc2_nav]
nodes: [a_course_node]
robot: robot4
---

## 증상

```
[a_course_node]: {"state": "ARTWORK_STOPPED", "detail": "1"}
[a_course_node]: 작품 1 해설 요청 (explain_artwork)
[a_course_node]: {"state": "FAILED", "detail": "해설 중 현재 작품 반원 이탈 또는 위치 정보 소실"}     ← 0.7 s 뒤
[a_course_node]: start_tour 끝: fail (해설 중 현재 작품 반원 이탈 또는 위치 정보 소실)
```

## 원인

로봇은 반원에 들어서자마자 멈춰서 **경계선 바로 안쪽**에 선다. 해설 중에는 0.1 s마다 "반원 안 + 위치가 2 s 안에 갱신됨"을 보는데, **한 번만 아니어도 즉시 실패**였다.
서 있어도 AMCL 위치가 몇 cm씩 흔들리고, 와이파이로 TF가 늦게 오기도 해서 금방 실패 조건에 걸렸다. (`_entry_inside` 주석에 이미 적혀 있던 위험)

## 해결

`[코스 관람 변경 ⑧]`: 해설 중 감시에만 여유를 둠 (진입·정지 판정은 그대로).
- 반원 밖 `explain_zone_margin` 0.15 m까지는 안으로 봄
- 밖이거나 위치가 끊긴 상태가 `explain_lost_grace` 3 s 넘게 **계속**될 때만 실패, 메시지도 "반원 이탈" / "위치 정보 소실"로 나눔
(`course_engine.py` `_explain_ok`, `tick` (5), `explanation_result`, 테스트 `test_explaining_tolerates_amcl_jitter`)

## 확인 방법

작품 4개 해설이 끝까지 나오고 `ARTWORK_COMPLETED`가 차례로 옴.
