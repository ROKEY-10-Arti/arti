---
title: 관람객이 1.5 m 넘게 멀어지면 로봇이 서 버리고, 놓친 뒤 최종 인식 지점으로도 안 감 (depth 바닥 점 → DWA 목표 경로 없음)
date: 2026-11-18
author: jsy0605
status: 확인 중
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

- FOLLOW 중 관람객이 1.5~2.2 m 떨어져 서 있거나 걸어가는데 로봇이 `cmd_vel` v=0, w=0으로 서 있음
- SEARCH에서 최종 인식 지점(0.7 m 앞)으로 이동해야 하는데 5 s(`search_move_time`) 동안 제자리에 있다가 회전으로 넘어감
- 로그에는 아무 경고도 없음 (`blocked`도 아님)

bag `~/arti_bags/follow/20261118_123207`:

| 구간 [s] | 상태 | 관람객 거리 | 실제 cmd_vel v |
|---|---|---|---|
| 35~39 | SEARCH (최종 지점까지 0.7 m) | - | 0.00 |
| 67~79 | FOLLOW | 1.6~2.2 m | 0.00~0.02 |
| 106~109 | FOLLOW | 1.7~1.8 m | 0.00 |

FOLLOW 중 거리 1.2 m 초과이면서 정지한 시간: 110919 9.6 s, 112125 2.9 s, 114122 43.8 s, 115116 23.5 s, 123207 20.7 s (오늘 bag 전부에 있었음).

## 재현 방법

```bash
ros2 launch arti_follow tour.launch.py      # 관람객이 로봇에서 1.5 m 넘게 떨어지게 걷기
```

## 원인

- **좌표 거리 제한은 없음.** 인식 좌표는 2.4 m까지 12.6 Hz로 끊김 없이 옴, map↔base 좌표 차이 p90 0.045 m. `on_person()`에 거리 조건 없음.
- **실제 노드 클래스로 bag을 다시 돌림** (DOMAIN 99, bag의 scan·TF·point_map을 같은 순서로 넣고 cmd_vel 시각마다 `step()`):
  - 정상 구간: 실제 cmd_vel과 평균 0.035 m/s 차이로 재현
  - 멈춘 구간(23 s): 실제 평균 0.017 m/s, 재현(LiDAR만) 0.30 m/s
  - 멈춘 구간에 "목표까지 길 없음"(`field()` 전부 50)을 넣으면 실제와 똑같이 v→0, w=0
  - 오전 bag 114122도 같은 패턴 20 s
- **DWA가 목표까지 길이 없으면 조용히 멈춘다.** 모든 후보 비용이 같아 `w_turn` 때문에 w=0, 가장 낮은 v가 뽑힘.
  SEARCH 이동도 같은 `dwa()`를 써서 최종 지점으로 못 감.
- 재현과 실제의 입력 차이는 OAK-D depth 바닥 점(`floor_pts`)뿐 (`use_depth` 기본 true, depth는 bag에 없음).
  바닥에 물체가 없었으므로 **빈 바닥이 높이 5 cm 이상으로 잡힌 것**으로 추정. 관람객이 가까우면 다리가 바닥을 가리고, 멀면 앞 바닥(1.2 m 안)이 보여서 멀 때만 생긴다.
  depth를 녹화하지 않아 직접 확인은 못 함.

## 해결

장애물 회피는 앞으로 뺄 예정이라 depth를 코드에서 뺐다 (`src/arti_follow/arti_follow/visitor_follower.py`).
- 파라미터 삭제: `use_depth`, `depth_step`, `depth_max_range`, `floor_min_height`, `floor_max_height`, `floor_min_points`, `depth_keep`
- `oakd/stereo/*` 구독, `on_depth()` 삭제, `obstacles()` = LiDAR 점만

## 확인 방법

- 오프라인: 수정한 코드로 같은 bag 재현 → 멈춘 구간 23 s에서 v 0.30 m/s (실제 0.017)
- 로봇 확인 필요: 관람객이 1.5 m 넘게 떨어졌을 때 FOLLOW 중 v=0·w=0 구간이 없는지, SEARCH에서 최종 지점까지 가는지

## 참고

- 근거 문서: `docs/follow/visitor_follower_주행근거.docx`
- 남은 위험: depth가 없어도 LiDAR 점으로 목표가 막히면 같은 조용한 정지가 생길 수 있다 (`dwa()` 동점 처리)
- 관련: [2026-11-18-search-spin-tiny-w-create3.md](2026-11-18-search-spin-tiny-w-create3.md)
