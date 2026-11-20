---
title: 관람 중 로봇이 뚝뚝 멈춤 (라이다 scan이 Wi-Fi로 0.5~0.8 s씩 끊김, scan_timeout 0.5 s)
date: 2026-11-20
author: 이정욱
status: 우회
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

관람(FOLLOW) 중 로봇이 짧게 섰다 가기를 반복함 (뚝뚝 끊김). visitor_follower 로그:

```
[WARN] [robot1.visitor_follower]: 라이다 scan 안 옴 (마지막 수신 0.6 s 전) → 정지 (확인: ros2 topic info /robot1/scan), 상태 FOLLOW, 0.0 s째
[INFO] [robot1.visitor_follower]: 센서 정상 → 다시 진행 (0.4 s 동안 정지했음, 상태 FOLLOW)
```

한 번 관람(약 3분)에 18번 정지 (합계 6.7 s, 가장 길게 1.2 s). 아침 실행에서도 28번.

## 재현 방법

```bash
ros2 launch arti_bringup localization.launch.py
ros2 launch arti_follow tour.launch.py diag:=true      # 진단 로그 켜기
# 키오스크(또는 CLI)로 prepare_tour → start_tour, 관람객을 따라가게 함
ping 192.168.107.101                                   # 다른 터미널, 같은 시간에
```

## 원인

- visitor_follower는 `scan_timeout`(0.5 s)보다 오래 `/robot1/scan`을 못 받으면 안전을 위해 정지한다 (`step()` [3]).
- 로봇 라이다는 정상: PC에서 `ros2 topic hz /robot1/scan` = 8.0 Hz, 간격 0.12~0.14 s.
- 끊김은 Wi-Fi 전달 중 생김 (추정, 근거 아래):
  - scan은 BEST_EFFORT(`sensor_data` QoS). 약 11 KB라 여러 UDP 조각으로 가고, 조각 하나만 잃어도 그 scan 전체를 버린다 → 3~4개 연속으로 빠지면 0.5 s 넘게 끊김.
  - 같은 scan을 visitor_follower, 인식 노드(LidarPersonLocalizer), AMCL이 각자 받아 Wi-Fi로 여러 번 보낸다.
  - ping 평균 18 ms, 최대 108 ms로 들쑥날쑥 (11/19에는 0.5~1.5 s).
- 인식 노드의 타겟 좌표 끊김(`관람객 놓침`)과는 다른 문제다. 라이다 끊김은 주행 모듈이 로봇 scan을 못 받은 것.

## 해결 (우회)

- `src/arti_follow/config/visitor_follower.json`: `scan_timeout` 0.5 → 0.8 s.
  0.5~0.8 s 끊김에는 멈추지 않는다 (0.2 m/s면 최대 약 16 cm를 새 scan 없이 감).
- `follow_test.launch.py`에 `diag:=true` 인자 추가 (기본 false): 토픽별 수신 간격·지연, 제어 주기, 끊긴 순간 로그.
- `control_period` 0.2 s도 시험했지만 개선이 없어 0.1 s로 유지.
- Wi-Fi 자체(공유기 채널·거리)와 scan 중복 전송(릴레이 노드)은 아직 손대지 않음.

## 확인 방법 (2026-11-20 15:17~15:23, `diag:=true`, scan_timeout 0.8 s)

관람 한 번 완주 (호출 → 등록 → 추종 → 작품1 설명 → 출구 → 도킹). 관람 중(FOLLOW·SEARCH) 2 s 창 통계:

| 토픽 | 평균 | 간격 최대 | 지연 평균 (최대) |
|---|---|---|---|
| scan | 7.8 Hz | 0.80 s | 0.068 s (2.0 s) |
| tf | 37.5 Hz | 0.68 s | - |
| target_person/point_map | 15.7 Hz | 11.56 s (사람이 안 보일 때) | 0.051 s (0.24 s) |

- 관람 중 라이다 때문에 멈춘 주기 0 (정지 이유 분포: hold 734, follow 508, search_spin 82, search_move 35, end_guide 19). RETURN 중 1번 (0.1 s).
- 0.6~0.8 s 끊김은 관람 끝 무렵 4번 있었다 → 예전 0.5 s였다면 모두 정지였을 것.
- 도킹 뒤(READY)에는 scan이 1~6.7 s씩 끊김. 도크 위에서 TurtleBot 4가 라이다를 쉬게 하는 것으로 추정 (관람에는 영향 없음, 미확인).

## 남은 문제

- **PC 계산**: 추종 중 제어 한 번 계산이 평균 35~55 ms, 최대 98 ms (그중 경로 계산 `field()` 최대 73 ms).
  목표 0.1 s에 거의 닿아 `[제어 밀림] 주기 0.2 s`가 26번. 팀원 브랜치 `fix/follow-stability-perf`(BFS 속도 개선)를 합치면 줄어들 것.
- **[시간] 경고**: 일부 2 s 창에서 scan 최소 지연이 0.9 s → 그 순간 Wi-Fi 지연 또는 시계 차이. `chrony`로 PC·로봇 시간 동기화 확인 필요.

## 참고

- 진단 로그: `~/.ros/log/python_147919_1795155468136.log` (로컬, 저장소에 안 올림)
- 관련: [2026-11-18 Create 3 작은 회전 명령](2026-11-18-search-spin-tiny-w-create3.md) (scan 간격 조정 이력)
