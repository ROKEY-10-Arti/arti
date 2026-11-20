---
title: 서버(PC3, 192.168.107.42)만 쓰면 새로 띄운 노드끼리 못 찾거나 아주 느림 (lifecycle "Waiting for service", bond 4.00s, 액션 서버 대기)
date: 2026-11-20
author: flypig341
status: 우회
packages: [arti_pc2_nav, arti_bringup]
nodes: [lifecycle_manager_localization, lifecycle_manager_navigation, a_course_node]
robot: robot4
---

## 증상

`ROS_DISCOVERY_SERVER="192.168.107.42:11811;"`(관제 PC 서버만)인 터미널에서 launch를 띄우면 그때그때 다르게 막혔다.

```
[lifecycle_manager_localization]: Waiting for service map_server/get_state...      ← 같은 노트북 노드인데 1~5분
[lifecycle_manager_localization]: Server map_server was unable to be reached after 4.00s by bond. This server may be misconfigured.
[lifecycle_manager_localization]: Failed to bring up all requested nodes. Aborting bringup.
[lifecycle_manager_localization]: Configuring amcl          ← 여기서 4분 넘게 멈춤
Waiting for an action server to become available...          ← ros2 action send_goal이 코스 노드를 못 찾음
```

- 새 프로세스로 `ros2 topic list --no-daemon` → `/parameter_events`, `/rosout`만 보임. 아침부터 떠 있던 ros2 daemon으로 보면 다 보임
- 서버 42를 재시작한 직후 robot4 토픽이 1~2분 안 보이다 다시 보임
- 코스 노드가 `dock_status`를 제때 못 받아 "도크에 없음"으로 판단 → 호출 거절

## 재현 방법

```bash
export ROS_DISCOVERY_SERVER="192.168.107.42:11811;"
ros2 launch arti_pc2_nav a_course.launch.py enable_motion:=true
# 다른 터미널: 새 프로세스로 보기 (daemon 거치지 않게)
ros2 topic list --no-daemon --spin-time 15 | grep -c robot4
```

## 원인

확실하지 않다. 서버 42 한 곳이 robot1·robot4·두 로봇 PC·키오스크의 discovery를 모두 중계하고, launch 노드가 super client(`ROS_SUPER_CLIENT=True`, `.bashrc`)이면 네트워크 전체 명단을 받는다.
그래서 서버와 와이파이 부담이 커서 새 참가자 등록이 늦거나 빠지는 것으로 보인다.
같은 노트북 노드끼리도 서버를 거쳐 서로를 찾으므로 서버가 느리면 같이 느려진다.

## 해결

우회 + 부담 줄이기 (코드 `[코스 관람 변경 ⑥]`):

- launch 노드는 super client 끄기 (`a_course.launch.py` `super_client:=False`, arti_bringup도 기본 false). 터미널 ros2 CLI만 True
- `bond_timeout` 30 s: Nav2는 `config/nav2_real.yaml`, localization은 params 파일로 못 바꿔서 launch `SetParameter(bond_timeout=30.0)`
- (시도했다 되돌림) Nav2 노드를 컨테이너 하나로(composition) 묶어 봤는데, 서버 42 환경에서 launch → 컨테이너 `load_node` 응답이 유실돼
  Nav2가 하나도 안 떴다 (`failed to send response to /robot4/nav2_container/_container/load_node (timeout)`, `Loaded node` 0개).
  컨테이너는 노드를 하나씩 넣고 응답을 기다려서 응답 하나만 잃어도 전체가 멈춘다 → 노드마다 프로세스(TurtleBot4 기본)로 둠
- 로봇 OAK-D 끔 ([기록](../robot/2026-11-20-robot4-oakd-disabled.md)), 인식 노드가 웹캠 직접 받기
- 막히면: launch Ctrl+C → (필요하면) 관제 PC에서 `scripts/run_discovery_server.sh` 재시작 → 1분 뒤 다시 launch
- 더 확실한 방법(팀 설정 문서의 guest 구성): 자기 로봇 서버도 같이 적기
  `export ROS_DISCOVERY_SERVER="192.168.107.42:11811;;;;192.168.107.104:11811;"` → 같은 PC 노드끼리는 robot4 서버로 바로 찾음 (이날 시험에서 localization 1 s 안에 active)

## 확인 방법

```
[lifecycle_manager_localization]: Managed nodes are active
ros2 topic list --no-daemon --spin-time 15 | grep -c robot4    # 20개 넘게 나오면 연결됨
```

## 참고

- [localization이 "Configuring amcl"에서 멈춤](2026-11-17-localization-stuck-discovery.md), [nav2 lifecycle 응답이 사라짐](2026-11-17-nav2-lifecycle-stuck-discovery.md) (같은 계열)
- `ros2 node list`가 꺼진 노드를 보여 줄 때: [ros2 daemon 예전 목록](2026-11-17-ros2-daemon-stale-topic-list.md) → `ros2 daemon stop`
