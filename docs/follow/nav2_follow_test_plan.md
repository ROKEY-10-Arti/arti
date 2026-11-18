# nav2_follow 테스트 계획

`nav2_follow`(주행을 Nav2에 맡긴 노드)가 `visitor_follower`를 **그대로 대신할 수 있는지** 확인한다.
검증용 코드는 추가하지 않는다. 모든 확인은 `ros2` 명령(topic·action·bag), 노드 로그, RViz, 줄자로 한다.

- 설계: `~/Desktop/nav2_follow_설계.docx` (v4)
- 코드: `src/arti_follow/arti_follow/nav2_follow.py`, 노드 설정 값 `src/arti_follow/config/nav2_follow.json` (`-p 이름:=값`으로 덮어씀), Nav2 설정 `src/common/arti_bringup/config/nav2_follow.yaml`, BT `src/common/arti_bringup/behavior_trees/`, launch `src/arti_follow/launch/tour_nav2.launch.py`
- 결과 기록: 이 문서 맨 아래 **결과 기록표**를 복사해 `docs/follow/nav2_follow_test_results_YYYY-MM-DD.md`로

## 0. 무엇을 검증하나

| 묶음 | 질문 | 합격 기준 (한 줄) |
|---|---|---|
| A. 기동 | Nav2가 우리 설정으로 뜨고, 노드가 뜨는가 | 에러 없이 `navigate_to_pose` 준비, 노드 `READY` |
| B. 관람 흐름 | 액션 하나로 undock → 위치 → 키오스크 보기 → 등록까지 가는가 | 상태 순서·feedback이 visitor_follower와 같음 |
| C. 추종 | 0.8 m를 유지하고, 서서 바라보고, 다시 출발하는가 | 서는 거리 0.8~0.95 m, 섰다 갔다 없음 |
| D. 이탈처리 | 놓치면 찾고, 다시 보이면 따라가고, 끝까지 없으면 끝내는가 | SEARCH → WAIT → LOST, 한 바퀴 ≥ 360° |
| E. 복귀·도킹 | 출구에 들어가면 도크로 돌아가 도킹하는가 | result `finished`, 도크 충전 |
| F. 멈춤·취소 | enable·취소·Ctrl-C에 로봇이 서는가 | 1 s 안에 정지 |
| G. 비교 | 지금 노드보다 나은가 | 아래 지표가 같거나 나음 |

## 1. 준비

### 1.1 단계 (위에서부터, 앞 단계를 통과해야 다음으로)

| 단계 | 어디서 | 하는 시험 | 로봇 움직임 |
|---|---|---|---|
| S0 격리 | 이 PC만 (`ROS_DOMAIN_ID=99`) | A1, A2, A3 | 없음 |
| S1 시뮬레이션 | Gazebo (gallery 월드, robot1) | B·C·D·E·F (도킹 제외) | 시뮬레이션 로봇만 |
| S2 로봇 | robot1 실물 | 전부 + G | 있음 (사람이 옆에서 지켜봄) |

### 1.2 S0 격리 환경 (로봇과 통신 안 함)

```bash
export ROS_DOMAIN_ID=99; unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### 1.3 S1 시뮬레이션 (터미널마다 저장소 루트에서, `use_sim_time`)

S0과 같은 격리 환경 변수를 먼저 설정한다. 처음 실행할 때 명령이 그대로 되는지부터 확인하고, 안 되면 트러블슈팅 기록을 남긴다
(저장소 `simulation/arti_sim.launch.py`는 robot4 코스주행용이라 그대로 쓰지 않고, 같은 구성을 robot1로 나눠 띄운다).

```bash
# 터미널 1: Gazebo + gallery 월드
GZ_SIM_RESOURCE_PATH=$PWD/simulation:$(ros2 pkg prefix turtlebot4_description)/share:$(ros2 pkg prefix irobot_create_description)/share \
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$PWD/simulation/arti_gallery.sdf -r -v 2"
# 터미널 2: 시계
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
# 터미널 3: robot1 생성 (언독 지점, 방 안쪽을 봄)
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py namespace:=robot1 world:=arti_gallery model:=lite \
  x:=-0.5 y:=0.7 yaw:=3.14 rviz:=false nav2:=false localization:=false slam:=false
# 터미널 4: localization
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 use_sim_time:=true map:=$PWD/maps/gallery.yaml
# 터미널 5: Nav2 (우리 설정)
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1 use_sim_time:=true \
  params_file:=$(ros2 pkg prefix arti_bringup)/share/arti_bringup/config/nav2_follow.yaml
# 터미널 6: RViz
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot1
# 터미널 7: 노드 (도크 없음, 등록 요청 없음 → 가짜 좌표를 바로 받음)
ros2 run arti_follow nav2_follow --ros-args -r __ns:=/robot1 \
  -p use_sim_time:=true -p undock:=false -p dock:=false -p auto_register:=false
```

**가짜 관람객** (S1): 인식 모듈 대신 좌표를 직접 보낸다. 좌표를 바꾸려면 Ctrl-C 후 값만 바꿔 다시 실행.
```bash
ros2 topic pub -r 10 /robot1/target_person/point_map geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: -1.5, y: 0.7}}"
```
- 멈추면(Ctrl-C) = 관람객이 사라짐(1.5 s 뒤 놓침), 다시 실행 = 다시 보임
- 0.15 m 넘게 바꾸면 = 관람객이 걸어감 (한 번에 1 m 넘게 바꾸면 칼만 필터가 5번 버린 뒤 새로 시작 → 0.5 s 늦게 반응하는 것이 정상)

### 1.4 S2 로봇

```bash
# 터미널 1: localization (docs/common/navigation.md). 기본 설정 nav2가 떠 있으면 먼저 끈다
ros2 launch turtlebot4_navigation localization.launch.py namespace:=$ARTI_ROBOT1_NS map:=$ARTI_MAP
# 터미널 2: Nav2(nav2_follow.yaml) + 인식·작품 구역·음성 + nav2_follow + bag 녹화
ros2 launch arti_follow tour_nav2.launch.py nav2:=true
#   (Nav2를 따로 띄워 두었으면 nav2:=true 없이)
# 터미널 3: 관람 시작
ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}" --feedback
```
G(비교)는 같은 시나리오를 `ros2 launch arti_follow tour.launch.py`(visitor_follower, Nav2는 기본 설정이어도 됨)로 한 번 더.

### 1.5 공통으로 띄워 둘 관찰 창

```bash
ros2 topic echo /robot1/follow/state                        # 상태 바뀔 때마다
ros2 topic hz /robot1/goal_update                           # FOLLOW 중 목표 갱신 빈도 (≤ 5 Hz)
ros2 topic echo /robot1/navigate_to_pose/_action/status     # Nav2 목표 상태 (2 = 실행 중, 4 = 성공, 5 = 취소, 6 = 실패)
ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static   # 로봇 위치
```
그리고 nav2_follow 터미널 로그.

## 2. 시험 항목

각 항목: **절차 → 무엇을 보나 → 합격 기준**. 결과는 기록표에 통과/실패와 근거(로그 한 줄·숫자)로 적는다.

### A. 기동 (S0, S1)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| A1 | nav2를 `nav2_follow.yaml`로 실행 | nav2 로그 | `Created controller : FollowPath of type nav2_rotation_shim_controller::RotationShimController`, `Created goal checker : general_goal_checker`, (S1·S2) `Managed nodes are active`, 파라미터 관련 ERROR 없음 |
| A2 | `ros2 run arti_follow nav2_follow --ros-args -r __ns:=/robot1` | 노드 로그, `follow/state` | `상태 None → READY`, `init_pose.yaml ... 못 읽음` 경고 없음 |
| A3 | (S0, Nav2 없이) 액션 보내기 / 관람 중 하나 더 보내기 / `{mode: COURSE}` 보내기 | 액션 result | 순서대로 `no_nav2` / `busy` / `unsupported_mode`, 셋 다 `success: false` |

### B. 관람 흐름 (S1, S2)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| B1 | (S2) 도크에 붙은 상태에서 액션 | 로그, 로봇 | `undock 시작` → `undock 끝` → `LOCALIZE` |
| B2 | 이어서 | 로그, RViz | `초기 위치 보냄` → `초기 위치 잡힘` → `START_FACE` (10 s 안) |
| B3 | 이어서 | 로봇 방향, Nav2 status | 제자리에서 키오스크(-1.11, -0.39) 쪽으로 돈 뒤 `시작 회전 끝 (succeeded)` → `WAIT_VISITOR`. 앞뒤 이동 없음 |
| B4 | 관람객이 앞에 섬 (S1: 가짜 좌표 시작) | 로그, 음성, feedback | 1 s 뒤 `관람객 등록 완료`, `guide_message: recognized`(음성), feedback `ARRIVED → FOLLOW`, `registered: true` |
| B5 | 관람 내내 | feedback state | MOVING → ARRIVED → FOLLOW(설명 중 EXPLAINING) → RETURNING 순서. visitor_follower와 같음 |

**B3이 실패하면** (돌지 않고 끝나거나 제자리에서 못 돎): 설계 12절의 "지금 위치 + 다른 방향" 함정. 그대로 기록하고 C로 넘어가지 않는다 (C의 서서 바라보기도 같은 방식).

### C. 추종 (S1, S2)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| C1 | 관람객이 로봇에서 2 m 떨어져 섬 | 로봇 이동, 서는 위치 | 다가가서 사람과 **0.8~0.95 m**에서 섬 (S2: 바닥 테이프·줄자, S1: tf2_echo 로봇 위치와 가짜 좌표 차이) |
| C2 | C1 뒤 사람이 그대로 서 있음 30 s | `cmd_vel`, `goal_update` | 섰다 갔다 없음: `ros2 topic echo /robot1/cmd_vel --field twist.linear.x`가 0 유지, `goal_update` 거의 안 나옴 |
| C3 | 사람이 제자리에서 옆으로 0.5 m 이동 (거리는 그대로) | 로봇 | 앞으로 안 가고 몸만 사람 쪽으로 돎 |
| C4 | 사람이 10 cm씩 천천히 멀어짐 | 로봇이 다시 출발하는 순간의 거리 | 약 **0.95 m**에서 출발 (0.85~0.95 사이에서는 안 움직임) |
| C5 | 사람이 걸어서 방을 한 바퀴 | `goal_update` 빈도, 움직임 | `ros2 topic hz` ≤ 5 Hz, 멈칫거림(같은 자리에서 서다 가다) 없음, 벽·기둥에 안 닿음 |
| C6 | 사람이 1.5~2.2 m로 멀어진 채 걸음 (지금 노드의 멈춤 문제 구간) | 로봇 | 계속 따라감 (서 버리지 않음) |

### D. 이탈처리 (S1, S2)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| D1 | 추종 중 사람이 기둥 뒤로 사라짐 (S1: 가짜 좌표 Ctrl-C) | 로그 | 1.5 s 뒤 `놓침 → 마지막 위치 (...)로 가서 왼/오른쪽으로 한 바퀴`, `FOLLOW → SEARCH` |
| D2 | 이어서 | 로봇, 로그 | 마지막 위치 쪽으로 최대 5 s 이동 → 제자리 한 바퀴(**≥ 360°**, 약 10 s, 사람이 가던 쪽 먼저) → `회전 탐색 끝 → 정지 대기`, `SEARCH → WAIT` |
| D3 | 계속 안 보임 | 로그, result | WAIT 10 s 뒤 `관람 끝 (lost)`, `arti/tour_end` 한 번, result `lost` |
| D4 | D2 회전 중에 다시 보임 (S1: 가짜 좌표 다시 실행) | 로그, 로봇 | 회전을 멈추고 `SEARCH → FOLLOW`, 다시 따라감 |
| D5 | WAIT 중에 다시 보임 | 로그 | `WAIT → FOLLOW` |

회전량 확인 (S1·S2): 회전 전후 `tf2_echo map base_link`의 yaw, 또는 bag의 odom.

### E. 복귀·도킹 (S1은 E1·E2만, S2 전부)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| E1 | 관람객이 출구 구역에 들어감 (S1: `ros2 topic pub --once /robot1/artwork_zone/entered std_msgs/msg/String "{data: exit}"`) | 로그 | `관람객 데이터 삭제 (arti/tour_end)`, `→ RETURN` |
| E2 | 이어서 | 로봇 | 언독 지점(-0.5, 0.7)으로 가서 **도크 쪽을 보고** 섬 → `도크 앞 도착` → `DOCK` (S1은 dock:=false라 바로 result `finished`) |
| E3 | (S2) 이어서 | 로봇, `dock_status` | `dock 끝`, 충전 표시, result `success: true, message: finished`, 상태 READY |
| E4 | (S2) 도크 앞까지 막아 둠 | result | 90 s 안에 못 가면 `return_failed` (또는 Nav2 실패) |

### F. 멈춤·취소 (S1, S2)

| ID | 절차 | 볼 것 | 합격 기준 |
|---|---|---|---|
| F1 | 추종 중 `ros2 topic pub --once /robot1/follow/enable std_msgs/msg/Bool "{data: false}"` | 로봇, Nav2 status | 1 s 안에 정지 (status 5 = 취소) |
| F2 | 이어서 `{data: true}` | 로봇 | 다시 따라감 |
| F3 | 추종 중 액션 취소 (send_goal 터미널에서 Ctrl-C) | result, 로봇 | result `canceled`, 정지, `arti/tour_end` 한 번, READY |
| F4 | 추종 중 nav2_follow 터미널 Ctrl-C | 로봇 | 정지 (취소가 Nav2에 전달됨) |
| F5 | `tour_nav2.launch.py drive:=false`로 관람 | 로봇, 로그 | 상태는 진행되지만 로봇은 안 움직임 (START_FACE는 15 s 뒤 넘어감) |

### G. 비교 (S2, visitor vs nav2)

같은 날, 같은 사람, 같은 경로(바닥 테이프로 표시: 직진 3 m → 멈춤 10 s → 옆으로 1 m → 기둥 뒤로 사라짐 → 다시 나타남 → 출구)를 두 노드로 각각 2번.

| 지표 | 보는 방법 | 비교 |
|---|---|---|
| 서는 거리 | 멈춤 구간에서 줄자 (3번 평균) | 0.8 m에 가까운 쪽 |
| 너무 가까움 | 0.65 m 안으로 들어온 횟수 (눈으로 + 테이프) | 적은 쪽 |
| 섰다 갔다 | 멈춤 10 s 동안 `cmd_vel` linear.x가 0 ↔ 양수 바뀐 횟수 (bag 재생 + `ros2 topic echo --field`) | 적은 쪽 |
| 좌우 흔들림 | 멈춤 구간 `angular.z` 부호가 바뀐 횟수 | 적은 쪽 |
| 놓친 횟수 | `follow/state`의 `FOLLOW → SEARCH` 횟수 (bag `/rosout` 또는 로그) | 적은 쪽 |
| 다시 찾기 | 사라진 뒤 다시 나타났을 때 FOLLOW로 돌아오기까지 시간 | 짧은 쪽 |
| 전체 완주 | result `finished`까지 갔는지 | 둘 다 성공 |

## 2.5 실주행 시나리오 (S2: 위 시험 항목을 묶어서 한 번에 주행)

### 실행 단위 = Run. Run 하나 = launch 한 번 = bag 하나

Nav2는 한 번만 켜 두고, Run마다 `tour_nav2.launch.py`만 다시 켠다 (끄면 bag이 닫히고, 켜면 새 bag).
```bash
# 한 번만: Nav2 (우리 설정)
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=$ARTI_ROBOT1_NS \
  params_file:=$(ros2 pkg prefix arti_bringup)/share/arti_bringup/config/nav2_follow.yaml
# Run마다: bag_dir로 Run 이름을 붙임 → ~/arti_bags/nav2_test/run1/<날짜_시각>/
ros2 launch arti_follow tour_nav2.launch.py bag_dir:=~/arti_bags/nav2_test/run1
```
- **Run을 시작할 때 로봇은 도크 위에** (또는 언독 지점 (-0.5, 0.7)에서 방 안쪽을 보게). 관람을 시작하면 노드가 AMCL에 "언독 지점에 있다"고 알려 주기 때문 (LOCALIZE). 다른 곳에서 시작하면 위치가 틀어진다.
- Run 안에서 시나리오를 바꿀 때마다 **시계 시각을 메모** (bag 시각 = 실제 시각이라 나중에 구간을 잘라 볼 수 있음).
- visitor_follower 비교 Run(4)도 Nav2는 그대로 둬도 된다 (visitor_follower는 Nav2를 쓰지 않고 cmd_vel을 직접 보냄).

| Run | 시작 | 시나리오 순서 (관람객 행동) | 시험 ID | 끝 |
|---|---|---|---|---|
| 1 전체 관람 | 도크 | ① 키오스크 앞에서 기다리다 로봇 앞에 서서 제스처로 등록 ② 2 m 앞에 서서 로봇이 다가와 서게 → 30 s 가만히 ③ 제자리에서 옆으로 0.5 m ④ 10 cm씩 천천히 뒤로 (로봇이 다시 출발하는 거리) ⑤ 테이프 경로를 보통 걸음으로 한 바퀴, 작품 2개 앞에서 5 s 멈춤 ⑥ 1.5~2 m 떨어진 채로 걷기 ⑦ 기둥 뒤로 숨었다가 로봇이 돌기 시작하면 다시 나옴 ⑧ 출구 구역으로 들어감 | B1~B5, C1~C6, D1·D2·D4, E1~E3 | 도킹 (자동) |
| 2 멈춤·분실 | 도크 | ① 등록 ② 따라오게 걷다가 enable false → 5 s → true ③ 기둥 뒤로 숨어서 끝까지 안 나옴 (약 30 s) | F1, F2, D1~D3, D5(선택: WAIT 중 나왔다 다시 숨기) | LOST (자동 종료). 로봇을 손으로 도크에 올려 둠 |
| 3 취소 | 도크 | ① 등록 ② 따라오게 걷다가 액션 터미널에서 Ctrl-C | F3 | canceled. 로봇을 도크에 올려 둠 |
| 4 비교 (visitor) | 도크 | Run 1의 ②·⑤·⑦·⑧만 같은 경로로 | G | 도킹. `ros2 launch arti_follow tour.launch.py bag_dir:=~/arti_bags/nav2_test/run4_visitor` |
| 5 비교 (nav2) | 도크 | Run 4와 똑같이 | G | 도킹. `tour_nav2.launch.py bag_dir:=~/arti_bags/nav2_test/run5_nav2` |

여유가 있으면 Run 4·5를 한 번씩 더 (같은 조건 2번 → 우연인지 구분).

### bag에 무엇이 남나 (`follow_test.launch.py` BAG_TOPICS, 카메라 영상 없음)

| 토픽 | 무엇 |
|---|---|
| `follow/state` | 상태가 바뀐 시각 (READY → … → FOLLOW ⇄ SEARCH …) |
| `/rosout` | 모든 노드 로그: nav2_follow(놓침·회전·등록·결과), Nav2(목표 성공·실패·경로 못 찾음), 인식 모듈 |
| `target_person/point_map`, `point_base`, `marker` | 인식 모듈이 본 관람객 위치 (map·로봇 기준) |
| `goal_update` | nav2_follow가 Nav2에 넘긴 추종 목표 (위치·방향) |
| `cmd_vel` | Nav2가 로봇에 보낸 최종 속도 |
| `odom` | 로봇이 실제로 움직인 속도·회전 |
| `tf`, `tf_static`, `amcl_pose`, `initialpose` | 로봇 위치 (map → odom → base_link), 초기 위치 |
| `scan` | 주변 장애물 (벽·기둥까지 거리) |
| `artwork_zone/state`, `artwork_zone/entered`, `guide_message`, `arti/tour_end` | 작품 구역 판정, 출구 진입, 음성 안내, 관람 끝 |
| `dock_status`, `follow/enable`, `target_person/register_front`, `gesture/status` | 도킹 여부, 멈춤 명령, 등록 요청·제스처 |

### bag으로 알 수 있는 것

| 알고 싶은 것 | 어떻게 (bag 안에서) | 시험 ID |
|---|---|---|
| 관람 흐름이 순서대로 갔나, 단계별 걸린 시간 | `follow/state` 전환 시각 + `/rosout` | B, E |
| 키오스크 쪽으로 제대로 돌았나 | START_FACE 동안 tf의 yaw 변화 vs 키오스크 방향 | B3 |
| 관람객과 몇 m에서 서나, 너무 가까워진 비율 | 매 순간 로봇 위치(tf) ↔ 관람객 위치(point_map) 거리 → 멈춘 구간의 거리 분포, 0.65 m 미만 비율 | C1, G |
| 섰다 갔다 하나, 좌우로 흔들리나 | 멈춘 구간 `cmd_vel` linear.x가 0 ↔ 양수 바뀐 횟수, angular.z 부호 바뀐 횟수 | C2, G |
| 몇 m 멀어지면 다시 출발하나 | `cmd_vel`이 0 → 양수가 된 순간의 거리 | C4 |
| 목표를 너무 자주·크게 바꾸나 | `goal_update` 빈도, 연속 목표 사이 거리 | C5 |
| 걸을 때 얼마나 늦게 따라오나, 멀 때 서 버리나 | 걷는 구간의 거리 변화, 1.5 m 넘는 구간의 `cmd_vel` | C5, C6 |
| 벽·기둥에 얼마나 가까이 가나 | 주행 중 `scan` 최솟값 | C5 |
| 몇 번 놓쳤고, 다시 찾는 데 몇 초 | `FOLLOW → SEARCH` 횟수, `SEARCH → FOLLOW`까지 시간 | D, G |
| 탐색 때 정말 한 바퀴 돌았나 | SEARCH 동안 `odom` 회전량 (°), 걸린 시간 | D2 |
| 놓치기 직전에 무슨 일이 있었나 | 그 직전 1.5 s의 `odom` 회전 속도·`point_map` 끊김 (빠른 회전 때문인지 등) | D1 |
| 멈춤·취소에 바로 서나 | `follow/enable`·취소 시각 → `odom` 속도가 0이 되기까지 | F |
| 위치 추정이 튀었나 | `amcl_pose` 연속 값의 점프 | 전체 |
| Nav2가 실패·재시도했나 | `/rosout`의 Nav2 경고·오류 (경로 없음, 진행 없음 등) | 전체 |

**bag으로 알 수 없는 것**: 관람객의 실제 위치 (point_map은 인식 모듈의 추정값) → 멈춘 순간의 진짜 거리는 **줄자**로 재서 메모.
인식 모듈이 다른 사람으로 바꿔 붙었는지는 영상이 없어서 `/rosout`의 대상 ID 변화로만 추정한다.

### Run 기록 메모 (실주행 중 손으로)

```
Run 1  시작 21:05  bag: ~/arti_bags/nav2_test/run1/20261119_210500
  21:06:10 ② 정지 시작 — 줄자 0.86 m
  21:07:30 ⑤ 걷기 시작
  21:09:05 ⑦ 숨음 → 21:09:20 나옴
  메모: 기둥 옆에서 잠깐 멈칫 (21:08:40)
```
이 메모와 bag 폴더를 결과 파일에 같이 남긴다.

## 3. 기록·정리

- **bag**: S2는 `tour_nav2.launch.py`(와 `tour.launch.py`)가 자동으로 `~/arti_bags/follow/<날짜_시각>/`에 녹화한다 (카메라 영상 제외, `goal_update` 포함). S1은 필요하면
  `ros2 bag record -o ~/arti_bags/nav2_sim/<이름> /robot1/follow/state /robot1/goal_update /robot1/cmd_vel /robot1/odom /robot1/tf /robot1/target_person/point_map /rosout`
- **로그**: nav2_follow 터미널 출력. 문제가 난 시험은 해당 부분을 기록표 "근거"에 붙인다.
- **결과 파일**: 아래 표를 복사해 `docs/follow/nav2_follow_test_results_YYYY-MM-DD.md`. bag 폴더 이름도 같이 적는다.
- **실패**: `docs/troubleshooting/follow/YYYY-MM-DD-slug.md`에 기록하고 README 목록에 한 줄 (CLAUDE.md 규칙). 기록표에는 그 파일 링크.
- **판정**: A~F 전부 통과 + G에서 나쁜 지표 없음 → `follower` 기본값을 `nav2`로 바꾸는 PR. 하나라도 실패 → 원인 고치고 그 묶음부터 다시.

## 결과 기록표 (복사해서 사용)

```markdown
# nav2_follow 테스트 결과 YYYY-MM-DD

- 단계: S0 / S1 / S2      시험자:        커밋: (git log -1 --format=%h)
- bag: ~/arti_bags/follow/....

| ID | 결과 (통과/실패/건너뜀) | 근거 (로그 한 줄·숫자) | 비고 |
|---|---|---|---|
| A1 | | | |
| A2 | | | |
| A3 | | | |
| B1 | | | |
| B2 | | | |
| B3 | | | |
| B4 | | | |
| B5 | | | |
| C1 | | 서는 거리: m | |
| C2 | | | |
| C3 | | | |
| C4 | | 출발 거리: m | |
| C5 | | goal_update Hz: | |
| C6 | | | |
| D1 | | | |
| D2 | | 회전: ° / s | |
| D3 | | | |
| D4 | | | |
| D5 | | | |
| E1 | | | |
| E2 | | | |
| E3 | | | |
| E4 | | | |
| F1 | | | |
| F2 | | | |
| F3 | | | |
| F4 | | | |
| F5 | | | |

## G. 비교 (visitor / nav2)

| 지표 | visitor 1 | visitor 2 | nav2 1 | nav2 2 |
|---|---|---|---|---|
| 서는 거리 (m) | | | | |
| 0.65 m 안 횟수 | | | | |
| 섰다 갔다 횟수 | | | | |
| 좌우 부호 바뀜 | | | | |
| 놓친 횟수 | | | | |
| 다시 찾기 (s) | | | | |
| 완주 | | | | |

## 판정 / 다음 할 일
```
