# visitor_follower 실행하면 일어나는 일

`src/arti_follow/arti_follow/visitor_follower.py` 한 파일. 로봇 1대(`/robot1`)가 관람객 1명을 맡아
**관람 시작 → 등록 → 추종 → 분실 처리 → 관람 끝**까지 움직임을 담당한다.
작품 설명 판단은 `artwork_zone`, 음성은 `arti_audio`의 `guide_audio_node`, 사람 인식은 `person_tracker`가 한다.

## 1. 켜자마자

```
노드 시작
  ├ init_pose.yaml   → 출발 지점 (x, y, yaw)        (robots.<네임스페이스>.undock)
  ├ artworks.yaml    → 바라볼 곳 (kiosk 구역 중심), 작품 수 (feedback total)
  ├ 구독·발행 준비, arti/start_tour 액션 서버 열기
  └ 상태 READY  → cmd_vel을 보내지 않고 액션을 기다림
```

`auto_start:=true`면 액션 없이 1초 뒤 바로 관람을 시작한다 (테스트용).

## 2. 키오스크가 `start_tour`를 보내면

```
goal {mode: FREE}
  ├ 이미 관람 중      → 바로 끝냄: success=false, message=busy (진행 중 관람은 그대로)
  ├ mode가 COURSE 등  → 바로 끝냄: success=false, message=unsupported_mode
  └ FREE 또는 ""      → 관람 시작
```

## 3. 관람 흐름 (상태)

```
READY
  │ start_tour
  ▼
UNDOCK        도크에 붙어 있으면 Create 3 undock 액션 (로봇이 스스로 움직임). 도크 밖이면 건너뜀
  │             실패 → 관람 끝 (undock_failed / no_undock_server)
  ▼
LOCALIZE      AMCL에 initialpose(출발 지점)를 보내고 map 위치가 근처(0.3 m)로 잡힐 때까지 기다림
  │             10 s 안에 map 위치가 아예 없으면 → 관람 끝 (no_localization)
  ▼
START_FACE    제자리 회전으로 키오스크 쪽을 봄 (15 s 안에 못 돌면 그냥 넘어감)
  ▼
WAIT_VISITOR  관람객 인식 기다림. 이 전까지 들어온 좌표는 버림
  │             좌표가 1 s 동안 끊김 없이(간격 0.5 s 이하) 들어오면 → 등록 완료
  │             확인 중에는 따라가지 않고 바라보기만. 끊기면 처음부터 다시
  ▼ on_registered(): registered=true, guide_message "recognized" ("인식되었습니다")
FOLLOW        관람객과 0.8 m 거리 유지하며 이동
  │ 좌표가 1.5 s(lost_timeout) 안 옴
  ▼
SEARCH        마지막 좌표까지 이동(0.35 m 안, 최대 5 s) → 거기서 0.63 rad/s(2π/10 s) 일정 속도로 한 바퀴(odom 360°)
  ▼
WAIT          정지 대기 10 s
  ▼
LOST          관람 끝 (lost)

SEARCH·WAIT 중 좌표가 다시 오면 → FOLLOW
액션 취소(cancel)는 언제든 → 정지 → READY (canceled)

FOLLOW·SEARCH·WAIT 중 관람객이 출구 구역(end_zone: exit)에 들어감 (artwork_zone/entered)
  ▼ 관람객 데이터 삭제 (arti/tour_end), feedback RETURNING
RETURN        언독 지점(init_pose.yaml undock = 도크 앞)으로 이동. 목표가 57° 넘게 틀어져 있으면 먼저 제자리 회전
  │             (출구에서 바로 dock하면 Create 3가 도크를 못 찾음) 90 s 안에 못 가면 → 관람 끝 (return_failed)
  ▼
DOCK_FACE     제자리에서 도크(init_pose.yaml dock) 쪽을 봄
  ▼
DOCK          Create 3 dock 액션 → 관람 끝 (success=true, finished) → READY
                실패 → dock_failed, 서버 없음 → no_dock_server
```

관람이 끝나면(취소·LOST·실패) 정지하고 `arti/tour_end`를 한 번 보낸다.
인식 모듈·`artwork_zone`·이 노드가 그 관람객 데이터(re-ID 특징, 좌표 기록, 필터)를 지운다 (로봇 1대 = 관람객 1명, 개인정보).

## 4. FOLLOW에서 움직이는 방식

```
좌표   칼만 필터(등속)로 위치·속도를 매끄럽게. 예측과 너무 다른 좌표는 버림 (5번 연속이면 다시 시작)
목표   관람객이 0.3 s 뒤 있을 곳에서 0.8 m 앞 지점
멈춤   0.85 m 안이면 감속해 서서 관람객 쪽으로 돎, 0.95 m 밖이 되면 다시 출발 (섰다 갔다 방지)
회피   LiDAR(벽·기둥)로 10 cm 격자 (OAK-D depth는 2026-11-18에 뺌)
       → 칸마다 목표까지 돌아가는 거리(BFS) → (v, w) 후보를 1.5 s 굴려 보고 안 부딪히며 가장 가까워지는 것 (DWA)
속도   최대 0.3 m/s, 0.8 rad/s. 후진하지 않음
안전   TF(odom→base_link)가 없거나 scan이 0.5 s 넘게 안 오면 정지
```

`follow/enable`이 false면 계산은 하지만 `cmd_vel`을 보내지 않는다 (다른 노드에 주행을 넘길 때).

## 5. 키오스크가 받는 것 (`start_tour` feedback, 0.2 s마다)

| 내부 상태 | feedback `state` | `registered` |
|---|---|---|
| READY·UNDOCK·LOCALIZE·START_FACE | MOVING | false |
| WAIT_VISITOR | ARRIVED | false |
| FOLLOW·SEARCH·WAIT | FOLLOW | true |
| 위 상태에서 작품 설명 중 | EXPLAINING | true |

- `current_artwork_id`: 설명 중인 작품 (art_02 → 2, 없으면 0)
- `current_index` / `total`: 끝까지 설명한 작품 수 / artworks.yaml 작품 수
- result: `success`(취소로 끝나면 true), `message`(끝난 이유), `visited_ids`(끝까지 설명한 작품, 순서대로)

**등록 성공 보고 = `registered`가 처음 true가 되는 feedback.**

## 6. 주고받는 토픽·액션 (`/robot1` 아래)

| 방향 | 이름 | 타입 | 상대 |
|---|---|---|---|
| 액션 서버 | `arti/start_tour` | arti_interfaces/StartTour | 키오스크·관제 |
| 입력 | `target_person/point_map` | PointStamped (map) | person_tracker |
| 입력 | `scan` | LaserScan | 로봇 LiDAR |
| 입력 | `tf`, `tf_static` | | 로봇, AMCL |
| 입력 | `dock_status` | DockStatus | 로봇 |
| 입력 | `follow/enable` | Bool | (필요할 때) |
| 입력 | `artwork_zone/explaining`, `artwork_zone/explained` | String | artwork_zone |
| 입력 | `artwork_zone/entered` | String | artwork_zone (출구 진입 → 복귀·도킹) |
| 액션 클라이언트 | `undock` | irobot_create_msgs/Undock | 로봇 |
| 액션 클라이언트 | `dock` | irobot_create_msgs/Dock | 로봇 (관람 끝, 도크 앞에서) |
| 출력 | `cmd_vel` | TwistStamped | 로봇 |
| 출력 | `initialpose` | PoseWithCovarianceStamped | AMCL |
| 출력 | `follow/state` | String (latched, 바뀔 때만) | artwork_zone, 확인용 |
| 출력 | `guide_message` | String | guide_audio_node |
| 출력 | `arti/tour_end` | Empty | person_tracker, artwork_zone |

## 7. 실행과 확인 (하나씩)

전체 관람 (인식 + 추종 + 작품 구역 + 음성). AMCL은 미리 떠 있어야 한다.
```bash
ros2 launch arti_follow tour.launch.py
```

관람 시작 (**로봇이 undock하고 회전하니 주변이 안전할 때**). Ctrl+C = 취소
```bash
ros2 action send_goal --feedback /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}"
```

상태 보기
```bash
ros2 topic echo /robot1/follow/state
```

## 8. 자주 바꾸는 파라미터 (`-p 이름:=값`, 전체는 노드 맨 위 `PARAMS`)

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `target_distance` | 0.8 | 유지 거리 [m] |
| `register_confirm_time` / `register_gap` | 1.0 / 0.5 | 등록 확인 시간 / 끊김으로 보는 간격 [s] |
| `lost_timeout` | 1.5 | 좌표가 이만큼 안 오면 SEARCH [s] |
| `search_time` / `wait_time` | 10 / 10 | 한 바퀴 회전 탐색(속도 = 2π/search_time) / 정지 대기 [s] |
| `search_move_time` / `search_arrive` | 5.0 / 0.35 | 마지막 좌표로 이동 시간 상한 [s] / 도착 거리 [m] |
| `lead_time` | 0.3 | 관람객 속도로 이만큼 앞을 보고 목표를 잡음 [s] |
| `kf_accel_std` / `kf_meas_std` | 2.0 / 0.2 | 칼만 가속도 잡음 / 좌표 잡음 |
| `max_linear` / `max_angular` | 0.3 / 0.8 | 최대 속도 [m/s] / [rad/s] (빨리 돌면 탑캠이 사람을 놓침) |
| `linear_accel` / `linear_decel` / `angular_accel` | 0.8 / 0.8 / 1.5 | 가속·감속 [m/s^2], 각가속 [rad/s^2] |
| `min_spin_cmd` | 0.1 | 서 있을 때 이보다 작은 회전 명령은 0 (Create 3는 작은 제자리 회전을 0.25 rad/s 이상으로 돎) [rad/s] |
| `face_zone` | kiosk | 시작 때 바라볼 구역 |
| `undock` | true | 시작 때 undock |
| `return_on_exit` / `end_zone` | true / exit | 관람객이 이 구역에 들어가면 도크 앞으로 복귀·도킹 |
| `return_point` / `dock_point` | undock / dock | init_pose.yaml에서 먼저 갈 곳 / 바라볼 도크 |
| `return_arrive` / `return_timeout` | 0.15 / 90 | 도착 거리 [m] / 복귀 시간 상한 [s] |
| `dock` / `dock_timeout` | true / 120 | 도킹 할지 / dock 액션 대기 [s] |
| `turn_first_angle` | 1.0 | 정해진 지점으로 갈 때 목표가 이보다 틀어져 있으면 먼저 제자리 회전 [rad] (DWA는 후진이 없어 뒤쪽 목표에서 멈춤) |
| `auto_start` | false | 액션 없이 바로 시작 (테스트) |

## 근거

알고리즘·파라미터를 정한 이유와 bag 수치: `visitor_follower_주행근거.docx`

## 한계

- 후진하지 않는다. U자 막다른 곳은 못 빠져나올 수 있고, 관람객이 로봇 쪽으로 다가오면 0.8 m보다 가까워진다
- TurtleBot 4 최고 속도가 약 0.3 m/s라 관람객이 그보다 빨리(0.4~0.6 m/s) 걸으면 거리가 벌어진다 (2026-11-18 bag)
- 바닥에 떨어진 낮은 물체(LiDAR 높이 아래)는 못 본다 (depth 제거)
- 목표까지 길이 없으면(LiDAR 점에 막힘) 경고 없이 선다 (`dwa()` 동점 처리)
- 등록은 "1 s 동안 계속 보임"까지만 본다. 특징 수집·한 명 확인(T13)은 아직 없다
- 도크로 돌아가는 것은 출구 진입으로 끝날 때만이다. 취소·LOST로 끝나면 그 자리에 선다
