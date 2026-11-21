# visitor_follower 실행하면 일어나는 일

노드 하나(`visitor_follower`), 파일은 기능별로 넷 (`src/arti_follow/arti_follow/`):

| 파일 | 맡는 것 |
|---|---|
| `visitor_follower.py` | 노드 본체: 파라미터, 구독·발행·액션 서버 등록, 입력 콜백(`on_person` 등), `control`/`step`(상태 분기), 진단 |
| `prepare_tour.py` | ① 호출·등록 `prepare_tour`: `run_prepare`, `undock`, `localize`, `face_start`, `wait_visitor`, `registered_wait` |
| `start_tour.py` | ② 관람 `start_tour`: `run_tour`, `follow`, `search`, `end_tour`, 작품 설명 feedback |
| `motion.py` | 주행·추정 공통: `PersonKF`(칼만 필터), `face`, `obstacles`·`field`·`dwa`, `ramp`, `drive` |

`class VisitorFollower(PrepareMixin, TourMixin, MotionMixin, Node)` — 세 파일의 함수를 상속해서 한 노드로 쓴다 (`self.state`, lock 공유).

로봇 1대(`/robot1`)가 관람객 1명을 맡아
**관람 시작 → 등록 → 추종 → 분실 처리 → 관람 끝**까지 움직임을 담당한다.
작품 설명 판단은 `artwork_zone`, 음성은 `arti_audio`의 `guide_audio_node`, 사람 인식은 `person_tracker`가 한다.

## 1. 켜자마자

```
노드 시작
  ├ init_pose.yaml   → 출발 지점 (x, y, yaw)        (robots.<네임스페이스>.undock)
  ├ artworks.yaml    → 바라볼 곳 (kiosk 구역 중심), 작품 수 (feedback total)
  ├ 구독·발행 준비, arti/prepare_tour·arti/start_tour 액션 서버 열기
  └ 상태 READY  → cmd_vel을 보내지 않고 액션을 기다림
```

`auto_start:=true`면 액션 없이 1초 뒤 prepare → start를 차례로 한다 (테스트용).

## 2. 키오스크가 보내는 액션 두 개 (팀 명세 `docs/common/ros2_interfaces.md`)

```
키오스크 쪽 규칙 (그래서 로봇은 busy·not_prepared 검사를 하지 않는다)
  - 첫 화면의 [호출]은 배터리가 30%를 초과하고 도킹된 로봇이 있을 때 켠다.
  - 자유 관람은 robot1에 배정하고, prepare_tour 성공 직후 start_tour를 자동 요청한다.

① prepare_tour {start: true}   호출·등록
  ├ start = false     → 바로 끝냄: success=false, message=not_requested
  └ 그 밖             → undock → 위치 잡기 → 키오스크 보기 → 관람객 등록
                         등록되면 success=true, message=registered  → 키오스크가 start_tour 자동 요청
                         실패: dock_failed (도크 밖인데 도킹 못 함) / undock_failed / no_undock_server / dock_status_unavailable / canceled

② start_tour {artwork_ids: []}  관람 시작 (prepare_tour 성공 뒤에만 옴)
  └ 바로 추종 시작. 관람 끝까지 feedback, 끝나면 result (artwork_ids는 보지 않음)
```

## 3. 관람 흐름 (상태)

```
READY
  │ prepare_tour                                       ┐
  ▼                                                    │
UNDOCK        Create 3 undock 액션 (로봇이 스스로 움직임). 관람은 항상 도크에서 시작
  │             도크 밖이면 먼저 Create 3 dock으로 올린 뒤 undock (dock_first), 실패 → 관람 끝 (dock_failed / undock_failed)
  ▼
LOCALIZE      바로 다음 (initialpose = dock은 UNDOCK 직전 도크 위에서 이미 보냄, AMCL 수렴은 안 기다림)
  ▼
START_FACE    언독 지점 → 키오스크 방향으로 돌 각도를 계산해 odom 기준으로 그만큼 제자리 회전
              (map 위치 안 씀, face_offset_deg로 보정, 15 s 안에 못 돌면 그냥 넘어감)
  ▼
WAIT_VISITOR  관람객 인식 기다림. 이 전까지 들어온 좌표는 버림
  │             좌표가 1 s 동안 끊김 없이(간격 0.5 s 이하) 들어오면 → 등록 완료
  │             확인 중에는 따라가지 않고 바라보기만. 끊기면 처음부터 다시
  │             등록 방식 = 인식 노드 (launch tracker 인자, 둘 다 person_tracker BaseTrackerNode 상속)
  │               gesture(기본) gesture_test_node: 로봇 앞에서 주먹 → 손바닥 펴기 (auto_register 끔)
  │               front         front_tracker_node: 좌표가 안 오는 동안 1 s마다 target_person/register_front
  │                             → 로봇 정면 ±30°, 2.5 m 안 가장 가까운 사람 자동 등록 (인식 창 클릭도 됨)
  ▼ on_registered(): registered=true, guide_message "recognized" ("인식되었습니다")
REGISTERED    prepare_tour 성공 (result success=true). 그 자리에 서서 기다림 (돌지도 따라가지도 않음) ┘ prepare_tour
  │             130 s(ready_timeout) 안에 start_tour가 안 오면 → 관람 끝 (not_started)
  │ start_tour                                         ┐
  ▼                                                    │ start_tour
FOLLOW        관람객과 0.8 m 거리 유지하며 이동
  │ 좌표가 1.5 s(lost_timeout) 안 옴
  ▼
SEARCH        마지막 좌표까지 이동(도착할 때까지. 3 s 동안 안 가까워지면 막힘, 상한 20 s. 뒤쪽이면 먼저 제자리 회전) → 거기서 odom 기준 한 바퀴(약 20 s, 0.31 rad/s) 회전 탐색
  ▼
WAIT          정지 대기 10 s
  ▼
LOST          관람 끝 (lost)

SEARCH·WAIT 중 좌표가 다시 오면 → FOLLOW
액션 취소(cancel)는 언제든 → 정지 → 관람 끝 (canceled)

관람객이 출구 구역(artworks.yaml exit) 진입 (start_tour 뒤 10 s 지나서, end_zone)
  ▼ artwork_zone: guide_message "tour_end"(종료 안내 음성) + artwork_zone/entered "exit"
  ▼ visitor_follower: 3 s(end_guide_wait) 서서 관람객 쪽을 봄 → 관람 끝 (completed, success=true)
관람 끝 (출구 completed·취소·분실·등록 뒤 start_tour 안 옴 등, 이번에 언독했을 때만, return_to_dock)
  ▼
RETURN        언독이 끝난 곳(odom 기록)까지 이동 (뒤쪽이면 먼저 회전, 최대 60 s)
  ▼
DOCK_FACE     도크 쪽(언독 지점에서 등 뒤)으로 제자리 회전
  ▼
DOCK          Create 3 dock 액션 (스스로 IR로 도크에 올라감, 최대 90 s) → READY / LOST
              그동안 start_tour feedback = RETURNING, 액션 result는 도킹이 끝난 뒤에 나감
```

### 3-1. step·on_person은 누가 시작시키나

액션(`run_prepare`, `run_tour`)이 부르지 않는다. 노드가 켜질 때 `__init__`에서 등록되어 **처음부터 계속 돈다.**
액션은 `self.state`만 바꾸고, 둘은 그 상태를 보고 할 일을 정한다.

```
__init__ (노드 켤 때 등록)
  ├ sub('target_person/point_map', on_person)   좌표 올 때마다 → on_person()
  └ create_timer(0.1, control)                  0.1 s마다     → control() → step()
        │
        ▼
executor.spin()  ← 여기서부터 계속 호출됨 (관람 전에도)

               ┌────────────── self.state (게시판) ──────────────┐
 run_prepare ──┤ begin_tour()        → UNDOCK                    │  상태를 "쓰는" 쪽
 (prepare_tour)│ undock 끝           → LOCALIZE                  │
 run_tour ─────┤ start_tour 받음     → FOLLOW (REGISTERED일 때만) │
 (start_tour)  └─────────────────────────────────────────────────┘
                     │ 읽음                       │ 읽음
                     ▼                            ▼
 step() (0.1 s마다)                     on_person() (좌표 올 때마다)
   READY·UNDOCK → [1] 바로 return          state ∈ {WAIT_VISITOR, REGISTERED, FOLLOW, SEARCH, WAIT}?
   LOCALIZE     → localize()                 아니오 → 좌표 버림
   START_FACE   → face_start()               예     → kf.x 갱신, seen 시각 기록
   WAIT_VISITOR → wait_visitor()  (등록되면 REGISTERED)
   REGISTERED   → registered_wait() (정지, 130 s 지나면 끝)
   FOLLOW·SEARCH·WAIT → 추종/탐색/대기
```

```
시간 →      노드 켬 ─ prepare 도착 ─ undock 끝 ─ 키오스크 봄 ─ 등록 ─── start 도착 ──▶
state        READY    UNDOCK        LOCALIZE    WAIT_VISITOR  REGISTERED   FOLLOW
step         할 일 없음 할 일 없음     위치 잡기    등록 확인      서서 기다림    따라가기
on_person    버림      버림          버림         저장           저장          저장
run_prepare            begin_tour → undock → 0.2 s마다 feedback → success ┘
run_tour                                                                  └ 0.2 s마다 feedback ──▶
```

관람이 끝나면(취소·LOST·실패) 정지하고 `arti/tour_end`를 한 번 보낸다.
인식 모듈·`artwork_zone`·이 노드가 그 관람객 데이터(re-ID 특징, 좌표 기록, 필터)를 지운다 (로봇 1대 = 관람객 1명, 개인정보).

## 4. FOLLOW에서 움직이는 방식

```
좌표   칼만 필터(등속)로 위치·속도를 매끄럽게. 예측과 너무 다른 좌표는 버림 (5번 연속이면 다시 시작)
목표   관람객이 0.3 s 뒤 있을 곳에서 0.8 m 앞 지점
멈춤   0.85 m 안이면 감속해 서서 관람객 쪽으로 돎, 0.95 m 밖이 되면 다시 출발 (섰다 갔다 방지)
회피   LiDAR(벽·기둥)만으로 10 cm 격자 (OAK-D depth는 2026-11-18에 뺌: 빈 바닥이 장애물로 잡혀 멀리 있는 관람객을 못 따라감)
       → 칸마다 목표까지 돌아가는 거리(최단 거리, SciPy 다익스트라 `grid_distances`) → (v, w) 후보를 1.5 s 굴려 보고 안 부딪히며 가장 가까워지는 것 (DWA)
속도   최대 0.3 m/s, 0.8 rad/s. 후진하지 않음. 서 있을 때 0.1 rad/s보다 작은 회전은 0 (Create 3가 작은 회전을 크게 돎)
안전   TF(odom→base_link)가 없거나 scan이 0.5 s 넘게 안 오면 정지
```

`start_enabled:=false`(launch `drive:=false`)면 계산은 하지만 `cmd_vel`을 보내지 않는다 (시험용).

## 5. 키오스크가 받는 것 (feedback 0.2 s마다)

`prepare_tour` (PrepareTour)

| 내부 상태 | feedback `state` |
|---|---|
| UNDOCK | UNDOCKING |
| LOCALIZE·START_FACE | MOVING |
| WAIT_VISITOR (아무도 안 보임) | ARRIVED |
| WAIT_VISITOR (1 s 확인 중) | REGISTERING |
| REGISTERED | REGISTERED → result `success=true`, `message=registered` |

**등록 성공 보고 = prepare_tour result `success=true`.**

`start_tour` (StartTour): feedback = `state` + `artwork_id`, result = `success` + `message`

| 내부 상태 | feedback `state` | `artwork_id` |
|---|---|---|
| FOLLOW·SEARCH·WAIT·LOST | FOLLOW | 0 |
| 위 상태에서 작품 설명 중 (`artwork_zone/explaining`) | EXPLAINING | 설명 중인 작품 번호 |
| 작품을 끝까지 설명함 (`artwork_zone/explained`) | ARTWORK_COMPLETED (작품마다 한 번) | 그 작품 |

- result: `success`(취소로 끝나면 true), `message`(끝난 이유: canceled / lost)
- 등록 뒤 start_tour가 130 s 안에 안 오면 관람 끝(`not_started`, arti/tour_end). 이때는 진행 중인 액션이 없어 result가 없다.

## 6. 주고받는 토픽·액션 (`/robot1` 아래)

| 방향 | 이름 | 타입 | 상대 |
|---|---|---|---|
| 액션 서버 | `arti/prepare_tour` | arti_interfaces/PrepareTour | PC3 키오스크·관제 |
| 액션 서버 | `arti/start_tour` | arti_interfaces/StartTour | PC3 키오스크·관제 |
| 입력 | `target_person/point_map` | PointStamped (map) | person_tracker |
| 입력 | `scan` | LaserScan | 로봇 LiDAR |
| 입력 | `oakd/stereo/image_raw/compressedDepth`, `oakd/stereo/camera_info` | | 로봇 OAK-D |
| 입력 | `tf`, `tf_static` | | 로봇, AMCL |
| 입력 | `dock_status` | DockStatus | 로봇 |
| 입력 | `artwork_zone/explaining`, `artwork_zone/explained` | Int32 (작품 번호, 0 = 없음) | artwork_zone |
| 액션 클라이언트 | `undock` | irobot_create_msgs/Undock | 로봇 |
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

호출·등록 (**로봇이 undock하고 회전하니 주변이 안전할 때**). `success: true`가 나오면 등록 완료
```bash
ros2 action send_goal --feedback /robot1/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}"
```

관람 시작 (등록 뒤 130 s 안에). Ctrl+C = 취소
```bash
ros2 action send_goal --feedback /robot1/arti/start_tour arti_interfaces/action/StartTour "{artwork_ids: []}"
```

상태 보기
```bash
ros2 topic echo /robot1/follow/state
```

## 8. 자주 바꾸는 파라미터 (`-p 이름:=값`, 전체는 `src/arti_follow/config/visitor_follower.json`, 값마다 설명 포함)

| 이름 | 기본값 | 뜻 |
|---|---|---|
| `target_distance` | 0.8 | 유지 거리 [m] (0.85 m 안이면 서고 0.95 m 밖이면 다시 출발) |
| `register_confirm_time` / `register_gap` | 1.0 / 0.5 | 등록 확인 시간 / 끊김으로 보는 간격 [s] |
| `lost_timeout` | 1.5 | 좌표가 이만큼 안 오면 SEARCH [s] |
| `search_move_time` / `search_arrive` | 20 / 0.35 | 마지막 위치로 이동하는 안전 시간 상한 [s] / 도착 거리 [m] |
| `person_wall_margin` / `map_file` | 0.0 / '' | 관람객 좌표가 지도의 벽(이만큼 넓혀 봄)·지도 밖이면 버림 [m] / 지도 yaml ('' = $ARTI_MAP) |
| `wall_leg` / `wall_leg_angle` / `wall_leg_margin` | true / 0.17 / 0.15 | 벽 안 좌표를 버리기 전에 같은 방향(이 각도 [rad] 안)에서 벽보다 이만큼 [m] 앞선 라이다 다리 후보가 있으면 그 위치로 대체 (인식 모듈이 사람 뒤 벽 거리를 잰 경우, 2026-11-21) |
| `reacquire_radius` / `reacquire_speed` / `reacquire_max` | 1.0 / 1.0 / 4.0 | 놓친 뒤 다시 잡을 때 마지막 확인 위치에서 허용 거리 = radius + speed × 지난 시간 (최대 max) [m, m/s, m]. 넘으면 버림 |
| `search_confirm_time` | 0.5 | 탐색 중 다시 보이면 이만큼 끊김 없이 보여야 FOLLOW [s] (그동안 마지막 위치 탐색 계속) |
| `search_stall_time` / `search_stall_dist` | 3 / 0.05 | 이 시간 동안 경로 거리(장애물을 돌아가는 거리)가 이만큼도 안 줄면 막힘 → 그 자리에서 회전 탐색 [s] / [m] |
| `robot_radius` / `safety_margin` | 0.18 / 0.02 | 로봇 반지름 / 여유 [m]. 합 0.20 안을 지나는 궤적 금지, 통과 가능한 최소 폭 약 0.65 m (0.20/0.05일 때 약 0.85 m) |
| `search_time` / `wait_time` | 20 / 10 | 회전 탐색 한 바퀴 시간 (속도 = 2π/search_time = 0.31 rad/s) / 정지 대기 [s] |
| `max_linear` / `max_angular` | 0.25 / 0.5 | 최대 속도 [m/s] / [rad/s] (빨리 돌면 탑캠 영상이 흔들려 인식을 놓침) |
| `face_zone` | kiosk | 시작 때 바라볼 구역 |
| `face_offset_deg` | -15 | 시작 때 바라볼 방향 보정 [deg] (+ 왼쪽, - 오른쪽) |
| `undock` | true | 시작 때 undock |
| `auto_start` | false | 액션 없이 바로 prepare → start (테스트) |
| `ready_timeout` | 130 | 등록 뒤 start_tour를 기다리는 시간 [s] |
| `return_to_dock` | true | 관람이 끝나면 언독 지점으로 돌아와 도킹 |
| `end_zone` / `end_guide_wait` / `end_min_time` | exit / 3 / 10 | 이 구역에 들어가면 관람 끝 / 종료 안내 듣는 시간 [s] / start_tour 뒤 이 시간 안 진입은 무시 [s] |
| `return_timeout` / `dock_timeout` | 60 / 90 | 언독 지점에 못 가면 이 주기마다 경고하고 계속 감 (도킹은 언독 지점 도착 뒤에만, 2026-11-21) / Create 3 도킹 대기 [s] |
| `detour_margin` / `w_path_align` | 0.3 / 0.15 | 경로 거리가 직선보다 이만큼 [m] 넘게 길거나 길 방향이 사람 방향에서 `view_half_angle` 넘게 벗어나면 '벽 너머' → 화면 유지 벌점(`w_view`) 끔 / 1.5 s 뒤 방향이 길 방향과 어긋난 각도 [rad]당 벌점 (모퉁이에서 제자리 회전을 고르게, 2026-11-21) |
| `follow_progress_first` / `follow_progress_min` / `follow_progress_ratio` / `follow_view_hold` / `view_off_on_detour` | true / 0.05 / 0.5 / 2.0 / true | **따라가기 우선**: 화면 유지 벌점 때문에 고른 궤적이 경로 거리를 거의 못 줄이면(min [m] 이하 또는 가장 잘 가는 후보의 ratio 이하) 화면 벌점 없이 다시 고르고, hold [s] 동안 계속 포기 (관람객이 잠깐 화면 밖으로 나가도 먼 길로 돌아감, 2026-11-21) / 벽 너머 판단이면 화면 벌점 끔 |
| `return_stall_time` | 5 | 도크 복귀 중 이만큼 경로 거리가 안 줄면 막힘 경고. 처음 막히면 지도 벽을 빼고 라이다만 보며 계속 감 [s] |
| `path_fallback_lidar` / `path_heading_radius` | true / 0.3 | 지도 벽까지 넣어 길이 없으면 라이다만으로 다시 길 찾기 / go_to가 '먼저 돌기'를 직선 대신 길 방향(로봇 둘레 이 반지름 [m])으로 판단 (2026-11-21: 벽 너머 목표에서 제자리 흔들림) |

## 한계

- 후진하지 않는다. U자 막다른 곳은 못 빠져나올 수 있다
- LiDAR 평면보다 낮은 바닥 물체(가방 등)는 못 본다 (depth를 뺐음)
- 등록은 "1 s 동안 계속 보임"까지만 본다. 특징 수집·한 명 확인(T13)은 아직 없다
- 도크 복귀 목표는 지도의 언독 지점(`init_pose.yaml`)을 그때의 AMCL(map→odom)로 바꾼 값 (`return_use_map`, 2026-11-20).
  AMCL이 없으면 언독 때 기록한 odom 위치로 돌아간다 (관람 중 바퀴 오차가 쌓여 20~30 cm 어긋나 도킹 실패가 잦았음).
