# arti_pc2_nav — robot4 코스 관람 (PC2)

키오스크(PC3)에서 **로봇 호출**과 **관람 시작**을 받아, robot4가 관람객을 등록하고 선택한 작품을 차례로 안내한 뒤 도킹한다.
이동은 전부 **Nav2**(`navigate_to_pose`)가 하고, 이 패키지는 `cmd_vel`을 보내지 않는다 (arti_follow는 `cmd_vel` 직접 제어 — 방식이 다름).
PC 간 약속은 팀 명세서(`docs/common/ros2_interfaces.md`, CLAUDE.md "PC 간 인터페이스" 표) 그대로다.

## 전체 흐름

```
① 키오스크 [로봇 호출]  PC3 ──/robot4/arti/prepare_tour {start: true}──▶ a_course_node
     feedback  UNDOCKING → MOVING → ARRIVED → REGISTERING → REGISTERED
     언독 → Nav2로 키오스크 → 정지 확인 → guide 'course_register'(주먹→손바닥 안내)
     → person_tracker target_person/point_map 1초 연속 → 등록 (인식 완료 안내 없음, 바로 ②의 course_start)
     result    success=true, message='registered'   → 키오스크가 곧바로 ② (130초 안에 ②가 없으면 도킹 복귀)

② 준비 성공 직후 자동 시작  PC3 ──/robot4/arti/start_tour {artwork_ids: [1, 3, 4]}──▶ a_course_node
     guide 'course_start'(저를 따라오세요) → 작품마다 (YAML 코스 순서):
        feedback MOVING(작품 번호) → 반원 진입·정지 → EXPLAINING(번호) → ARTWORK_COMPLETED(번호)
        작품 설명: explain_artwork(번호) → guide_audio_node → PC3 /arti/description(번호) → edge-tts 재생
     → 출구 (guide 'tour_end') → feedback RETURNING → 도킹 접근점 → Dock → dock_status 확인
     result    success=true, message='completed'   (도킹이 끝난 뒤)
```

- **위치는 로봇 PC 고정**: 키오스크·작품·도킹 좌표는 `config/a_course.yaml`. PC3는 작품 **번호**만 보낸다.
- **출구 구역은 follow 팀과 공통**: `arti_common/config/artworks.yaml`의 `exit` 직사각형(x -0.82~0.13, y 1.36~1.91)을 `arti_common.zones`(follow 팀과 같은 코드)로 읽어서 도착 판정과 RViz 표시에 쓴다.
- **작품 번호** = `a_course.yaml`의 `artwork_id` = PC3 DB 번호 (int). 설명 문장은 PC3 DB에만 있다.
- `artwork_ids: []`이면 전체 작품. 순서는 요청 순서가 아니라 YAML 코스 순서(동선대로).

### 이동 중 관람객 감시 (start_tour)

robot4는 앞장서고 관람객이 뒤에서 따라온다. 노트북 웹캠을 **뒤쪽**으로 올려 둔다 (`camera_yaw:=180`).

| 상황 | 동작 |
|---|---|
| 관람객이 `far_distance`(2.5 m)보다 멀어짐 / `lost_timeout`(3 s) 넘게 안 보임, 이 상태가 `far_hold`(1 s) 계속 | Nav2 goal 취소(정지) + guide `course_come_here`(뒤쪽 카메라 앞으로 와 주세요, `come_here_interval` 8 s에 한 번까지) (feedback은 MOVING 유지) |
| 다시 `resume_distance`(1.8 m) 안으로 옴, `near_hold`(0.5 s) 계속 | 같은 목표를 Nav2에 다시 보내 이어서 이동 |

거리는 최근 `distance_window`(1 s) 동안 받은 관람객 좌표의 **중앙값**으로 잰다. 인식 위치가 가끔 튀어도(카메라 추정·LiDAR 매칭이 번갈아 나옴) 멈춤·출발이 반복되지 않게 하려는 것.
| 멈춘 뒤 `visitor_give_up`(10 s) 동안 안 옴 | guide `register_failed` → 도킹 복귀 → result `visitor_lost` |

판단 값(1.0 s 등록, 0.5 s 끊김, 3 s 분실, 0.8+0.15 m)은 arti_follow와 같다. 주행 값(속도·가감속)은 Nav2 설정을 따른다.

### 실패·취소 — 모두 도킹 복귀

| 상황 | prepare_tour result | start_tour result |
|---|---|---|
| 출발 전 점검 실패 (enable_motion, 도크 상태, 배터리, odom, Dock 서버, explain_artwork 서버 …) | `false`, 이유 (움직이지 않음) | `false`, 이유 (Nav2·AMCL도 봄) |
| 언독 후 60 s 안에 scan·AMCL·TF·Nav2가 준비 안 됨 | `false`, `not_ready: …` → **그 자리에서 Dock으로 재도킹** (Nav2 안 씀) | — |
| prepare 없이 start_tour | — | `false`, `not_prepared` |
| 키오스크에서 60 s 안에 등록 못 함 | `false`, `register_failed` (복귀 시작하자마자) | — |
| 등록 뒤 130 s 동안 start_tour 없음 | (이미 success) → 도킹 복귀 | — |
| PC3 Action Cancel | `canceled` (언독 중이면 언독 끝난 뒤, 키오스크로 출발 전이면 바로 재도킹) | 도킹 후 `canceled` |
| Nav2 이동 실패 | `false`, `nav_failed` | 도킹 후 `false`, `nav_failed` |
| 작품 설명 실패 (PC3 무응답 등) / 복귀 중 Nav2 실패 / 도킹 실패 | `false` (ERROR, 그 자리 정지) | `false` (ERROR, 그 자리 정지) |

## 노드와 인터페이스

`a_course.launch.py`가 띄우는 것 (모두 `robot_namespace`, 기본 `robot4` 안):

| 노드 | 패키지 | 역할 |
|---|---|---|
| `a_course_node` | arti_pc2_nav | 상태 기계 실행. 액션 서버 2개, Nav2·Undock·Dock·explain_artwork 클라이언트 |
| `arti_zone_markers` | arti_pc2_nav | RViz 구역 마커 (`arti/a_course/zones`): 키오스크·작품 반원 + 출구 직사각형(arti_common) |
| `guide_audio_node` | arti_audio | 안내·작품 설명 음성 (`audio:=true`) |
| `top_camera_publisher_node`, `top_cam_tf_node` | person_tracker | 노트북 웹캠 → `top_cam/image_raw/compressed`, 카메라 TF (`tracker:=true`) |
| `multi_person_tracker_node` | person_tracker | YOLO 인식 + LiDAR → `target_person/point_map` (`tracker:=true`) |

`a_course_node` (namespace 안 상대 이름):

| 종류 | 이름 | 타입 | 방향 |
|---|---|---|---|
| 액션 서버 | `arti/prepare_tour` | `arti_interfaces/action/PrepareTour` | PC3 → 여기 |
| 액션 서버 | `arti/start_tour` | `arti_interfaces/action/StartTour` | PC3 → 여기 |
| 액션 클라이언트 | `explain_artwork` | `arti_interfaces/action/ExplainArtwork` (작품 번호 int) | 여기 → guide_audio_node |
| 액션 클라이언트 | `navigate_to_pose`, `undock`, `dock` | Nav2, `irobot_create_msgs` | 여기 → 로봇 |
| 구독 | `target_person/point_map` | `geometry_msgs/PointStamped` | person_tracker → 여기 |
| 구독 | `amcl_pose`, `odom`, `scan`, `battery_state`, `dock_status`, `tf` | | 로봇 → 여기 |
| 발행 | `guide_message` | `std_msgs/String` (guide_lines.yaml 키) | 여기 → guide_audio_node |
| 발행 | `arti/tour_end` | `std_msgs/Empty` | 여기 → person_tracker (등록 대상 지우기) |
| 발행 | `/arti/course_status` | `std_msgs/String` JSON (전역) | 로그·디버그용. PC3와의 약속 아님 |

안내 키(`guide_message`)는 `arti_audio/config/guide_lines.yaml`에 있어야 소리가 난다:
robot4 전용 `course_register`, `course_start`, `course_come_here`(follow 팀 문장과 따로) + 공용 `tour_end`, `register_failed` (키 이름은 파라미터로 바꿀 수 있음).

## 초기 대비 바뀐 점 (코스 관람, 2026-11-20)

11/19 첫 실주행 뒤 고친 것 (⑪은 11/20 실주행 뒤). 코드에는 같은 번호로 `[코스 관람 변경 ③]`처럼 표시해 두었다 → `grep -rn "코스 관람 변경 ③" src/`.

| # | 바뀐 것 | 왜 (전에 생긴 문제) | 어디를 보면 되나 |
|---|---|---|---|
| ① | launch 하나로 localization + Nav2 + 코스 노드 | 터미널 3개를 순서대로, RViz 2D Pose를 손으로 | `launch/a_course.launch.py` `_localization_and_nav2` |
| ② | AMCL 시작 = `init_pose.yaml` undock, 언독 직후 `initialpose` 발행 | 도킹 중엔 LiDAR가 꺼져 2D Pose Estimate가 안 먹음 | `a_course_node.py` `_send_initial_pose`, `_load_init_poses` / `guided_tour_plan.py` `load_start_pose` / `arti_common/config/init_pose.yaml` |
| ③ | 호출 점검 두 단계 (도킹 중엔 도크 복귀 가능 여부만, 언독 뒤 scan·AMCL·TF·Nav2) | 도킹 중에 전부 확인해서 호출이 항상 거절됨 | `a_course_node.py` `_check_ready(before_undock)`, `_tick` (3) |
| ④ | 언독 후 60 s 안에 준비 안 되거나 취소되면 그 자리 재도킹 | 실패로 그 자리 정지 → 사람이 도크에 올려야 함 | `course_engine.py` `redock` / `a_course_node.py` `_tick` (3), `_update_prepare` |
| ⑤ | Nav2 `autostart=false`, 언독 뒤 scan·TF가 오면 STARTUP | 도킹 중 map TF가 없어 Nav2가 `Aborting bringup` | `a_course_node.py` `_activate_nav2` / launch `_nav2_actions` |
| ⑥ | launch 노드 super client 끄기, `bond_timeout` 30 s (localization 4 s, Nav2 10 s였음) | 서버 경유 discovery가 느려 bringup 포기 | launch `super_client`, localization `SetParameter` / `config/nav2_real.yaml` |
| ⑦ | 키오스크 정지 후 등(웹캠)이 키오스크를 보게 spin, `kiosk.goal_pose.yaw` 반대로 | 키오스크를 보고 서서 뒤쪽 웹캠이 반대쪽을 봄. 고정 각도로 돌렸다가 반만 돎 | `course_engine.py` `FACE_KIOSK`, `_kiosk_turn` / `a_course_node.py` `_start_spin` / `a_course.yaml` kiosk |
| ⑧ | 해설 중 반원 밖 0.15 m·3 s까지 봐줌 | 호 바로 안쪽에 서서 AMCL이 한 번 튀면 0.7 s 만에 실패 | `course_engine.py` `_explain_ok`, `tick` (5), `explanation_result` |
| ⑨ | 도킹 접근 원에서 정면이 도크를 보게 spin 후 Dock | 도크를 안 보고 Dock → Create 3가 앞 벽에 부딪혀 실패 | `course_engine.py` `FACE_DOCK`, `_dock_turn` / `a_course.yaml` dock |
| ⑩ | 인식 노드가 웹캠 직접 열기, `camera_pub` 안 띄움 (`direct_camera`) | 영상이 토픽을 거치고, 같은 웹캠을 두 번 엶 | launch `_tracker` (인식 노드에 `video_source`를 넘기고 `phone_camera.launch.py` `publish_camera:=false`. person_tracker 코드는 안 고침) |
| ⑪ | 키오스크는 반원 진입 즉시 말고 `kiosk.goal_pose`(RViz로 찍은 자리)까지 가서 서고, 등이 키오스크 면을 똑바로 보게 회전 (`kiosk_stop_at_goal`, `kiosk_face_target: plane`) | 반원 가장자리에 서서 서는 자리·방향이 매번 달라, 관람 시작 때 그림 1 쪽으로 크게 휙 돎 | `course_engine.py` `tick` (1), `nav_result`, `_kiosk_turn` / `a_course.yaml` kiosk `goal_pose` (-1.032, 0.0668) → (-1.5188, 0.6984), 반원 반지름 0.6176 → 1.2 / 자리 찍기 `launch/zone_view.launch.py` |

그 밖에: 로봇 OAK-D 끔 (robot4는 웹캠 사용, 로봇 `standard.launch.py`에서 주석 처리 — 저장소 밖).

## 구역·목표 보기, 좌표 찍기 (로봇 없이)

```
ros2 launch arti_pc2_nav zone_view.launch.py
ros2 topic echo /robot4/clicked_point geometry_msgs/msg/PointStamped   # 다른 터미널
```

`a_course.yaml`의 반원(키오스크 파랑, 작품 초록)·출구(주황)와 `goal_pose`(흰 점 + 화살표 = 서는 자리·정면)를 지도 위에 보여 준다.
RViz "Publish Point"로 클릭하면 좌표가 나온다. `goal_pose`는 자기 반원 **안**이어야 한다 (밖이면 코스 노드가 설정을 못 읽음).
follow 팀 `arti_follow zone_view.launch.py`는 `arti_common/artworks.yaml`을 보여 주므로 robot4 값과 다르다.

## 실행

**로봇은 도크에 붙인 채로**, 터미널 하나에서 `source ~/arti/install/setup.bash` 후:

```
ros2 launch arti_pc2_nav a_course.launch.py enable_motion:=true
```

이것 하나로 localization → (5 s 뒤) Nav2 → 코스 노드·음성·인식·RViz 구역 마커가 다 뜬다.
- **Nav2는 띄우기만 하고(autostart=false) 언독 뒤에 켠다.** 도킹 중엔 LiDAR가 꺼져 AMCL이 map→odom TF를 안 내서,
  launch 때 켜면 global_costmap이 TF를 기다리다 `Aborting bringup`으로 멈춘다 (2026-11-20). 호출 → 언독 → initialpose →
  scan·TF가 들어오면 코스 노드가 `lifecycle_manager_navigation/manage_nodes` STARTUP을 요청한다 (로그 `Nav2 STARTUP 완료`).
  한 번 켜지면 다음 호출부터는 그대로 쓴다. 손으로 켜려면
  `ros2 service call /robot4/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"`.
- **localization**: follow 팀과 같은 `arti_bringup localization.launch.py`. AMCL 시작 위치 = `arti_common/config/init_pose.yaml`의
  `/robot4.undock` (언독 직후 위치, 2026-11-20 측정 (-0.12, 0.03, -179°)) → **RViz·2D Pose Estimate 불필요** (RViz는 보고 싶을 때만 따로).
  로그에 `시작 위치 = init_pose.yaml undock`이 나오면 된다. 지도 기본 = 워크스페이스 `maps/gallery.yaml` (`map:=`로 바꿈).
- **언독 직후 initialpose**: 도킹 중엔 LiDAR가 꺼져 AMCL이 위치를 못 잡으므로, 언독이 끝나면 코스 노드가 같은 `undock` 값을
  `initialpose`로 한 번 보낸다 (= 2D Pose Estimate를 코드가 대신, follow `visitor_follower` LOCALIZE와 같음). 도크를 옮기면 다시 잰다.
- **도킹 중에는 TurtleBot4 절전(power_saver)으로 LiDAR가 꺼져 있다.** 그래서 호출(prepare_tour) 때는 도크로 돌아올 수 있는지만 점검하고 언독한 뒤,
  scan·AMCL·TF·Nav2가 3 s 연속 준비될 때까지 최대 60 s 기다렸다 키오스크로 출발한다. 안 되면(또는 그 사이 취소) **그 자리에서 Dock 액션으로 다시 도킹**한다.
- localization·Nav2를 따로 띄웠다면 `nav:=false`.

Nav2는 이 패키지의 `config/nav2_real.yaml`을 쓴다. TurtleBot4 기본 `nav2.yaml`과 다른 값:
- 회전 최고 속도 1.0 → **0.5** rad/s: DWB `max_vel_theta`, spin `max_rotational_vel`, velocity_smoother 회전 상한 (변경 ⑮, 2026-11-21: 회전이 너무 빠름)
- footprint 팔각형 반지름 0.189 → **0.23**, `inflation_radius` 0.22 → **0.32** (변경 ⑬ 0.28/0.45 → ⑯ 0.23/0.32, 2026-11-21: 위에 올린 노트북(40×40 cm, 정가운데 전제)이 장애물에 부딪힘. 노트북 모서리는 약 5 cm 밖으로 나옴)
- `xy_goal_tolerance` 0.25 → **0.12** (작품 4 반원이 작아서(당시 r=0.43, ⑭에서 0.6) 0.25면 반원 밖에 정지)
- local·global costmap `transform_tolerance` 0.3 → **1.0** s (와이파이·서버 경유로 AMCL TF가 0.9 s 늦게 와서 Nav2가 출발하자마자 실패한 적 있음)
- `lifecycle_manager_navigation` `bond_timeout` 10 → **30** s (서버 경유 discovery가 느려 Nav2 bringup이 중간에 멈춘 적 있음)
- localization `lifecycle_manager_localization` `bond_timeout` 4 → **30** s: params 파일로 못 바꿔서(nav2_bringup이 안 넘김) `a_course.launch.py`가 `SetParameter`로 넣는다 (2026-11-20, 서버 경유 4 s 초과로 localization이 포기함)

연결 확인 (움직이지 않음):

```
ros2 action info /robot4/arti/prepare_tour
ros2 action info /robot4/explain_artwork
ros2 topic echo /arti/course_status
```

키오스크 없이 직접 호출 (**로봇이 움직인다**):

```
ros2 action send_goal /robot4/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}" --feedback
ros2 action send_goal /robot4/arti/start_tour arti_interfaces/action/StartTour "{artwork_ids: [1, 3, 4]}" --feedback
```

시험용 조합:

| 목적 | 인자 |
|---|---|
| 주행 잠금 (기본) — 카메라·인식·음성만 켜 보기 | `enable_motion:=false` |
| 인식 없이 주행만 | `tracker:=false skip_register:=true watch_visitor:=false` |
| 음성 없이 (설명은 2 s 모의) | `audio:=false simulation_mode:=true` |
| 재생 음량 | `player_cmd:="pw-play --volume 0.6"` |

## launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_namespace` | `robot4` | 모든 노드의 namespace |
| `config_file` | `share/arti_pc2_nav/config/a_course.yaml` | 좌표 설정 |
| `enable_motion` | `false` | true여야 요청 수락 (안전 잠금) |
| `simulation_mode` | `false` | true면 작품 설명을 2 s 모의, 배터리 시험값 허용 |
| `use_sim_time` | `false` | Gazebo면 true |
| `skip_undock`, `simulate_docking` | `false` | 시뮬레이션 전용 |
| `require_dock_status` | `true` | Dock 후 `dock_status.is_docked` 확인 |
| `simulation_battery_percent` | `-1.0` | simulation_mode 배터리 값 |
| `audio` | `true` | arti_audio `guide_audio.launch.py` include |
| `player_cmd` | `pw-play` | 재생 명령 |
| `tracker` | `true` | person_tracker 웹캠·인식 노드 |
| `skip_register` | `false` | 인식 없이 등록 완료 |
| `watch_visitor` | `true` | 관람 중 관람객 거리 감시 |
| `camera_yaw` | `180.0` | 웹캠 방향 [deg] → 카메라 TF(`yaw`)와 인식 노드(`cam_yaw`) 둘 다 |
| `tracker_node` | `gesture_test_node` | 인식 노드. `gesture_test_node` = 주먹→손바닥(또는 클릭)으로 대상 지정, `multi_tracker_node` = 클릭만 |

## a_course_node 파라미터 (주요)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `minimum_battery_percent` | 30.0 | 이 미만이면 출발 거부 [%] |
| `undock_timeout` / `dock_timeout` | 25.0 / 90.0 | [s] |
| `activate_nav2` / `nav2_lifecycle_manager` / `nav2_startup_retry` | true / `lifecycle_manager_navigation` / 10.0 | 언독 후 scan·TF가 들어오면 Nav2 STARTUP 요청, 실패하면 이 간격 뒤 RESET → STARTUP [s] |
| `dock_face` | true | 도킹 접근 원에서 멈춘 뒤 정면이 충전 스테이션(`dock.station`)을 보게 Nav2 spin → Dock. 도크를 안 보고 Dock하면 Create 3가 찾다가 앞 벽에 부딪혀 실패 (2026-11-20). 허용 각도·시간은 `face_tolerance`·`face_timeout` |
| `explain_zone_margin` / `explain_lost_grace` | 0.15 / 3.0 | 해설 중 반원 밖 0.15 m까지는 안으로 보고, 밖·위치 끊김이 3 s 넘게 계속될 때만 실패 (호 바로 안쪽에 멈춰 AMCL 흔들림으로 바로 실패하던 문제) [m, s] |
| `kiosk_face` / `face_tolerance` / `face_timeout` | true / 0.35 / 20.0 | 키오스크 반원에서 멈춘 뒤 등(웹캠이 뒤를 봄)이 키오스크(반원 중심)를 보게 Nav2 spin 제자리 회전 → 등록 시작. 차이가 0.35 rad 이하면 안 돎, 20 s 넘거나 실패하면 그대로 등록 [rad, s] |
| `kiosk_stop_at_goal` / `kiosk_face_target` | true / `plane` | [⑪] true: 키오스크 반원에 들어서도 멈추지 않고 `kiosk.goal_pose`까지 감 (반원 안에서 Nav2 ABORTED면 도착으로 봄). `plane`: 도착 뒤 정면 = 반원 방향(`zone.direction`) → 등(웹캠)이 키오스크 면을 똑바로 봄. 전 방식은 false / `center` |
| `kiosk_goal_tolerance` | 0.30 | [⑪] 키오스크 반원 안이고 `goal_pose`에서 이 거리 안이면 도착으로 멈춤. 찍은 자리가 벽 가까이라 Nav2가 끝까지 못 들어가는 경우 (2026-11-20: 0.22 m 앞에서 `Failed to make progress` 반복) [m] |
| `set_initial_pose` / `start_pose_file` / `start_point` | true / `''`(= arti_common `init_pose.yaml`) / `undock` | 언독 직후 initialpose로 보낼 위치 |
| `post_undock_sensor_timeout` / `post_undock_settle_time` | 60.0 / 3.0 | 언독 후 scan·AMCL·TF·Nav2 준비 대기 / 연속 유지 [s]. 넘으면 재도킹 |
| `visitor_topic` | `target_person/point_map` | person_tracker 출력 |
| `register_confirm_time` / `register_gap` | 1.0 / 0.5 | 끊김 없이 1 s 보이면 등록, 0.5 s 넘게 끊기면 다시 셈 [s] |
| `register_timeout` | 60.0 | 키오스크 도착 후 등록 제한 [s] |
| `ready_timeout` | 130.0 | 등록 후 start_tour 대기 [s] |
| `far_distance` / `resume_distance` | 2.5 / 1.8 | 멈춤 / 다시 출발 거리 [m] (2026-11-19 실주행: 1.5 / 0.95는 너무 자주 멈춤) |
| `distance_window` / `far_hold` / `near_hold` | 1.0 / 1.0 / 0.5 | 거리 중앙값 구간 / 멈추기 전 유지 / 출발 전 유지 [s] |
| `come_here_interval` | 8.0 | '따라오세요' 음성 최소 간격 [s] |
| `lost_timeout` / `visitor_give_up` | 3.0 / 10.0 | 안 보임 판정 / 포기 [s] |
| `*_guide` | `course_register`, `recognized_guide`=''(안내 없음), `register_failed`, `course_start`, `course_come_here` | guide_lines.yaml 키 |

## config/a_course.yaml

| 키 | 내용 |
|---|---|
| `kiosk.goal_pose`, `kiosk.zone` | 만남 장소. 카메라가 뒤쪽이므로 관람객이 서는 쪽으로 로봇이 **등을 대도록** yaw를 잡는다 |
| `artworks[].artwork_id` | 작품 번호 (int, PC3 DB 번호). YAML 순서 = 방문 순서 |
| `artworks[].goal_pose`, `zone` | Nav2 목표 / 도착 판정 반원. goal_pose는 자기 반원 안이어야 함 (아니면 로드 실패) |
| `exit.goal_pose` | 출구 Nav2 목표점 (공통 출구 직사각형 안이어야 함) |
| `exit.farewell` | 출구 안내 키 (기본 `tour_end`) |
| `exit.zone` | `arti_common` → 출구 도착 = arti_common `artworks.yaml` exit 직사각형 안 (없으면 목표점 0.30 m) |
| `dock.station`, `dock.approach_pose` | 충전 스테이션 / Nav2로 가는 정면 접근점 (마지막 정밀 접근은 Dock 액션) |

## 파일

| 파일 | 내용 |
|---|---|
| `arti_pc2_nav/course_engine.py` | 상태 기계 `TourMachine` (ROS 없음 — Event 목록만 돌려줌) |
| `arti_pc2_nav/a_course_node.py` | ROS 어댑터: 센서 → `tick()`, Event → Nav2·Dock·음성, 액션 서버 2개 |
| `arti_pc2_nav/guided_tour_plan.py` | YAML 파서, `Pose2D`·`Semicircle`·`CoursePlan` |
| `arti_pc2_nav/zone_geometry.py`, `zone_marker_node.py` | 반원 호 계산, RViz 구역 마커 |
| `launch/a_course.launch.py` | 위 노드 전부 + localization(arti_bringup, 시작 위치 = init_pose.yaml undock)·Nav2 (`nav:=true`) |
| `config/nav2_real.yaml` | robot4 Nav2 설정 (기본값 + xy_goal_tolerance 0.12, costmap transform_tolerance 1.0, bond_timeout 30) |
| `test/test_full_course.py` | 상태 기계 단위 테스트 |

액션 실행 함수(`_prepare_execute`, `_tour_execute`)는 별도 스레드에서 기다리며 feedback/result만 보내고,
판단과 상태 변경은 전부 10 Hz `_tick`(`_update_prepare`, `_update_tour`)이 한다 (스레드끼리 상태를 동시에 바꾸지 않게).

## 테스트

```
cd ~/arti/src/arti_pc2_nav && python3 -m pytest -q test/
```

## 아직 실물에서 확인 못 한 것 (2026-11-19)

- 뒤쪽 웹캠(`camera_yaw` 180)으로 관람객 map 좌표가 맞게 나오는지, `far_distance` 2.5 m가 적당한지
- 키오스크 `goal_pose` yaw (관람객 쪽으로 등)
