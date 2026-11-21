# ArtI 자유 관람(`arti_follow` · `person_tracker`) 빌드 및 단계별 실행·테스트 가이드

실로봇 전체 통합 시나리오부터 기능 조합 테스트, 단품 노드·가짜 입력 테스트, 코드 레벨 모듈 단위 검증까지 단계별로 실행하는 명령 모음입니다.

---

## 0. 빌드 및 테스트 환경 준비

### ① 패키지 빌드
```bash
cd ~/project/arti
source /opt/ros/jazzy/setup.bash

# 전체 빌드 + setup 반영 (~/.bashrc에 alias 등록 시)
pb
# (alias 풀어서 실행 시: colcon build --symlink-install && source install/setup.bash)

# 인식 + 주행 관련 패키지 3개만 빠르게 빌드
colcon build --symlink-install --packages-select arti_common arti_follow person_tracker
source install/setup.bash
```

### ② 네트워크 환경 분리 (실로봇 vs 1대 PC 오프라인 테스트)

* **실로봇 / 현장 통합 테스트**: 기본 셸 환경(`ROS_DOMAIN_ID=1`, `ROS_DISCOVERY_SERVER`)을 그대로 사용합니다. 토픽이 안 보이면 데몬을 재시작하세요.
  ```bash
  echo $ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER
  ros2 daemon stop; ros2 daemon start
  ros2 topic list | grep robot1
  ```

* **1대 PC 격리 테스트 (Bag 재생 / 단품·모듈 검증용)**: 현장 로봇과 토픽이 섞이지 않도록 **여는 모든 터미널마다** 먼저 실행합니다.
  ```bash
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
  export ROS_DOMAIN_ID=99
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ros2 daemon stop
  source ~/project/arti/install/setup.bash
  ```

---

## 1. 실제 시나리오 (Full Integration — 로봇 주행 + 인식 + 구역 + 음성 + 액션)

> **전제**: Robot1이 도크 위에 대기 중이고, `/robot1` 네트워크에 연결되어 있어야 합니다.

```bash
# [터미널 1] Localization (map -> odom) — "Creating bond timer..." 확인 후 다음 진행
ros2 launch arti_bringup localization.launch.py namespace:=/robot1

# [터미널 2] (선택) RViz 위치 확인 및 초기 위치 미세 보정
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot1

# [터미널 3] 자유 관람 전체 통합 실행 (인식 + 추종 + 작품 구역 + 음성)
ros2 launch arti_follow tour.launch.py namespace:=/robot1
```

* **자주 쓰는 통합 옵션** (`tour.launch.py`):
  * `audio:=false` — 음성 노드(`guide_audio_node`) 제외하고 구역 판정까지만 실행
  * `camera_pub:=true` — `gesture`·`front` 모드에서도 `<ns>/top_cam/image_raw/compressed` 토픽 외부 발행
  * `tracker:=front` — 제스처(`gesture`, 기본) 대신 정면 자동·클릭 등록(`front_tracker_node`) 사용
  * `diag:=true` — `visitor_follower` 토픽 Hz·지연·제어 주기 진단 로그 출력
  * `test:=true` — `BAG_TOPICS` + `/rosout`을 `~/arti_bags/follow/<날짜_시각>/`에 MCAP 녹화 (영상 토픽 제외)

* **키오스크 대신 터미널에서 직접 액션 시작/종료**:
  ```bash
  # 0) 도크 상태 확인
  ros2 topic echo /robot1/dock_status --once
  # 1) 호출·준비 (언독 → 초기 위치 → 키오스크 보기 → 관람객 등록)
  ros2 action send_goal --feedback /robot1/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}"
  # 2) 자유 관람 시작 (prepare_tour 성공 후 130초 이내 / 종료는 Ctrl+C 취소)
  ros2 action send_goal --feedback /robot1/arti/start_tour arti_interfaces/action/StartTour "{artwork_ids: []}"
  ```

---

## 2. 부분 시나리오 (기능 조합 테스트)

### ① 작품 구역·음성 제외하고 인식 + 추종만 테스트 (`follow_test.launch.py`)
```bash
# 기본: 제스처 등록(주먹 -> 손바닥 펴기) + 추종
ros2 launch arti_follow follow_test.launch.py namespace:=/robot1

# 정면 자동 등록 + 클릭 등록 (front_tracker_node)
ros2 launch arti_follow follow_test.launch.py namespace:=/robot1 tracker:=front

# 바퀴 구동 없이(cmd_vel 미발행) 인식·경로 계산·진단 로그만 확인
ros2 launch arti_follow follow_test.launch.py namespace:=/robot1 drive:=false diag:=true

# 액션 없이 켜자마자 prepare -> start 자동 진입 + 영상 토픽 송출
ros2 launch arti_follow follow_test.launch.py namespace:=/robot1 auto_start:=true camera_pub:=true
```

### ② 녹화된 Camera+LiDAR+TF Bag 기반 오프라인 리플레이 (격리 환경 `ROS_DOMAIN_ID=99`)
`~/bag/test_full`, `~/bag/zzz3`, `~/bag/robot1_full` 등 카메라 영상이 포함된 bag을 1배속으로 재생하며 인식·칼만/키네마틱 게이팅·주행 반응을 확인합니다.

```bash
# [터미널 1] 트래커 노드 (GUI 창에서 사람을 클릭해 타겟 등록)
ros2 run person_tracker gesture_test_node
# kinematic gating OFF 비교 시:
# ros2 run person_tracker gesture_test_node --ros-args -p use_kinematic_gating:=false
# 출력 토픽 발행 없이 화면/로그만 관찰 시:
# ros2 run person_tracker gesture_test_node --ros-args -p publish_target_topics:=false

# [터미널 2] 기존 녹화된 target_person/* 토픽 충돌 방지를 위해 제외하고 1배속 재생
ros2 bag play ~/bag/test_full -x '/.*target_person.*'

# [터미널 3] (선택) 출력 좌표 확인 또는 주행 노드 Dry-run 반응 확인
ros2 topic echo /robot1/target_person/point_base
# 또는:
ros2 run arti_follow visitor_follower --ros-args -r __ns:=/robot1 -r cmd_vel:=cmd_vel_dryrun -p diag:=true
```
> **참고 (`oakd` 전용 bag)**: `~/bag/rosbag2_2026_11_17-10_08_43`처럼 `top_cam`이 없는 bag을 테스트할 때는 `-p image_topic:=/robot1/oakd/rgb/image_raw/compressed`를 지정해 실행합니다.

---

## 3. 부분 테스트 (단품 모듈 · 가짜 입력 테스트)

### ① 주행 모듈 단독 (`visitor_follower` — 인식 모듈 없이 가짜 좌표 테스트)
> 전제: Localization(`map -> odom`) 실행 중 (`COMMANDS.txt` 기준)
```bash
# [터미널 1] 속도 제한 예시 (로봇 안 움직이게 하려면 -r cmd_vel:=cmd_vel_dryrun 추가)
ros2 run arti_follow visitor_follower --ros-args -r __ns:=/robot1 -p max_linear:=0.15 -p max_angular:=0.8

# [터미널 2] 가짜 관람객 map 좌표 발행 -> 로봇이 0.8 m 앞까지 이동해 멈추는지 확인
ros2 topic pub -r 5 /robot1/target_person/point_map geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 1.0, y: 0.0}}"

# [터미널 3] 상태 변화 모니터링
ros2 topic echo /robot1/follow/state
```

### ② 인식 모듈 단독 (`person_tracker`)
```bash
# A. 상단 캠 TF + 퍼블리셔 + 트래커 통합 실행 (로컬라이제이션 제외)
ros2 launch person_tracker bringup.launch.py namespace:=/robot1 use_localization:=false run_init_pose:=false

# B. 상단 캠 TF + 카메라 퍼블리셔만 실행
ros2 launch person_tracker phone_camera.launch.py namespace:=/robot1 video_source:=0

# C. 트래커 노드만 단독 실행 (로컬 웹캠 또는 지정 토픽 구독)
ros2 run person_tracker gesture_test_node --ros-args -r __ns:=/robot1 -p video_source:=0
ros2 run person_tracker front_tracker_node --ros-args -r __ns:=/robot1 -p image_topic:=/robot1/top_cam/image_raw/compressed
```

### ③ 작품 구역 · 설명 서비스 단독
```bash
# 작품/출구/키오스크 구역 RViz 확인 (로봇 없이 standalone 실행)
ros2 launch arti_follow zone_view.launch.py standalone:=true

# artwork_zone 단독 (follow/state 조건 해제)
ros2 run arti_follow artwork_zone --ros-args -r __ns:=/robot1 -p need_follow:=false
ros2 topic echo /robot1/artwork_zone/state

# PC3 작품 설명 서비스 호출 확인
ros2 service call /arti/description arti_interfaces/srv/Description "{artwork_id: 1}"
```

---

## 4. 모듈 단위 테스트 (코드 레벨 Smoke / Unit 검증)

`docs/common/setup.md` 기준 가상환경(`rokey_venv`)과 시스템 `colcon test`/`pytest` 충돌을 피하기 위해, 격리 환경(`ROS_DOMAIN_ID=98`)에서 `python3`로 직접 모듈 import 및 핵심 클래스 단위를 검증합니다.

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=98 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source ~/project/arti/install/setup.bash

# 1) 주행 모듈 (MotionMixin / TourMixin / VisitorFollower) import 및 문법 확인
python3 -c "import arti_follow.motion, arti_follow.start_tour, arti_follow.visitor_follower; print('FOLLOW_IMPORT_OK')"

# 2) 인식 모듈 (LidarPersonLocalizer / BaseTrackerNode / gesture_test_node / front_tracker_node) import 확인
python3 -c "import person_tracker.core.lidar_person_localizer, person_tracker.nodes.base_tracker_node, person_tracker.nodes.front_tracker_node, person_tracker.debug.gesture_test_node; print('PERCEPTION_IMPORT_OK')"
```
