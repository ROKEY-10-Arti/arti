# 주행 띄우기 (localization → nav2 → rviz)

TurtleBot 4 기본 launch로 지도 위에서 주행을 띄우는 순서다. 터미널 3개를 열고 **각 터미널에서 먼저 `cd ~/project/arti`** 를 한다 (direnv가 환경을 켠다).
로봇은 언도크된 상태여야 한다.

지도는 `arti_common` 패키지(`src/common/arti_common/maps/`)의 `rokey_map.yaml`을 쓴다 (빌드 후 설치 경로).

## robot1

```bash
# 1. localization (AMCL + map_server) — "Creating bond timer..."가 보이면 다음 단계
ros2 launch turtlebot4_navigation localization.launch.py namespace:=$ARTI_ROBOT1_NS \
  map:=$(ros2 pkg prefix arti_common)/share/arti_common/maps/rokey_map.yaml

# 2. rviz — 상단 "2D Pose Estimate"로 로봇 초기 위치 지정
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=$ARTI_ROBOT1_NS

# 3. nav2 — 초기 위치를 지정한 뒤 실행
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=$ARTI_ROBOT1_NS
```

## robot4

위와 같고 `$ARTI_ROBOT1_NS`만 `$ARTI_ROBOT2_NS`(`/robot4`)로 바꾼다.

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=$ARTI_ROBOT2_NS \
  map:=$(ros2 pkg prefix arti_common)/share/arti_common/maps/rokey_map.yaml
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=$ARTI_ROBOT2_NS
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=$ARTI_ROBOT2_NS
```

## 두 로봇이 같은 지도를 쓴다는 것

```
rokey_map.yaml ──┬─> /robot1/map_server ─> /robot1/map ─> /robot1/amcl ─> /robot1/amcl_pose (map 좌표)
                 └─> /robot4/map_server ─> /robot4/map ─> /robot4/amcl ─> /robot4/amcl_pose (map 좌표)
```

- 로봇마다 localization을 따로 띄우고, **같은 지도 파일**을 각자 불러온다. map_server를 하나만 두는 방식이 아니다.
- 지도 파일이 같으므로 origin도 같다. 그래서 두 로봇의 `map` 좌표계가 일치한다.
  → 작품 위치, 만남 위치, 충전 위치, 좁은 통로 구간은 **좌표를 한 번만 정의하면 두 로봇이 같이 쓴다** (`arti_common/config/*.yaml` 예정).
- TF는 네임스페이스별로 분리된다(`/robot1/tf`, `/robot4/tf`). 그래서 rviz 하나에는 한 로봇만 보인다. 관제에서 두 로봇 위치를 함께 볼 때는 각 `amcl_pose`를 구독한다.
- 상대 로봇은 LiDAR에 잡히면 costmap에 장애물로만 나타난다. 좁은 통로 통행 순서는 `arti_fleet`이 두 `amcl_pose`를 보고 정한다.
- 지도를 새로 만들면 두 로봇 모두 새 지도로 다시 띄우고, config의 좌표도 새 지도 기준으로 다시 잡는다.

## 확인

```bash
ros2 topic echo /robot1/amcl_pose --once     # 위치 추정 값이 나오는지
ros2 topic echo /robot4/amcl_pose --once     # 두 로봇 좌표가 같은 지도 기준인지 (실제 위치와 비교)
ros2 action list | grep navigate_to_pose      # /robot1/navigate_to_pose 가 보이면 nav2 준비 완료
```

## 메모

- 원래 명령은 `~/rokey_ws/NAV_COMMANDS.txt` (지도 경로만 arti 패키지로 바꿈).
- `~/rokey_ws/maps/rokey_map`과 이 지도는 **origin이 다른 별개 지도**다. 섞어 쓰지 않는다.
- 나중에 `arti_bringup` launch 파일로 묶을 예정.
