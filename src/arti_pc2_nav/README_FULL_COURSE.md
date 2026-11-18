# ArtI 전체 코스 Gazebo 검증 완료

검증 일자: 2026-11-18 (한국 시간)
요청 ID: 72eef901-c6f8-433a-b77f-41ab6fb3b2bb
최종 결과: COMPLETED, completed=["1","2","3","4"]

## 실제 확인한 순서

실제 Gazebo Undock 성공 → 키오스크 반원 진입·Nav2 취소·Odometry 정지 → 작품1→2→3→4 각각 반원 진입·목표 취소·정지·모의 해설 완료 → 최초 언도킹 후 저장한 접근 위치로 Nav2 복귀 → 정지 → 실제 Gazebo Dock 액션 성공 → dock_status.is_docked=true → COMPLETED.

Gazebo 로그에서 Undock Goal Succeeded와 Dock Servo Goal Succeeded를 확인했습니다. Dock 결과의 is_docked도 검사하며, 센서 확인 전에는 COMPLETED를 보내지 않습니다. 출구로 이동하지 않습니다.

이번 검증 중 그림4 접근에서 컨트롤러 설정을 수정하고 복구 승인 1회로 같은 목표를 재시도했습니다. 처음부터 무개입으로 완주한 시험이라고 주장하지 않습니다. 수정된 설정은 저장되어 다음 실행부터 적용됩니다. 전체 미션은 같은 request_id와 완료 목록을 유지한 채 복구 후 실제 도킹까지 완료했습니다.

## 수정 원인과 코드

- 반원 반지름의 80% 안쪽만 인정하던 추가 제한 제거: 원래 설정된 반원 전체와 방향을 검사합니다.
- Nav2 목표별 generation을 구분해 이전 목표의 늦은 결과를 다음 작품에 적용하지 않습니다.
- DWB의 회전 전환 허용오차와 최종 goal checker 허용오차를 모두 0.12m로 일치시켰습니다. 0.25m/0.12m 불일치가 근거리 접근 정체를 일으켰습니다.
- 좁은 실내 환경에 맞춰 progress_checker 이동 기준 0.05m, 허용 시간 30초를 사용합니다.
- simulation의 물리 간격 0.01초. 시뮬레이션 시계를 사용해 저속 실행 중의 시간초과 왜곡을 줄입니다.
- 전체 미션·정지·도킹 확인·반원 방향을 포함한 테스트 7개 통과.

구현: /home/rokey/f_project/src/arti_pc2_nav
주행 상태 머신: arti_pc2_nav/course_engine.py
ROS 어댑터: arti_pc2_nav/a_course_node.py
출구 표시: arti_pc2_nav/zone_geometry.py
좌표: config/a_course.yaml
시뮬레이션 Nav2: /home/rokey/f_project/simulation/nav2_full_sim.yaml

## 실행

현재 Gazebo와 RViz는 완료 상태로 열어 두었습니다. 중복으로 실행하지 마세요.

환경이 종료된 다음 새로 시작:
```bash
cd /home/rokey/f_project
./sim-start
```
다른 터미널에서 준비 완료 후 전체 코스 한 번 요청:
```bash
source /home/rokey/f_project/setup_sim.sh
ros2 topic pub --once /robot4/arti/tour/select std_msgs/msg/String '{data: A}'
```
상태 확인:
```bash
ros2 topic echo /arti/course_status
```

## 검증 범위와 실물 연결

객체인식은 실행하지 않으며 OBJECT DETECTION TEAM TODO 주석에 향후 연동 지점을 남겼습니다. 이번 해설은 2초의 명시적 모의 완료입니다. 실제 음성 액션과 PC3 예약 서비스는 구현/검증 범위에 포함하지 않습니다. 배터리 시험값 75%는 simulation_mode에서만 사용합니다. 도킹은 모의 도킹이 아닌 실제 Gazebo 액션·센서 결과입니다.

실물과 같은 namespace robot4와 navigate_to_pose, undock, dock, amcl_pose, odom, scan, battery_state, dock_status, guide_message를 사용합니다. /tf와 /tf_static은 /robot4/tf 및 /robot4/tf_static으로 연결합니다. PC2는 cmd_vel을 발행하지 않습니다. 실행 그래프는 ArtI_ROS_interfaces.txt에 있습니다.

실물 설정: use_sim_time=false, simulation_mode=false, skip_undock=false, simulate_docking=false, require_dock_status=true. 실제 ExplainArtwork 액션 서버와 실제 배터리 입력이 준비되지 않으면 시작을 거부합니다. 이 PC에 arti_interfaces/ExplainArtwork가 없어 실제 음성 액션은 검증하지 못했습니다. COMPLETED 보고를 받은 PC3가 예약 해제를 수행해야 합니다.

전체 코스 알고리즘과 Gazebo 물리 이동·복귀·도킹은 이번 실행으로 검증했습니다. 실물 로봇에는 실행하지 않았으며 실제 초기 위치·센서·충전기 및 음성/PC3 연동 검증은 별도입니다.

출구 반원은 지름 (0.167,1.950)~(0.147,2.280), 중심 (0.157,2.115), 반지름 약 0.1653m, 내부 -X 방향의 주황색 표시를 유지합니다.

## 증거

- ArtI_verified_mission.log: 한 request_id의 작품1~4 완료, 복귀, 도킹, COMPLETED
- ArtI_verified_COMPLETED_RViz.png 및 ArtI_verified_COMPLETED_Gazebo.png: 실제 완료 화면
- simulation/log/verified-run.log: Gazebo Undock/Dock 성공 원본 로그
