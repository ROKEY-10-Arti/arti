# 사용자 추종 (follow)

- 로봇: **`/robot1`** (`env.bash`의 `ARTI_ROBOT1_NS`). 코스주행은 `/robot4`
- 패키지: `src/follow/arti_follow` (인터페이스는 `src/common/arti_interfaces`)
- 일정 (노션 간트): 11/16 ~ 11/18 코딩 → 11/19 ~ 11/20 테스트
- **작업 목록: [tasks.md](tasks.md)** (노션 T01~T46)
- 관련 요구사항: [common/requirements.md](../common/requirements.md)
- 원본 플로우차트: 노션 `tracking_pc_ver3.drawio`

## 범위

관람객을 추적 대상으로 등록하고, 지도 위 위치를 계속 추정해서 약 0.8 m 거리를 유지하며 따라간다.
작품 반원 구역에서 5초 멈추면 해설하고, 대상을 놓치면 분실 처리, 배터리 부족·종료 구역이면 도킹한다.
상태는 중앙 서버가 아니라 **로봇 노드가 직접** 관리한다. 웹(Flask)은 다른 팀이고 ROS 2로 데이터만 주고받는다.

## 하드웨어 (SRD 기준)

- 상판 노트북 웹캠(top_cam, Victus 15: 1280×720, 화각 88°): 사람 인식·재식별
  - 이유: OAK-D는 낮아서 가까운 사람의 하반신만 보인다
- OAK-D Pro Depth + LiDAR: 거리 측정
- 웹캠 박스 → Depth 매칭은 캘리브레이션(호모그래피 + 외부 파라미터)으로 한다 → T05
- 음성 출력: **에어팟** (블루투스 이어폰). 작품 설명·안내 음성을 `guide_audio_node`가 재생 → T35. 연결이 끊기면 `/arti/event`로 알림

## 노드 구성

| 노드 | 하는 일 | 입력 | 출력 | 작업 |
|---|---|---|---|---|
| `laptop_cam_node` | 노트북 웹캠 → 압축 이미지 | 웹캠 | 압축 이미지 (640×480 JPEG) | T10 |
| `visitor_perception` | 사람 추적·재식별, 등록, 손바닥 제스처 | 압축 이미지 | `VisitorDetection`, 디버그 이미지, palm / `RegisterVisitor` 액션 서버 | T12, T13, T14 |
| `visitor_localizer` | 웹캠 박스 + Depth → map 좌표, 칼만 필터 | `VisitorDetection`, Depth, camera_info, `/scan`, TF | `visitor/pose`, 마커 | T15, T16, T17 |
| `amr_controller` | 명령 실행 (UNDOCK·FACE·FOLLOW·GOTO·SEARCH·STOP·DOCK), 안전 워치독 | `AmrCommand`, `visitor/pose`, 범퍼·낭떠러지 | `AmrStatus`, Nav2 목표, `cmd_vel` | T20–T27 |
| 로봇 상태 노드 (이름 미정) | 상태 전환·판정 (등록 → 추적 → 설명 → 분실 → 종료) | `visitor/pose`(+속도), `AmrStatus`, `battery_state`, `artworks.yaml` | `AmrCommand`, `ArtiEvent`, `explain_artwork` 호출, `guide_message` / `StartSession` 서버 | T30–T34 |
| `guide_audio_node` | 작품 설명·안내 음성 재생 | `guide_message` | `ExplainArtwork` 액션 서버 | T35 |

## 인터페이스 (arti_interfaces에 추가할 것)

| 이름 | 종류 | 내용 | 작업 |
|---|---|---|---|
| `AmrCommand` / `AmrStatus` | msg | 로봇 명령(이동·정지·도킹 등) / 실행 결과·도착 여부·에러 코드 | T20 |
| `StartSession` | srv | 웹 버튼 → 로봇이 관람 세션 시작, 결과 응답 | T30 |
| `ArtiEvent` | msg | 등록 성공·분실·배터리 부족·관람 종료(사유) 등 상태 변화 | T30 |
| `RegisterVisitor` | action | 관람객 등록 (진행률·취소·타임아웃) | T13 |
| `VisitorDetection` | msg | 찾음 여부·ID·bbox | T12 |
| `ExplainArtwork` | action | 작품 설명 재생 (진행률·취소·완료) | T35 |

## 다른 팀과 주고받는 것

| 이름 | 종류 | 방향 | 상대 | 내용 |
|---|---|---|---|---|
| `/arti/start_session` | srv (`StartSession`) | 받음 | 웹(Flask) | 관람 등록 요청 |
| `/arti/event` | topic (`ArtiEvent`) | 보냄 | 웹(Flask) | 등록 성공·분실·배터리 부족·관람 종료 |
| 로봇 상태 (이름 미정) | topic | 보냄 | 웹(Flask) | 배터리·현재 상태 주기 발행 |
| 상대 로봇 위치 (미정) | topic | 받음 | 코스주행 | 경로 겹침 대응 (T28) |

## 로봇 내부 토픽·액션 (네임스페이스 아래)

노션 작업 목록의 `/robot2/...`는 모두 `/robot1/...`로 읽는다. 코드에서는 상대 이름(`visitor/pose`)으로 쓰고 launch에서 네임스페이스를 붙인다.

`visitor/pose`, `goal_update`, `explain_artwork`, `guide_message`, `spin`, `navigate_to_pose`

## 설정 파일 (공용 좌표는 `arti_common/config/`)

| 파일 | 내용 | 작업 |
|---|---|---|
| `artworks.yaml` | 작품별 좌표·정면 방향·반원 반경·음성 파일, 종료 구역 | T04 |
| `init_pose.yaml` | 로봇별 초기 위치·도킹 앞 좌표 | T06 |
| `calib.yaml`, top_cam camera_info | 웹캠↔Depth 호모그래피, 웹캠 내부 파라미터 | T05 |

## 파라미터 (기본값은 노션 작업 목록 기준)

| 이름 | 기본값 | 설명 |
|---|---|---|
| 추종 거리 | 0.8 m | 더 가까우면 정지 |
| 목표 갱신 거리 / 주기 | 0.3 m / 초당 2회 이하 | 로봇 떨림 방지 |
| 최대 속도 | 0.3 m/s | 예시 값 |
| 정지 판정 | 5 s, 반경 0.6 m | `is_dwelling` |
| 설명 중 이탈 | 1 m | 넘으면 설명 취소 |
| 인식 끊김 → 분실 | 1 s | 이하면 마지막 목표 유지 |
| 유효 거리 | 0.3–4 m | Depth 값 필터 |
| 등록 | 30 프레임, 10 s 타임아웃, 3회 재시도 | T13, T31 |
| 분실 처리 | 탐색 10 s, 대기 10 s, 재등록 20 s | T33 |
| 배터리 종료 기준 | 20% | 예시 값 |
| FACE 허용 오차 | ±5° | T23 |

## ⚠️ 확인 필요

- **탑CAM:** 11/15 메모는 "휴대폰", SRD·작업 목록은 "노트북 웹캠" → 작업 목록(노트북) 기준으로 정리함.
- **T18 bag 녹화:** 요구사항상 관람객 영상은 저장하지 않는다 → 테스트용 녹화 범위를 팀에서 합의.

## 참고

- 기존 실습 코드: `~/rokey_ws/src/yolo_follow` (YOLO → Depth → TF(map) → Nav2 목표), `~/rokey_ws/src/day3/depth_to_3d*`
- 트러블슈팅: [../troubleshooting/](../troubleshooting/README.md) 의 `follow/` 폴더
