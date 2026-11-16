# 트러블슈팅 기록

에러를 만나고 해결했으면 (또는 해결하지 못했어도) 여기에 남긴다.
같은 에러를 다른 팀원이 다시 겪지 않게 하는 것이 목적이다.

## 폴더 구조

문제가 **어디서 났는지** 기준으로 폴더를 나눈다. 폴더는 처음 기록할 때 만든다.

| 폴더 | 범위 |
|---|---|
| `env/` | 개발 PC 환경: 빌드(colcon), source, venv, 의존성, `ROS_DOMAIN_ID`·네트워크·DDS 통신 |
| `robot/` | TurtleBot 4 본체: 배터리, 도킹, Create3, 센서 하드웨어, 로봇 쪽 설정 |
| `common/` | `src/common/` 패키지: interfaces, bringup(localization·nav2), 지도·공용 좌표 |
| `follow/` | `src/arti_follow/` 사용자 추종 |

`src/`의 역할 폴더와 이름을 맞춘다. 새 역할 폴더가 생기면 여기에도 같은 이름으로 만든다.

여러 노드·패키지에 걸친 문제는 **원인이 있던 쪽** 폴더에 둔다.
원인을 모르면 증상이 보인 쪽에 두고, 원인을 찾으면 파일을 옮긴다.

## 기록 방법

1. [`TEMPLATE.md`](TEMPLATE.md)를 복사한다.
   ```bash
   cp docs/troubleshooting/TEMPLATE.md docs/troubleshooting/<폴더>/YYYY-MM-DD-짧은-설명.md
   ```
2. 파일 이름은 `날짜-영문-소문자-하이픈` 형식으로 쓴다. 예: `2026-11-16-domain-id-mismatch.md`
3. 에러 메시지는 **복사해서 그대로** 붙인다. 검색으로 다시 찾을 수 있어야 한다.
4. 아래 목록에 한 줄을 추가한다.
5. 기능 작업과 같은 PR에 넣어도 되고, 기록만 따로 PR로 올려도 된다.

## 찾는 법

```bash
grep -ri "에러 메시지 일부" docs/troubleshooting/
grep -rl "status: 미해결" docs/troubleshooting/     # 아직 못 푼 문제
grep -rl "nodes:.*follow_node" docs/troubleshooting/ # 특정 노드 관련 기록
```

## 목록

| 날짜 | 폴더 | 제목 | 상태 | 작성 |
|---|---|---|---|---|
| 2026-11-16 | env | [env.bash의 ROS_DOMAIN_ID가 로봇 설정을 덮어씀](env/2026-11-16-domain-id-mismatch.md) | 해결 | choongman.choi |
| 2026-11-16 | env | [colcon test가 launch_testing 플러그인 에러로 실패](env/2026-11-16-colcon-test-pytest-venv.md) | 해결 | choongman.choi |
| 2026-11-16 | follow | [visitor_follower 주행 테스트 중 로봇이 앞 벽 쪽으로 감](follow/2026-11-16-visitor-follower-wall-approach.md) | 미해결 | jsy0605 |
| 2026-11-16 | follow | [rqt_image_view에 visitor_follower 디버그 화면이 안 나옴](follow/2026-11-16-debug-image-not-shown.md) | 해결 | jsy0605 |
| 2026-11-16 | follow | [실제 인식 좌표로 추종하면 흔들리고 섰다 갔다 하며, 놓친 뒤 회전 탐색을 안 함](follow/2026-11-16-follow-shaky-no-search.md) | 미해결 | jsy0605 |
