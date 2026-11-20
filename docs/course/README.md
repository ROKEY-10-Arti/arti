# course — robot4 코스 관람 (src/arti_pc2_nav/)

노드·인터페이스·파라미터·실행 방법은 패키지 README가 원본이다: [`src/arti_pc2_nav/README.md`](../../src/arti_pc2_nav/README.md).

## 다른 팀과 주고받는 것

| 상대 | 이름 | 방식 | 방향 |
| --- | --- | --- | --- |
| 키오스크(PC3) | `/robot4/arti/prepare_tour` (`PrepareTour`) | Action | PC3 → PC2 |
| 키오스크(PC3) | `/robot4/arti/start_tour` (`StartTour`) | Action | PC3 → PC2 |
| 오디오(arti_audio) | `/robot4/explain_artwork` (`ExplainArtwork`, 작품 번호 int), `/robot4/guide_message` | Action, Topic | PC2 안 |
| 사람 인식(person_tracker) | `/robot4/target_person/point_map`, `/robot4/arti/tour_end` | Topic | PC2 안 |

## 파일

- `arti_course_flow.drawio` — 코스 주행 흐름 그림 (draw.io). **예전 구조 기준**(hardcoded_tts_node, 'A' 요청)이라 현재 코드와 다르다. 고치기 전까지는 패키지 README를 본다.
