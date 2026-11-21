# course — robot4 코스 관람 (src/arti_pc2_nav/)

노드·인터페이스·파라미터·실행 방법은 [`src/arti_pc2_nav/README.md`](../../src/arti_pc2_nav/README.md)를 참고한다.

## 다른 팀과 주고받는 것

| 상대 | 이름 | 방식 | 방향 |
| --- | --- | --- | --- |
| 키오스크(PC3) | `/robot4/arti/prepare_tour` (`PrepareTour`) | Action | PC3 → PC2 |
| 키오스크(PC3) | `/robot4/arti/start_tour` (`StartTour`) | Action | PC3 → PC2 |
| 오디오(arti_audio) | `/robot4/explain_artwork` (`ExplainArtwork`, 작품 번호 int), `/robot4/guide_message` | Action, Topic | PC2 안 |
| 작품 설명 | `/arti/description` (`Description`, 작품 번호 int) | Service | PC2 → PC3 |
| 사람 인식(person_tracker) | `/robot4/target_person/point_map`, `/robot4/arti/tour_end` | Topic | PC2 안 |

## 파일

- [`arti_course_flow.drawio`](arti_course_flow.drawio)는 키오스크 초기 조회, 로봇 호출과 자동 시작, 코스 상태 전이와 노드 간 통신을 보여준다.
