# 지도

| 파일 | 내용 |
|---|---|
| `rokey_map.pgm` / `rokey_map.yaml` | 실습장 SLAM 지도 (2026-11-16 노션에서 가져옴). 72 x 126 px, 해상도 0.05 m → 약 3.6 m x 6.3 m |

- `.yaml`의 `image:`는 상대 경로다. 두 파일은 항상 같은 폴더에 둔다.
- 지도를 새로 만들면 파일명에 날짜나 장소를 붙여 추가하고, 이 표에 한 줄 적는다.
- 설치 후 경로: `$(ros2 pkg prefix arti_navigation)/share/arti_navigation/maps/rokey_map.yaml`
