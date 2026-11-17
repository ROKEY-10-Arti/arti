# 2026-11-17 20:18 visitor_follower 추종 주행 (robot1)

- 실행: `ros2 launch arti_follow follow_test.launch.py` (drive=True, tracker=true, 웹캠 0)
- 길이 134 s, 메시지 15,054개
- 카메라 영상·depth 없음 (개인정보 요구사항). 좌표·거리·명령만
- 원본(123 MB)에서 `/robot1/follow/debug`(RViz 마커, 지금은 삭제된 기능)를 빼고 MCAP zstd로 압축

정지 이유 (follow_timing.csv, 0.1 s 주기 수): follow 525, hold 529, follow+blocked 112,
search_spin 58, search_move 16, idle 82, no_pose 35, scan_timeout 17

```bash
ros2 bag info bags/follow/20261117_201800
ros2 bag play bags/follow/20261117_201800
```
