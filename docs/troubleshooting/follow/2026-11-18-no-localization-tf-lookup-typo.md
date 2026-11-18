---
title: start_tour 액션을 보내면 undock 뒤 10초 만에 no_localization으로 끝남 (TF 조회 코드 오타)
date: 2026-11-18
author: 이정욱
status: 확인 중
packages: [arti_follow]
nodes: [visitor_follower]
robot: robot1
---

## 증상

localization·nav2를 띄운 상태에서 `tour.launch.py`를 켜고 `start_tour` 액션을 보냈다.
undock까지는 되는데, 초기 위치를 보낸 지 10초 뒤 관람이 `no_localization`으로 끝난다. 세 번 해도 같았다.

```
[robot1.visitor_follower]: undock 끝
[robot1.visitor_follower]: 상태 UNDOCK → LOCALIZE
[robot1.visitor_follower]: 초기 위치 보냄 (map -0.50, 0.70, 180°)
[robot1.visitor_follower]: 관람 끝 (no_localization)
[robot1.visitor_follower]: 관람 결과: success=False, message=no_localization, visited_ids=[]
```

## 재현 방법

```bash
# 터미널 1
ros2 launch arti_follow tour.launch.py
# 터미널 2
ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}" --feedback
```

## 원인

localization 문제가 아니었다.

- AMCL 로그: `initialPoseReceived` → `Setting pose (-0.500 0.700 3.142)`. 초기 위치는 받았다.
  (`Failed to transform initial pose in time` 경고도 같이 나오지만, 바로 뒤에 위치를 설정하므로 원인이 아니다.)
- 같은 실행의 bag(`~/arti_bags/follow/20261118_201112`): `/robot1/tf`에 `map → odom`이 613개, LOCALIZE 동안에도 계속 들어왔다.

그런데 노드는 `map → base_link`를 못 찾았다(`no_localization`은 10초 동안 위치를 하나도 못 얻었을 때만 나옴).
커밋하지 않은 수정에 한글 자모가 섞여 들어가 있었다 (`visitor_follower.py` `lookup()`):

```python
return tr.translation.x, tr.translation.y, yaw_of(tr.rotatㅈion)   # 'ㅈ'
```

`tr.rotatㅈion`은 없는 속성이라 `AttributeError`가 나는데, 바로 위 `try ... except Exception: pass`가 삼켜서
**에러 없이 `lookup()`이 항상 None**을 돌려줬다. 그래서 LOCALIZE에서 위치를 못 잡고 끝났다.
(LOCALIZE를 넘겼더라도 FOLLOW 등 TF를 쓰는 모든 곳에서 로봇이 서 있었을 것이다.)

## 해결

`src/arti_follow/arti_follow/visitor_follower.py:441` `tr.rotatㅈion` → `tr.rotation`.
symlink 설치(`build/arti_follow/arti_follow` → `src`)라서 다시 빌드하지 않고 launch만 다시 켜면 된다.

## 확인 방법

- `python3 -m py_compile`로 문법 확인. 코드 줄(주석 제외)에 섞인 한글 자모가 이것 하나뿐인지 grep으로 확인:
  `grep -nP "^[^#]*[A-Za-z_][\x{3131}-\x{318E}]" src/arti_follow/arti_follow/*.py`
- 로봇에서 다시 액션을 보내 `상태 LOCALIZE → START_FACE` 로그가 나오는지 확인 (아직 안 함).

## 참고

- `except Exception: pass`처럼 에러를 다 삼키는 곳은 오타도 조용히 숨긴다. 이런 증상이면 노드 코드를 먼저 의심.
- 관련 기록: [사람 좌표는 오는데 로봇이 안 움직임 (localization 꺼져서 map→odom 없음)](2026-11-16-no-map-odom-follower-idle.md)
