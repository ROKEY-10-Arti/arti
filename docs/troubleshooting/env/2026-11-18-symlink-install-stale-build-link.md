---
title: pull 뒤 colcon build 실패 "can't copy ... doesn't exist" + send_goal "The passed action type is invalid"
date: 2026-11-18
author: jsy0605
status: 해결
packages: [arti_audio, arti_follow, arti_interfaces]
nodes: [visitor_follower]
robot: 해당없음
---

## 증상

main을 pull한 뒤(`StartTour` 액션 추가) 빌드하지 않은 채 관람 시작 액션을 보냈더니 타입을 못 찾았다.

```
$ ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{}" --feedback
The passed action type is invalid
```

빌드하니 이번엔 pull로 지워진 파일 때문에 실패했다.

```
--- stderr: arti_audio
error: can't copy '/home/mu-01/arti/build/arti_audio/config/guide_audio.yaml': doesn't exist or not a regular file
---
--- stderr: arti_follow
error: can't copy '/home/mu-01/arti/build/arti_follow/launch/tour_no_tracker.launch.py': doesn't exist or not a regular file
---
```

## 재현 방법

```bash
# 예전에 --symlink-install로 빌드해 둔 상태에서
git pull                                   # src에서 config/guide_audio.yaml, launch/tour_no_tracker.launch.py가 삭제됨
colcon build --symlink-install
```

## 원인

- `The passed action type is invalid`: 새 액션 `StartTour`가 아직 빌드되지 않았다(`install/arti_interfaces/share/arti_interfaces/action/`에 `ExplainArtwork`만 있었음).
  또 저장소 밖(`~`, rokey_venv)에서 실행해 arti `install/setup.bash`가 source되지 않은 상태였다.
- `can't copy ... doesn't exist`: `--symlink-install`은 `build/<패키지>/`에 소스 파일로의 심볼릭 링크를 만든다.
  pull로 소스 파일이 지워져도 build 폴더의 링크는 남아서(깨진 링크) setuptools가 복사하다 실패한다.

## 해결

해당 패키지의 build·install 결과물만 지우고 다시 빌드 (소스는 그대로).

```bash
cd ~/arti
rm -rf build/arti_audio install/arti_audio build/arti_follow install/arti_follow
colcon build --symlink-install
```

그 뒤 `cd ~/arti`로 다시 들어가(direnv가 install/setup.bash 다시 로드) 액션을 보낸다.
빌드 전에 띄워 둔 visitor_follower는 액션 서버가 없는 예전 코드라 다시 띄운다.

## 확인 방법

```bash
ros2 interface show arti_interfaces/action/StartTour    # goal/result(reason)/feedback(state)가 나옴
```
`Summary: 6 packages finished`

## 참고

- 파일이 지워지거나 이름이 바뀐 pull 뒤에는 이 에러가 다시 날 수 있다. 에러에 나온 패키지만 `build/`·`install/`에서 지우면 된다.
