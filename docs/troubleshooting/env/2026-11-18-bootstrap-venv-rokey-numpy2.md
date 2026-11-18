---
title: person_tracker 노드가 arti_venv 대신 rokey_venv의 numpy 2·opencv 5를 불러와 cv_bridge KeyError
date: 2026-11-18
author: jsy0605
status: 해결
packages: [person_tracker]
nodes: [gesture_test_node, multi_tracker_node, camera_pub]
robot: 해당없음
---

## 증상

`~/arti`(direnv로 arti_venv가 켜진 상태)에서 person_tracker 노드를 띄우면 numpy 2.5.4, cv2 5.0.0이 로드되고 cv_bridge가 실패한다.

```
  File "/opt/ros/jazzy/lib/python3.12/site-packages/cv_bridge/core.py", line 276, in cv2_to_imgmsg
    if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
KeyError: 16
```

## 재현 방법

```bash
cd ~/arti      # VIRTUAL_ENV = ~/arti/arti_venv (env.bash의 ARTI_VENV)
python3 -c "from person_tracker.env_bootstrap import bootstrap_venv; print(bootstrap_venv()); import numpy; print(numpy.__version__)"
# → /home/mu-01/venvs/rokey_venv/lib/python3.12/site-packages, 2.5.4
```

## 원인

`env_bootstrap.bootstrap_venv()`가 후보 venv(`$VIRTUAL_ENV`, `~/venvs/arti_venv`, `~/venvs/rokey_venv` ...)를 차례로 보며
**sys.path에 아직 없는** site-packages를 맨 앞에 넣는다.
arti_venv python으로 실행하면 arti_venv는 이미 sys.path에 있어서 건너뛰고, 다음 후보 rokey_venv(numpy 2.5.4, opencv-python 5.0)를 맨 앞에 넣었다.
ROS Jazzy cv_bridge는 시스템 numpy 1.26 기준이라 numpy 2와 섞이면 깨진다.

## 해결

`src/person_tracker/person_tracker/env_bootstrap.py`: 후보의 site-packages가 이미 sys.path에 있으면 그 venv를 쓰는 것으로 보고 멈춘다(다음 후보를 넣지 않음).

arti_venv 패키지는 `bash scripts/make_venv.sh`로 `requirements.txt` 고정 버전 설치
(numpy 1.26.4, opencv(-contrib)-python 4.11.0.86, mediapipe 1.2.3, ultralytics 8.4.208, torch 2.15.0, lap 0.5.12).

## 확인 방법

```bash
cd ~/arti
python3 -c "from person_tracker.env_bootstrap import bootstrap_venv; print(bootstrap_venv()); import numpy, cv2; print(numpy.__version__, cv2.__version__)"
# → /home/mu-01/arti/arti_venv/lib/python3.12/site-packages, 1.26.4 4.11.0
```
cv_bridge `cv2_to_imgmsg` 정상, `import person_tracker.gesture_test_node` 정상.

## 참고

- 노드는 꼭 `~/arti` 안에서 띄운다. 밖에서는 `.bashrc`의 rokey_venv가 `VIRTUAL_ENV`라서 같은 문제가 난다.
- 프롬프트 앞 `(rokey_venv)` 표시는 direnv가 바꾸지 않는다. 실제로 쓰는 python은 `which python3`로 확인.
