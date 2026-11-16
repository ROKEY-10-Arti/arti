---
title: colcon test가 launch_testing 플러그인 에러로 실패 (venv의 pytest 9)
date: 2026-11-16
author: choongman.choi
status: 해결
packages: [arti_bringup, arti_navigation]
nodes: []
robot: 해당없음
---

## 증상

`colcon test`를 실행하면 테스트가 하나도 돌지 않고 error로 끝난다.

```
Error while loading plugin "pytest11".
pluggy._manager.PluginValidationError: Plugin 'launch_testing' for hook 'pytest_pycollect_makemodule'
```

## 재현 방법

```bash
colcon test --packages-select arti_navigation --event-handlers console_direct+
colcon test-result --all     # 2 tests, 2 errors
```

## 원인

- `.bashrc`가 `rokey_venv`를 켜므로 `python3`/`pytest`가 venv 것(pytest 9.2.0)으로 잡힌다.
- ROS Jazzy의 `launch_testing` pytest 플러그인은 시스템 pytest(7.4.4) 기준이라 pytest 9와 맞지 않는다.

## 해결

arti 전용 venv(`~/venvs/arti_venv`)를 만들었다 (`scripts/make_venv.sh`).

- `--system-site-packages`로 만들고 pytest·colcon을 pip로 설치하지 않는다 → 시스템 pytest 7.4.4가 쓰인다.
- venv 안의 `colcon`은 apt colcon을 venv python으로 실행하는 래퍼다.
- `.envrc`가 이 venv를 `rokey_venv`보다 앞에 둔다.

결과: `colcon test-result --all` → 10 tests, 0 errors, 0 failures.

## 확인 방법

```bash
python3 -m pytest --version     # 어느 pytest가 잡히는지
colcon test-result --all        # errors 0
```
