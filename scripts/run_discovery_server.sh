#!/usr/bin/env bash
# Host PC 전용: discovery server(ID 0)를 띄운다 (노션 CASE 1-3, 3단계)
# Guest PC와 통신하는 동안 이 터미널을 계속 켜 둔다. 끄면 PC 간 토픽이 끊긴다.
#   bash scripts/run_discovery_server.sh
set -e
cd "$(dirname "$0")/.."
[ -f env.bash ] && source env.bash

# Guest PC에서 서버를 띄우면 아무도 쓰지 않는 서버가 하나 더 생겨 헷갈리기만 한다 → 막는다
if [ "${ARTI_PC_ROLE:-}" != host ]; then
  echo "[arti] 이 PC는 ARTI_PC_ROLE='${ARTI_PC_ROLE:-}' 이라 discovery server를 띄우지 않는다." >&2
  echo "[arti] 서버 ID 0은 Host PC(${ARTI_HOST_PC_IP:-env.bash의 ARTI_HOST_PC_IP})에서만 띄운다." >&2
  exit 1
fi

unset ROS_DISCOVERY_SERVER   # 서버 자신은 클라이언트 설정이 없어야 한다
exec fastdds discovery --server-id 0
