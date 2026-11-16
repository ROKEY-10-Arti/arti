#!/usr/bin/env bash
# Host PC 전용: discovery server(ID 0)를 띄운다 (노션 CASE 1-3, 3단계)
# Guest PC와 통신하는 동안 이 터미널을 계속 켜 둔다. 끄면 PC 간 토픽이 끊긴다.
#   bash scripts/run_discovery_server.sh
set -e
unset ROS_DISCOVERY_SERVER   # 서버 자신은 클라이언트 설정이 없어야 한다
exec fastdds discovery --server-id 0
