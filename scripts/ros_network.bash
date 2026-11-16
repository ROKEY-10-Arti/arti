# arti ROS 2 네트워크 설정 — .envrc가 env.bash 다음에 source 한다
# 원본: /etc/turtlebot4_discovery/setup.bash (turtlebot4-setup이 만든 PC 설정, robot1만 있음)
#
# env.bash에서 받는 값
#   ROS_DOMAIN_ID           팀 공통 1
#   ARTI_DISCOVERY_SERVERS  "서버ID:로봇IP ..."  예) "1:192.168.107.101 4:192.168.107.104"
#                           서버ID = 로봇의 discovery server ID (turtlebot4-setup → ROS Setup → Discovery Server)

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Discovery Server는 Fast DDS 전용
export ROS_SUPER_CLIENT=True                 # PC에서 ros2 topic list 등으로 모든 토픽이 보이게

# ROS_DISCOVERY_SERVER 형식: 세미콜론으로 구분, "몇 번째 칸"이 서버 ID
#   ID 1, 4 → ";IP1:11811;;;IP4:11811"
_arti_ds=()
for _entry in ${ARTI_DISCOVERY_SERVERS:-}; do
  _id="${_entry%%:*}"; _ip="${_entry#*:}"
  if [[ "$_id" =~ ^[0-9]+$ && "$_ip" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
    _arti_ds[$_id]="$_ip:11811"
  else
    echo "[arti] ARTI_DISCOVERY_SERVERS 항목 무시: '$_entry' (형식: 서버ID:IP)" >&2
  fi
done
if [ ${#_arti_ds[@]} -gt 0 ]; then
  _max=0; for _i in "${!_arti_ds[@]}"; do (( _i > _max )) && _max=$_i; done
  _s=""; for (( _i=0; _i<=_max; _i++ )); do _s+="${_arti_ds[$_i]:-};"; done
  export ROS_DISCOVERY_SERVER="$_s"   # 끝의 ; 는 turtlebot4-setup 형식과 같게 둔다
fi
unset _arti_ds _entry _id _ip _max _i _s
