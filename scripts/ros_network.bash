# arti ROS 2 네트워크 설정 — .envrc가 env.bash 다음에 source 한다
# 기준: 노션 "Multi Robot Custom Discovery Setup" CASE 1-3
#   관제 PC(host)에 로봇 PC 2대(guest)가 붙는다. 로봇 PC는 각자 로봇 한 대를 맡는다.
#   관제 PC가 discovery server(ID 0)를 직접 띄우고, Guest PC는 그 Host를 0번 칸에 등록한다.
#   Host PC는 별도 터미널에서 scripts/run_discovery_server.sh 를 켜 두어야 한다.
#
# env.bash에서 받는 값
#   ROS_DOMAIN_ID           팀 공통 1
#   ARTI_PC_ROLE            host | guest | (비우면 PC 간 통신 없이 로봇만, CASE 1-1)
#   ARTI_HOST_PC_IP         guest일 때 Host PC의 IP (Host PC에서 `hostname -I`)
#   ARTI_DISCOVERY_SERVERS  "서버ID:로봇IP ..."  이 PC가 맡은 로봇만 적는다
#                           예) robot1 담당 "1:192.168.107.101"
#                           서버ID = 로봇의 discovery server ID (turtlebot4-setup → ROS Setup → Discovery Server)
#
# 여기서 만드는 값
#   ARTI_ROBOT_NS           이 PC가 맡은 로봇 네임스페이스 (/robot<서버ID>). launch들의 namespace 기본값

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Discovery Server는 Fast DDS 전용
export ROS_SUPER_CLIENT=True                 # PC에서 ros2 topic list 등으로 모든 토픽이 보이게

# ROS_DISCOVERY_SERVER 형식: 세미콜론으로 구분, "몇 번째 칸"이 서버 ID
#   Host  + robot1(ID 1) → "127.0.0.1:11811;IP1:11811;"
#   Guest + robot4(ID 4) → "HOST_IP:11811;;;;IP4:11811;"
_arti_ds=()

case "${ARTI_PC_ROLE:-}" in
  host)  _arti_ds[0]="127.0.0.1:11811" ;;
  guest)
    if [[ "${ARTI_HOST_PC_IP:-}" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
      _arti_ds[0]="$ARTI_HOST_PC_IP:11811"
    else
      echo "[arti] ARTI_PC_ROLE=guest 인데 ARTI_HOST_PC_IP가 비었거나 형식이 틀림: '${ARTI_HOST_PC_IP:-}'" >&2
    fi ;;
  "") ;;
  *) echo "[arti] ARTI_PC_ROLE 값 무시: '$ARTI_PC_ROLE' (host | guest | 빈 값)" >&2 ;;
esac

for _entry in ${ARTI_DISCOVERY_SERVERS:-}; do
  _id="${_entry%%:*}"; _ip="${_entry#*:}"
  if [[ "$_id" =~ ^[0-9]+$ && "$_ip" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
    if [[ "$_id" == 0 ]]; then
      echo "[arti] 서버ID 0은 Host PC 자리라 로봇에 쓸 수 없음: '$_entry'" >&2
      continue
    fi
    _arti_ds[$_id]="$_ip:11811"
    # 이 PC가 맡은 로봇 = 첫 번째 항목 (서버ID = 로봇 번호, 1 → /robot1). env.bash에서 직접 정하면 그 값 우선
    [ -z "${ARTI_ROBOT_NS:-}" ] && export ARTI_ROBOT_NS="/robot$_id"
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
