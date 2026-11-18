#!/usr/bin/env bash
# source ~/f_project/setup_sim.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'source ~/f_project/setup_sim.sh 로 불러오세요.' >&2
  exit 1
fi
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/rokey/turtlebot4_ws/install/local_setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export GZ_PARTITION=arti_sim_rokey
export QT_QPA_PLATFORM=xcb
arti_sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_LOG_DIR="$arti_sim_root/simulation/log/ros"
mkdir -p "$ROS_LOG_DIR"
if [[ -f "$arti_sim_root/simulation/install/local_setup.bash" ]]; then
  source "$arti_sim_root/simulation/install/local_setup.bash"
fi
if [[ -f "$arti_sim_root/install/setup.bash" ]]; then
  source "$arti_sim_root/install/setup.bash"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$arti_sim_root/simulation/fastdds-local.xml"
