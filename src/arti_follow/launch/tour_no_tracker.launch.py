"""관람 안내 (tour.launch.py)에서 인식 노드(multi_tracker_node)만 뺀 것.

    ros2 launch arti_follow tour_no_tracker.launch.py

뜨는 것: 카메라(camera_pub, top_cam_tf) + visitor_follower + artwork_zone + guide_audio_node + 로그·bag
안 뜨는 것: multi_tracker_node → target_person/point_map은 다른 곳(다른 PC의 인식 노드, rosbag 재생,
            ros2 topic pub 테스트)에서 와야 로봇이 움직인다

옵션은 tour.launch.py와 같다 (drive:=false, test:=false, bag:=false, audio:=false, video_source:=, player_cmd:= ...).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# tour.launch.py로 그대로 넘기는 옵션 (tracker_node는 false로 고정)
PASS = {
    'namespace': EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
    'drive': 'true', 'test': 'true', 'bag': 'true', 'video_source': '0',
    'log_dir': '~/arti_logs/follow', 'audio': 'true', 'player_cmd': 'pw-play', 'zones_file': '',
}


def generate_launch_description():
    tour = os.path.join(get_package_share_directory('arti_follow'), 'launch', 'tour.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in PASS.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tour),
            launch_arguments=[(k, LaunchConfiguration(k)) for k in PASS] + [('tracker_node', 'false')])])
