"""카메라 + 추종만 띄우고 인식 노드는 다른 곳에서: follow_test.launch.py를 tracker:=camera로 띄운다.

    ros2 launch arti_follow cam_test.launch.py
    = ros2 launch arti_follow follow_test.launch.py tracker:=camera

뜨는 것 (모두 <ns> 아래)
    phone_camera.launch.py   camera_pub(웹캠 → top_cam/image_raw/compressed), top_cam_tf
    visitor_follower         target_person/point_map을 받아 추종
    ros2 bag (test:=true)    follow_test.launch.py의 BAG_TOPICS (카메라 영상 제외)
안 뜨는 것: 인식 노드 → 다른 터미널·다른 PC에서 같은 네임스페이스로 띄운다 (둘 중 하나만)
    ros2 run person_tracker gesture_test_node --ros-args -r __ns:=/robot1     # 제스처(주먹 → 손바닥)로 등록
    ros2 run person_tracker multi_tracker_node --ros-args -r __ns:=/robot1    # 바로 앞 사람 자동 등록·클릭 등록
  인식 노드는 <ns>/top_cam/image_raw/compressed, <ns>/scan, <ns>/tf를 받고 <ns>/target_person/point_map을 낸다.
  다른 PC에서 띄울 때는 같은 ROS_DOMAIN_ID·Discovery Server에 붙어 있어야 한다.

visitor_follower는 관람객을 기다릴 때 target_person/register_front(바로 앞 등록 요청)를 보낸다.
multi_tracker_node만 받고, gesture_test_node는 무시한다 (제스처로 등록).

옵션은 follow_test.launch.py와 같다: drive:=false, test:=false, auto_start:=true, bag_dir:=, namespace:=
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    follow_test = os.path.join(get_package_share_directory('arti_follow'), 'launch', 'follow_test.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(               # tracker만 고정, 나머지 인자는 명령줄 값이 그대로 전달됨
            PythonLaunchDescriptionSource(follow_test),
            launch_arguments={'tracker': 'camera'}.items()),
    ])
