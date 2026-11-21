import glob
import os
import warnings
from setuptools import find_packages, setup

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    import setuptools.command.easy_install as easy_install
    # colcon build --symlink-install (setup.py develop) 시에도 가상환경 파이썬을 사용하도록 고정
    easy_install.ScriptWriter.get_header = classmethod(
        lambda cls, script_text="", executable=None: "#!/usr/bin/env python3\n"
    )

package_name = 'person_tracker'
model_files = [
    f for f in (glob.glob('models/*.pt') + glob.glob('models/*.task'))
    if os.path.isfile(f)
]
launch_files = [f for f in glob.glob('launch/*.launch.py') if os.path.isfile(f)]
config_files = [f for f in glob.glob('config/*.yaml') if os.path.isfile(f)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'requirements.txt', 'setup_and_build.sh', 'README.md']),
]
if model_files:
    data_files.append(('share/' + package_name + '/models', model_files))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsb',
    maintainer_email='user@todo.todo',
    description='Lightweight YOLOv8 (BBox & Segmentation) + Multi-Mode Person Tracker for TurtleBot4 over ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'front_tracker_node = person_tracker.nodes.front_tracker_node:main',
            'camera_pub = person_tracker.nodes.camera_publisher_node:main',
            'top_cam_tf = person_tracker.nodes.top_cam_tf_node:main',
            'init_pose = person_tracker.nodes.init_pose_node:main',
            'gesture_test_node = person_tracker.debug.gesture_test_node:main',
            'net_monitor = person_tracker.debug.net_monitor_node:main',
            'multi_tracker_node = person_tracker.legacy.multi_tracker_node:main',
            'tracker_node = person_tracker.legacy.tracker_node:main',
            'bbox_tracker_node = person_tracker.legacy.tracker_node:main',
            'seg_tracker_node = person_tracker.legacy.seg_tracker_node:main',
        ],
    },
)
