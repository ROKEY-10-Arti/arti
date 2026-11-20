from setuptools import setup, find_packages
from glob import glob
# ament_python 패키지 설정: config/*.yaml, launch/*.py 를 share 로 설치
package_name = 'arti_pc2_nav'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='ArtI PC2 team', maintainer_email='maintainer@example.invalid',
      description='ArtI PC2 ROS2 Jazzy course navigation state machine',
      license='Apache-2.0',
      # ros2 run arti_pc2_nav <이름> 으로 실행되는 노드들
      entry_points={'console_scripts': ['a_course_node = arti_pc2_nav.a_course_node:main',
                                                'zone_marker_node = arti_pc2_nav.zone_marker_node:main']})
