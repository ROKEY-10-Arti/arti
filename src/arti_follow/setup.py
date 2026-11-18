from glob import glob

from setuptools import find_packages, setup

package_name = 'arti_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROKEY-10-Arti',
    maintainer_email='arti@todo.invalid',
    description='Arti user following: register, track, keep distance, lost handling',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visitor_follower = arti_follow.visitor_follower:main',
            'nav2_follow = arti_follow.nav2_follow:main',
            'artwork_zone = arti_follow.artwork_zone:main',
            'zone_viz = arti_follow.zone_viz:main',
        ],
    },
)
