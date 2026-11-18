"""시험용 가짜 PC3 작품 설명 서버 — PC3 없이 guide_audio_node를 시험할 때만 띄운다.

/arti/description (arti_interfaces/srv/Description): 작품 번호 → config/artworks_sample.yaml의 description.
없는 번호면 success=false. 실제 관람에서는 PC3(키오스크/관제)의 서버를 쓴다 (docs/common/ros2_interfaces.md).

  ros2 run arti_audio fake_description_server
  ros2 run arti_audio fake_description_server --ros-args -p delay_sec:=5.0   # 응답 늦음(시간 초과) 시험
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from arti_interfaces.srv import Description
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from arti_audio.content import sample_descriptions


class FakeDescriptionServer(Node):

    def __init__(self):
        super().__init__('fake_description_server')
        self.declare_parameter('artworks_file', '')     # 비우면 share/arti_audio/config/artworks_sample.yaml
        self.declare_parameter('service', '/arti/description')
        self.declare_parameter('delay_sec', 0.0)         # 응답 전에 기다릴 시간 (시간 초과 시험)
        path = self.get_parameter('artworks_file').value or os.path.join(
            get_package_share_directory('arti_audio'), 'config', 'artworks_sample.yaml')
        self.texts = sample_descriptions(path)
        self.delay = float(self.get_parameter('delay_sec').value)
        srv = self.create_service(Description, self.get_parameter('service').value, self.on_request)
        self.get_logger().info(f'가짜 작품 설명 서버: {srv.srv_name}, 작품 {sorted(self.texts)} ({path})')

    def on_request(self, request, response):
        if self.delay > 0:
            time.sleep(self.delay)
        text = self.texts.get(request.artwork_id, '')
        response.success = bool(text)
        response.description = text
        self.get_logger().info(f'요청 {request.artwork_id} → success={response.success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeDescriptionServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
