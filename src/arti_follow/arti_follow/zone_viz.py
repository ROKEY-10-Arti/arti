"""작품·안내 구역을 RViz 마커로 보여 준다 (T04-2 구역 시각화 확인).

입력  arti_common/config/artworks.yaml (artwork_zone 노드와 같은 파일)
출력  artwork_zone/markers (MarkerArray, frame map, latched)
        파랑 = 작품 구역, 주황 = 안내 구역(출구·키오스크), 반원 화살표 = 정면(방 쪽), 글자 = 이름·ID·좌표
      artwork_zone/markers 는 시작할 때 한 번 + republish_period마다 다시 보낸다 (RViz를 나중에 켜도 보이게)

YAML을 고친 뒤에는 이 노드만 다시 켜면 된다 (빌드는 --symlink-install이면 필요 없음).
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from arti_follow.artwork_zone import load_zones

ART = (0.2, 0.5, 1.0)       # 작품 구역 색
GUIDE = (1.0, 0.6, 0.1)     # 안내 구역 색


class ZoneViz(Node):

    def __init__(self):
        super().__init__('zone_viz')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('republish_period', 2.0)
        path = self.get_parameter('zones_file').value or os.path.join(
            get_package_share_directory('arti_common'), 'config', 'artworks.yaml')
        self.frame = self.get_parameter('map_frame').value
        arts, guides = load_zones(path)
        self.msg = MarkerArray(markers=self.build(arts, ART) + self.build(guides, GUIDE, start=1000))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'artwork_zone/markers', latched)
        self.pub.publish(self.msg)
        self.create_timer(self.get_parameter('republish_period').value, lambda: self.pub.publish(self.msg))
        for z in arts + guides:
            shape = (f'직사각형 x {z.box[0]}~{z.box[1]}, y {z.box[2]}~{z.box[3]}' if z.shape == 'rect' else
                     f'반원 중심 ({z.x:.2f}, {z.y:.2f}), 정면 {math.degrees(math.atan2(z.fy, z.fx)):.0f}°, 반지름 {z.r} m')
            self.get_logger().info(f'{z.name}({z.id}): {shape}' + (f', 안내 "{z.guide}"' if z.guide else ''))
        self.get_logger().info(f'{path} → artwork_zone/markers ({len(arts) + len(guides)}개 구역)')

    def build(self, zones, rgb, start=0):
        ms = []
        for i, z in enumerate(zones):
            base = start + i * 10
            if z.shape == 'rect':
                ms += self.build_rect(z, rgb, base)
                continue
            yaw = math.atan2(z.fy, z.fx)
            arc = [(z.x + z.r * math.cos(yaw + a), z.y + z.r * math.sin(yaw + a))
                   for a in [math.radians(d) for d in range(-90, 91, 6)]]

            fill = self.marker(base, Marker.TRIANGLE_LIST, rgb, 0.25)
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            for a, b in zip(arc, arc[1:]):          # 중심에서 부채꼴 삼각형들
                fill.points += [Point(x=z.x, y=z.y), Point(x=a[0], y=a[1]), Point(x=b[0], y=b[1])]

            edge = self.marker(base + 1, Marker.LINE_STRIP, rgb, 0.9)
            edge.scale.x = 0.03
            edge.points = [Point(x=x, y=y, z=0.01) for x, y in [(z.x, z.y)] + arc + [(z.x, z.y)]]

            arrow = self.marker(base + 2, Marker.ARROW, rgb, 1.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.04, 0.08, 0.1
            tip = (z.x + 0.5 * z.r * z.fx, z.y + 0.5 * z.r * z.fy)
            arrow.points = [Point(x=z.x, y=z.y, z=0.02), Point(x=tip[0], y=tip[1], z=0.02)]

            text = self.marker(base + 3, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 1.0)
            text.scale.z = 0.15
            text.pose.position.x, text.pose.position.y, text.pose.position.z = \
                z.x + 0.4 * z.r * z.fx, z.y + 0.4 * z.r * z.fy, 0.3
            text.text = f'{z.name} {z.id}\n({z.x:.2f}, {z.y:.2f})'
            ms += [fill, edge, arrow, text]
        return ms

    def build_rect(self, z, rgb, base):
        x0, x1, y0, y1 = z.box
        c = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        fill = self.marker(base, Marker.TRIANGLE_LIST, rgb, 0.25)
        fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        fill.points = [Point(x=x, y=y) for x, y in (c[0], c[1], c[2], c[0], c[2], c[3])]
        edge = self.marker(base + 1, Marker.LINE_STRIP, rgb, 0.9)
        edge.scale.x = 0.03
        edge.points = [Point(x=x, y=y, z=0.01) for x, y in c + [c[0]]]
        text = self.marker(base + 3, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 1.0)
        text.scale.z = 0.15
        text.pose.position.x, text.pose.position.y, text.pose.position.z = z.x, z.y, 0.3
        text.text = f'{z.name} {z.id}\nx {x0}~{x1}, y {y0}~{y1}'
        return [fill, edge, text]

    def marker(self, mid, kind, rgb, alpha):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = 'artwork_zone', mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
        return m


def main():
    rclpy.init()
    node = ZoneViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
