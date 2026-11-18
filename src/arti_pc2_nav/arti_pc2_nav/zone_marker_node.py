"""RViz 반원 마커 노드: 키오스크/작품 반원 + 출구 반원을 표시.

토픽: arti/a_course/zones (visualization_msgs/MarkerArray, transient_local)
RViz에서 Add -> By topic -> /robot4/arti/a_course/zones 로 볼 수 있습니다.
표시 전용이며 로봇 동작에는 영향이 없습니다.
"""
from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .zone_geometry import exit_semicircle, boundary_points
from .guided_tour_plan import load_course_config


def point(x, y, z=0.06):
    """geometry_msgs/Point 생성 헬퍼."""
    p = Point(); p.x = float(x); p.y = float(y); p.z = float(z)
    return p


def rgba(red, green, blue, alpha):
    """std_msgs/ColorRGBA 생성 헬퍼."""
    from std_msgs.msg import ColorRGBA
    c = ColorRGBA(); c.r = red; c.g = green; c.b = blue; c.a = alpha
    return c


class ZoneMarkerNode(Node):
    def __init__(self):
        super().__init__('arti_zone_markers')
        self.declare_parameter('config_file', '')
        self.declare_parameter('map_frame', 'map')
        # 나중에 켠 RViz도 마지막 마커를 받을 수 있도록 transient_local
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'arti/a_course/zones', qos)
        # (라벨, 반원, 색상) 목록. 출구는 항상 표시(주황)
        self.zones = [('EXIT', exit_semicircle(), (1., .42, .08))]
        filename = self.get_parameter('config_file').value
        if filename:
            try:
                cfg = load_course_config(filename)
                self.zones.append(('KIOSK', cfg.kiosk_zone, (.05, .75, .95)))     # 키오스크: 파랑
                self.zones.extend((f'ART {a.artwork_id}', a.zone, (.12, .92, .22)) for a in cfg.artworks)  # 작품: 초록
            except Exception as exc:
                self.get_logger().warn(f'작품 좌표가 아직 준비되지 않아 출구만 표시: {exc}')
        self.timer = self.create_timer(1.0, self.publish_markers)   # 1초마다 재발행
        self.publish_markers()

    def publish_markers(self):
        """반원마다 마커 3개(채우기, 외곽선, 라벨)를 만들어 발행."""
        frame = self.get_parameter('map_frame').value
        arr = MarkerArray()
        for i, (label, zone, color) in enumerate(self.zones):
            arc = boundary_points(zone, 40)
            # 채우기: 중심에서 호로 부채꼴 삼각형들. 지도를 가리지 않게 투명도 낮게.
            fill = self._marker(frame, i*3, Marker.TRIANGLE_LIST)
            fill.color = rgba(*color, .10)
            for k in range(len(arc)-1):
                fill.points.extend([point(zone.cx, zone.cy, .04), point(*arc[k], .04), point(*arc[k+1], .04)])
            arr.markers.append(fill)
            # 외곽선: 호 + 지름(첫 점으로 되돌아가 닫음)
            outline = self._marker(frame, i*3+1, Marker.LINE_STRIP)
            outline.scale.x = .028
            outline.color = rgba(*color, .95)
            outline.points = [point(*xy) for xy in arc+[arc[0]]]
            arr.markers.append(outline)
            # 라벨 텍스트
            title = self._marker(frame, i*3+2, Marker.TEXT_VIEW_FACING)
            title.scale.z = .15
            title.pose.position = point(zone.cx, zone.cy, .25)
            title.color = rgba(*color, 1.)
            title.text = label
            arr.markers.append(title)
        self.pub.publish(arr)

    def _marker(self, frame, mid, marker_type):
        """공통 필드를 채운 Marker 생성 (lifetime 0 = 계속 유지)."""
        msg = Marker()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'arti_pc2_zones'
        msg.id = mid
        msg.type = marker_type
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = 1.0; msg.scale.y = 1.0; msg.scale.z = 1.0
        msg.lifetime.sec = 0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ZoneMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
