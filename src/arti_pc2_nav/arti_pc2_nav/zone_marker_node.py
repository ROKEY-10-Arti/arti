"""RViz 구역 마커 노드: 키오스크/작품 반원 + 출구 직사각형(arti_common, follow 팀과 같은 구역)을 표시.

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
from .zone_geometry import boundary_points
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
        # (라벨, 중심 (x, y), 외곽점 목록, 색상). 키오스크·작품 = a_course.yaml, 출구 = arti_common
        self.zones = []
        filename = self.get_parameter('config_file').value
        if filename:
            try:
                cfg = load_course_config(filename)
                ez = cfg.exit_zone
                if ez is not None and ez.shape == 'rect':
                    x0, x1, y0, y1 = ez.box
                    self.zones.append(('EXIT', (ez.x, ez.y), [(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                                       (1., .42, .08)))                                   # 출구: 주황 직사각형
                self.zones.append(self._semi('KIOSK', cfg.kiosk_zone, (.05, .75, .95)))     # 키오스크: 파랑
                self.zones.extend(self._semi(f'ART {a.artwork_id}', a.zone, (.12, .92, .22))
                                  for a in cfg.artworks)                                  # 작품: 초록
            except Exception as exc:
                self.get_logger().warn(f'a_course.yaml을 읽지 못해 반원을 표시하지 않음: {exc}')
        self.timer = self.create_timer(1.0, self.publish_markers)   # 1초마다 재발행
        self.publish_markers()

    def publish_markers(self):
        """반원마다 마커 3개(채우기, 외곽선, 라벨)를 만들어 발행."""
        frame = self.get_parameter('map_frame').value
        arr = MarkerArray()
        for i, (label, (cx, cy), arc, color) in enumerate(self.zones):
            # 채우기: 중심에서 외곽점으로 부채꼴 삼각형들 (반원·직사각형 모두 볼록). 지도를 가리지 않게 투명도 낮게.
            fill = self._marker(frame, i*3, Marker.TRIANGLE_LIST)
            fill.color = rgba(*color, .10)
            ring = arc + [arc[0]]
            for k in range(len(ring)-1):
                fill.points.extend([point(cx, cy, .04), point(*ring[k], .04), point(*ring[k+1], .04)])
            arr.markers.append(fill)
            # 외곽선: 첫 점으로 되돌아가 닫음
            outline = self._marker(frame, i*3+1, Marker.LINE_STRIP)
            outline.scale.x = .028
            outline.color = rgba(*color, .95)
            outline.points = [point(*xy) for xy in arc+[arc[0]]]
            arr.markers.append(outline)
            # 라벨 텍스트
            title = self._marker(frame, i*3+2, Marker.TEXT_VIEW_FACING)
            title.scale.z = .15
            title.pose.position = point(cx, cy, .25)
            title.color = rgba(*color, 1.)
            title.text = label
            arr.markers.append(title)
        self.pub.publish(arr)

    @staticmethod
    def _semi(label, zone, color):
        """반원 -> (라벨, 중심, 호 외곽점 41개, 색상)."""
        return (label, (zone.cx, zone.cy), boundary_points(zone, 40), color)

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
