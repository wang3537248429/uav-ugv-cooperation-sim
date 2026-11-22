#!/usr/bin/env python3
"""Publish Gazebo-consistent competition field markers for RViz."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, Int32
from visualization_msgs.msg import Marker, MarkerArray

from .competition_paths import (
    UAV_REFERENCE_WAYPOINTS_NED,
    UGV_GATE_SEQUENCE,
    UGV_PATH_GAZEBO,
    UGV_ROUTE_WAYPOINTS,
    UGV_STRIKE_GAZEBO_X,
    UGV_STRIKE_GAZEBO_Y,
    gazebo_xy_to_rviz_map,
    local_ned_to_rviz_map,
)


class CompetitionFieldMarkers(Node):
    def __init__(self):
        super().__init__("competition_field_markers")
        self.declare_parameter("frame_id", "uav_map")
        self.declare_parameter("topic", "/sim/competition_field_markers")
        self.declare_parameter("coordinate_frame", "rviz_map")
        self.declare_parameter("show_field", True)
        self.declare_parameter("show_ugv_route", True)
        # /mission/waypoints already publishes the live UAV route. Keep the
        # optional static reference off by default so labels are not doubled.
        self.declare_parameter("show_uav_reference", False)
        self.declare_parameter("show_aruco", True)
        self.declare_parameter("ugv_map_shift_x", 0.0)
        self.declare_parameter("ugv_map_shift_y", 0.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        self.coordinate_frame = str(self.get_parameter("coordinate_frame").value)
        self.show_field = bool(self.get_parameter("show_field").value)
        self.show_ugv_route = bool(self.get_parameter("show_ugv_route").value)
        self.show_uav_reference = bool(self.get_parameter("show_uav_reference").value)
        self.show_aruco = bool(self.get_parameter("show_aruco").value)
        self.ugv_map_shift_x = float(self.get_parameter("ugv_map_shift_x").value)
        self.ugv_map_shift_y = float(self.get_parameter("ugv_map_shift_y").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, topic, qos)
        self.aruco_id = '?'
        self.create_subscription(Int32, '/simulation/aruco_id', self._aruco_cb, qos)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info(
            f"RViz场地标记已发布: {topic}, frame={self.frame_id}, "
            f"coordinate_frame={self.coordinate_frame}"
        )

    def _header(self, marker):
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

    def _aruco_cb(self, message):
        self.aruco_id = str(message.data)
        self.publish_markers()

    def _pt_gz(self, x_gz, y_gz, z=0.04):
        if self.coordinate_frame in ("gazebo", "ugv_map"):
            x, y = float(x_gz), float(y_gz)
            if self.coordinate_frame == "ugv_map":
                x += self.ugv_map_shift_x
                y += self.ugv_map_shift_y
        else:
            x, y = gazebo_xy_to_rviz_map(float(x_gz), float(y_gz))
        return Point(x=x, y=y, z=float(z))

    def _pt_ned(self, x_ned, y_ned, z=0.08):
        x, y = local_ned_to_rviz_map(float(x_ned), float(y_ned))
        return Point(x=x, y=y, z=float(z))

    def _color(self, r, g, b, a=1.0):
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

    def _line_strip(self, ns, mid, points, color, width=0.05):
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color = color
        marker.points = points
        return marker

    def _arrow(self, ns, mid, start, end, color):
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.05, y=0.16, z=0.20)
        marker.color = color
        marker.points = [start, end]
        return marker

    def _sphere(self, ns, mid, point, color, diameter=0.18):
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=diameter, y=diameter, z=diameter)
        marker.color = color
        return marker

    def _cube(self, ns, mid, point, scale, color):
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale = scale
        marker.color = color
        return marker

    def _text(self, ns, mid, point, text, color, size=0.22):
        marker = Marker()
        self._header(marker)
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=point.x, y=point.y, z=point.z + 0.18)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(size)
        marker.color = color
        marker.text = text
        return marker

    def publish_markers(self):
        markers = []

        if self.show_field:
            field = [
                self._pt_gz(-3.0, -2.5, 0.02),
                self._pt_gz(3.0, -2.5, 0.02),
                self._pt_gz(3.0, 2.5, 0.02),
                self._pt_gz(-3.0, 2.5, 0.02),
                self._pt_gz(-3.0, -2.5, 0.02),
            ]
            markers.append(
                self._line_strip(
                    "field_boundary",
                    1,
                    field,
                    self._color(0.85, 0.85, 0.85, 1.0),
                    0.04,
                )
            )

            start_center = self._pt_gz(-2.25, 0.0, 0.015)
            markers.append(
                self._cube(
                    "zones",
                    10,
                    start_center,
                    Vector3(x=1.5, y=5.0, z=0.02),
                    self._color(0.0, 0.8, 0.1, 0.20),
                )
            )
            markers.append(
                self._sphere(
                    "origins",
                    11,
                    self._pt_gz(-2.25, 0.0, 0.12),
                    self._color(0.2, 1.0, 0.2, 1.0),
                    0.18,
                )
            )
            markers.append(
                self._text(
                    "zone_labels",
                    12,
                    self._pt_gz(-2.25, -2.15, 0.06),
                    "UAV MAP ORIGIN / TAKEOFF",
                    self._color(0.7, 1.0, 0.7, 1.0),
                    0.18,
                )
            )
            markers.append(
                self._sphere(
                    "origins",
                    13,
                    self._pt_gz(0.0, 0.0, 0.12),
                    self._color(1.0, 1.0, 1.0, 1.0),
                    0.16,
                )
            )
            markers.append(
                self._text(
                    "origin_labels",
                    14,
                    self._pt_gz(0.0, 0.0, 0.10),
                    "GAZEBO ORIGIN",
                    self._color(1.0, 1.0, 1.0, 1.0),
                    0.16,
                )
            )

        if self.show_ugv_route:
            ugv_points = [self._pt_gz(x, y, 0.07) for x, y in UGV_PATH_GAZEBO]
            markers.append(
                self._line_strip(
                    "ugv_route",
                    20,
                    ugv_points,
                    self._color(0.1, 0.55, 1.0, 1.0),
                    0.08,
                )
            )
            # Keep the rounded line, but only draw arrows/balls at major points.
            # Do not create two separate render objects for every curve sample.
            route_points = [self._pt_gz(x, y, 0.07) for x, y in UGV_ROUTE_WAYPOINTS]
            for i, (start, end) in enumerate(zip(route_points, route_points[1:]), start=21):
                markers.append(
                    self._arrow(
                        "ugv_direction",
                        i,
                        start,
                        end,
                        self._color(0.1, 0.55, 1.0, 0.9),
                    )
                )
            for i, point in enumerate(route_points):
                markers.append(
                    self._sphere(
                        "ugv_route_points",
                        40 + i,
                        point,
                        self._color(0.0, 0.35, 1.0, 1.0),
                        0.14,
                    )
                )

            for i, (label, x_gz, y_gz, _) in enumerate(UGV_GATE_SEQUENCE, start=1):
                point = self._pt_gz(x_gz, y_gz, 0.12)
                markers.append(
                    self._sphere(
                        "ugv_gates",
                        60 + i,
                        point,
                        self._color(1.0, 0.85, 0.0, 1.0),
                        0.22,
                    )
                )
                markers.append(
                    self._text(
                        "ugv_gate_labels",
                        70 + i,
                        point,
                        label,
                        self._color(1.0, 0.92, 0.25, 1.0),
                        0.18,
                    )
                )

            strike = self._pt_gz(UGV_STRIKE_GAZEBO_X, UGV_STRIKE_GAZEBO_Y, 0.13)
            markers.append(
                self._sphere(
                    "strike_zone",
                    90,
                    strike,
                    self._color(1.0, 0.05, 0.05, 1.0),
                    0.26,
                )
            )
            markers.append(
                self._text(
                    "strike_labels",
                    91,
                    strike,
                    "UGV STOP / STRIKE",
                    self._color(1.0, 0.35, 0.35, 1.0),
                    0.18,
                )
            )

        if self.show_uav_reference:
            uav_points = [
                self._pt_ned(x_ned, y_ned, 0.10)
                for x_ned, y_ned, z, _ in UAV_REFERENCE_WAYPOINTS_NED
                if z > 0.0
            ]
            markers.append(
                self._line_strip(
                    "uav_route_reference",
                    100,
                    uav_points,
                    self._color(1.0, 0.45, 0.0, 1.0),
                    0.06,
                )
            )
            for i, (x_ned, y_ned, z, label) in enumerate(UAV_REFERENCE_WAYPOINTS_NED):
                point = self._pt_ned(x_ned, y_ned, 0.11)
                color = (
                    self._color(1.0, 0.45, 0.0, 1.0)
                    if z > 0.0
                    else self._color(0.8, 0.8, 0.8, 0.8)
                )
                markers.append(
                    self._sphere(
                        "uav_reference_points", 120 + i, point, color, 0.13
                    )
                )
                markers.append(
                    self._text(
                        "uav_reference_labels",
                        140 + i,
                        point,
                        f"UAV {label}",
                        color,
                        0.16,
                    )
                )

        if self.show_aruco:
            aruco_point = self._pt_gz(1.75, 0.0, 0.08)
            markers.append(
                self._sphere(
                    "aruco",
                    180,
                    aruco_point,
                    self._color(0.1, 1.0, 0.25, 1.0),
                    0.18,
                )
            )
            markers.append(
                self._text(
                    "aruco_labels",
                    181,
                    aruco_point,
                    f"ARUCO:{self.aruco_id}",
                    self._color(0.55, 1.0, 0.55, 1.0),
                    0.18,
                )
            )

        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = CompetitionFieldMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
