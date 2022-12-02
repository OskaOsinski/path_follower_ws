#!/usr/bin/env python3

import json
from math import atan2, cos, sin, sqrt

import rospy
import utm
from azsp_msgs.msg import ObstacleAvoidancePath
from geometry_msgs.msg import PoseStamped


class FollowPath:
    """
    This is a class for path convertion to match path follower's standard.
    This class contains methods such as:
    -
    -
    -
    """

    def __init__(self, latitude: float, longitude: float, altitude: float, heading: float):
        self.lat = latitude
        self.lon = longitude
        self.alt = altitude
        self.angle = heading

        self.to_utm_conversion()

    def to_utm_conversion(self):

        utms = utm.from_latlon(self.lon, self.lat)
        self.utm_y = utms[0]
        self.utm_x = utms[1]

    def convert(self, path: ObstacleAvoidancePath):

        path_rotated = []
        point_rotated = PoseStamped()

        for point in path.path_points:

            # x_rotated, y_rotated = self.rotate(point, self.angle)
            point_rotated.header.seq = 0
            point_rotated.header.stamp = rospy.Time.now()
            point_rotated.header.frame_id = "map"

            point_rotated.pose.position.x, point_rotated.pose.position.y = self.rotate(
                point, self.angle
            )
            path_rotated.append(point_rotated)

            # path_rotated.append({"x": x_rotated, "y": y_rotated})

            # CONVERTION TO JSON TYPE
            # path_converted = json.dumps(
            #     {
            #         "gps_ref_alt": self.alt,
            #         "gps_ref_lat": self.lat,
            #         "gps_ref_lon": self.lon,
            #         "utm_ref_x": self.utm_x,
            #         "utm_ref_y": self.utm_y,
            #         "waypoints": path_rotated,
            #     },
            #     indent=4,
            # )

        return path_rotated

    @staticmethod
    def rotate(point, delta: float):

        alfa = atan2(point.y, point.x)
        beta = alfa + delta
        modul = sqrt(point.x**2 + point.y**2)
        x_rotated = modul * cos(beta)
        y_rotated = modul * sin(beta)

        return x_rotated, y_rotated
