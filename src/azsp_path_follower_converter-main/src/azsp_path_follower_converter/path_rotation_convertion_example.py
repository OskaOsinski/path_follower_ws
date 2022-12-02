#!/usr/bin/env python3

import json
from math import atan2, cos, sin, sqrt

import numpy
import rospy
from azsp_msgs.msg import ObstacleAvoidancePath


def rotate(point, delta: float = 0.700341939926):

    alfa = atan2(point.y, point.x)
    beta = alfa + delta
    modul = sqrt(point.x**2 + point.y**2)
    x_rotated = modul * cos(beta)
    y_rotated = modul * sin(beta)

    return x_rotated, y_rotated


def save_to_json(waypoints, save_path):

    with open(
        f"{save_path}/path_rotated.json",
        "w",
    ) as file:

        path = json.dumps(
            {
                "gps_ref_alt": 278.2388000488281,
                "gps_ref_lat": 52.268157,
                "gps_ref_lon": 21.013881,
                "utm_ref_x": 500947.23,
                "utm_ref_y": 5790864.17,
                "waypoints": waypoints,
            },
            indent=4,
        )
        file.write(path)


def callback(arr):

    save_path = rospy.get_param("/save_path")

    lista = arr.path_points

    waypoints = []
    x = []
    y = []
    xr = []
    yr = []

    for point in lista:

        x.append(point.x)
        y.append(point.y)

        x_rotated, y_rotated = rotate(point)
        xr.append(x_rotated)
        yr.append(y_rotated)

        waypoints.append({"x": x_rotated, "y": y_rotated})

    points_org = numpy.asarray([x, y])
    points_org = points_org.T
    numpy.savetxt(f"{save_path}/points_org.csv", points_org, delimiter=",")

    points_rotated = numpy.asarray([xr, yr])
    points_rotated = points_rotated.T

    numpy.savetxt(
        f"{save_path}/points_rotated.csv",
        points_rotated,
        delimiter=",",
    )

    save_to_json(waypoints, save_path)

    return


if __name__ == "__main__":

    rospy.init_node("test_node2", anonymous=True)
    rospy.Subscriber("/obstacle_avoidance_path", ObstacleAvoidancePath, callback)
    rospy.spin()
