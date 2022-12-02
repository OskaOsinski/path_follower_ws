#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import PolygonArray
import kml_parser_publisher.msg as kml_parser_msgs

class RosCommunication:
    def __init__(self, rate):
        self._rate = rospy.Rate(rate)

    @property
    def rate(self):
        return self._rate

    def initialize_publisher(self):
        self.grid_points = rospy.Publisher('kml_points_to_grid', \
            kml_parser_msgs.kmlPointArray, queue_size=1000)
        self.grid_lines = rospy.Publisher('kml_lines_to_grid', \
            kml_parser_msgs.kmlLineStringArray, queue_size=1000)
        self.grid_polygons = rospy.Publisher('kml_polygons_to_grid', \
            kml_parser_msgs.kmlPolygonArray, queue_size=1000)

        self.points_viz = rospy.Publisher('kml_visualization_points', \
            MarkerArray, queue_size=1000)
        self.polygons_viz = rospy.Publisher('kml_visualization_polygons', \
            PolygonArray, queue_size=1000)
        self.paths_viz = rospy.Publisher('kml_visualization_paths', \
            MarkerArray, queue_size=1000)

    def pub_to_grid(self, msg_pts, msg_lines, msg_poly):
        self.grid_points.publish(msg_pts)
        self.grid_lines.publish(msg_lines)
        self.grid_polygons.publish(msg_poly)

    def pub_to_vis(self, pts, linrd, poly):
        self.points_viz.publish(pts)
        self.paths_viz.publish(linrd)
        self.polygons_viz.publish(poly)

    def sleep(self):
        self._rate.sleep()