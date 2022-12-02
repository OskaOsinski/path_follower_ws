#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker

class RosPublisher:
    def __init__(self, rate):
        self._rate = rospy.Rate(rate)

    @property
    def rate(self):
        return self._rate

    def initialize_publisher(self):
        self.triangles = rospy.Publisher('triangle_list', \
            MarkerArray, queue_size=10)

    def pub_triangles(self, trians):
        self.triangles.publish(trians)
