#!/usr/bin/env python3

#import json

import rospy
from azsp_msgs.msg import ObstacleAvoidancePath
from gps_msgs.msg import SystemState
from nav_msgs.msg import Path

from azsp_path_follower_converter import FollowPath


class FollowPathNode:
    """
    This is a class responsible for communication with ROS.
    It subscribes to topics:
    -
    -
    -
    and publishes to topics
    -
    -
    -
    """

    def __init__(self):

        self.init_params()
        self.lat = 1  # None
        self.lon = 1  # None
        self.utm_x = None
        self.utm_y = None
        self.yaw = 0  # None
        self.alt = 0  # None

        rospy.init_node("PathConverter")
        self.subs_init()

    def init_params(self):
        self.path_topic = rospy.get_param("/avoidance_path_topic")
        self.gps_topic = rospy.get_param("/gps_topic")
        # self.path_topic = rospy.get_param("")

    def subs_init(self):
        # GPS message subscriber
        rospy.Subscriber(self.gps_topic, SystemState, self.gps_callback)
        # Avoidance path subscriber
        rospy.Subscriber(self.path_topic, ObstacleAvoidancePath, self.path_converter_callback)
        rospy.spin()

    def gps_callback(self, gps_msg: SystemState):
        self.lat = gps_msg.nav_sat_fix.latitude
        self.lon = gps_msg.nav_sat_fix.longitude
        self.alt = gps_msg.nav_sat_fix.height
        self.yaw = 0 # gps_msg.orientation_rad.heading

    def path_converter_callback(self, path_msg: ObstacleAvoidancePath):
        converter = FollowPath(self.lat, self.lon, self.alt, self.yaw)
        self.converted_path = converter.convert(path_msg)
        # rospy.loginfo(self.converted_path)
        self.path_publisher()

    def path_publisher(self):
       
        rospy.loginfo("before publisher")
        pub_path = rospy.Publisher("path_to_follow", Path, queue_size=10)
        path_msg = Path()

        path_msg.header.seq = 0
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        path_msg.poses = self.converted_path
        rospy.loginfo(path_msg)
        pub_path.publish(path_msg)
        rospy.loginfo("message sent")


def main():
    start = FollowPathNode()


if __name__ == "__main__":
    main()
