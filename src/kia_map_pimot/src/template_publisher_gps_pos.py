#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import numpy as np
import utm
import json
import rospy
from gps_msgs.msg import SystemState
from std_msgs.msg import String
from std_msgs.msg import Bool


class PublishPath:
    def __init__(self):
        self.message = False
        self.data_ = None
        self.count = 0
        rospy.init_node('talker_gps_node')
        pub_topic = rospy.get_param('actual_kia_position_topic')
        sub_topic = rospy.get_param('start_topic')
        pub = rospy.Publisher(pub_topic, SystemState, queue_size=2)
        rospy.Subscriber(sub_topic, Bool, self.callback)
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            if self.message:
                msg = SystemState()
                if self.count >= len(self.data_['waypoints']):
                    self.message = False
                    self.count = 0
                    continue
                if self.count == 0:
                    y = self.data_['gps_ref_lat']
                    x = self.data_['gps_ref_lon']
                    u = utm.from_latlon(y, x)
                dx = self.data_['waypoints'][self.count]['x']
                dy = self.data_['waypoints'][self.count]['y']
                utm_1 = u[0]+dx
                utm_2 = u[1]+dy
                xx = utm.to_latlon(utm_1, utm_2, 34, 'U')
                msg.nav_sat_fix.longitude = xx[1]
                msg.nav_sat_fix.latitude = xx[0]
                pub.publish(msg)
                self.count +=1
            rate.sleep()

    def callback(self,data):
        f = open(rospy.get_param('path_follower_path'))
        self.data_ = json.load(f)
        f.close()
        self.message = True

if __name__ == '__main__':
    try:
        PublishPath()
    except rospy.ROSInterruptException:
        pass