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

def callback(data):
    global data_,messages
    f = open('/home/mateusz/catkin_ws/src/template_gui_package-main/nodes/path/sciezka.json')
    data_ = json.load(f)
    f.close()
    messages = True


def move():
    global data_,messages
    messages = False
    count = 0
    x = 0.0
    y = 0.0
    # START
    # 2 lines init node and publisher
    rospy.init_node('talker_gps_node')
    pub = rospy.Publisher("/gps_state", SystemState, queue_size=2)
    rospy.Subscriber("/valid_path", Bool, callback)
    rate = rospy.Rate(50) # Hz
    # END
    while not rospy.is_shutdown():
        if messages:
            msg = SystemState()
            if count >= len(data_['waypoints']):
                messages = False
                count = 0
                continue
            if count == 0:
                y = data_['gps_ref_lat']
                x = data_['gps_ref_lon']
                u = utm.from_latlon(y, x)
            dx = data_['waypoints'][count]['x']
            dy = data_['waypoints'][count]['y']
            utm_1 = u[0]+dx
            utm_2 = u[1]+dy
            #print(utm_1,utm_2)
            xx = utm.to_latlon(utm_1, utm_2, 34, 'U')
            #print(xx)
            msg.nav_sat_fix.longitude = xx[1]
            msg.nav_sat_fix.latitude = xx[0]
            # msg.nav_sat_fix.longitude = x + dx*2.47/230000.0
            # msg.nav_sat_fix.latitude = y + dy*1.87/230000.0
            pub.publish(msg)
            count +=1
        rate.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass



#   y = data["gps_ref_lat"];
#   x = data["gps_ref_lon"];
#   float prev_x = 0.0;
#   float prev_y = 0.0;
#   //std::cout << data["waypoints"].size() << std::endl;
#   while (ros::ok())
#   {
#     gps_msgs::SystemState msg;

#     if(count >= data["waypoints"].size() ){
#       count = 0;
#     }
#     if(count == 0){
#         dx = 0;
#         dy = 0;
#     }
#     else{
#       dy = data["waypoints"][count]["y"];
#       dx = data["waypoints"][count]["x"];
#     }
#     //std::cout<<data["gps_ref_lat"] << std::endl;
#     //std::cout<<data["gps_ref_lon"] << std::endl;
    
#     //msg.x = 21.014118194580078 + (21.01401122-21.014585214049692)/10*count;
#     //msg.y = 52.26820373535156 + (52.26802585-52.26869226538378)/10*count;

#     msg.nav_sat_fix.longitude = x + dx*2.47/230000.0;
#     msg.nav_sat_fix.latitude = y + dy*1.87/230000.0;