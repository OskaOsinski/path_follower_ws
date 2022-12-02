#!/usr/bin/env python
import rospy
import cv2
import utm
import json
import numpy as np
from azsp_msgs.msg import FloatPoint



def callback(data):
    #print(' ')
    # wspolrzedne obrazu
    xmin = 21.0110884233579
    xmax = 21.0190682401996
    ymin = 52.2679688744638
    ymax = 52.2701910569562

    point_A = (386,259)
    point_B = (int(data.x),int(data.y))

    w = point_A[0]/880.0 * (xmax-xmin) + xmin
    h = (1-point_A[1]/404.0)* (ymax-ymin) + ymin
    # print(h,w)
    u = utm.from_latlon(h, w)

    wB = point_B[0]/880.0 * (xmax-xmin) + xmin
    hB = (1-point_B[1]/404.0)* (ymax-ymin) + ymin
    #print(hB,wB)
    uB = utm.from_latlon(hB, wB)
    print(uB[0]-u[0])
    print(uB[1]-u[1])
    dist = np.sqrt(np.power(uB[0]-u[0],2)+np.power(uB[1]-u[1],2)) 
    krok = 0.3
    ilosc = int(dist / krok)
    #print(ilosc)
    data = {
        'gps_ref_lat': h, 
        'gps_ref_lon': w,
        'utm_ref_x': u[0], 
        'utm_ref_y': u[1],
        'waypoints' : [
        ]
    }
    for i in range(int(ilosc)):
        entry = {'x': (i+1)/1.0/ilosc*(uB[0]-u[0]),'y': (i+1)/1.0/ilosc*(uB[1]-u[1])}
        data['waypoints'].append(entry)
    
    with open('/home/mateusz/catkin_ws/src/template_gui_package-main/nodes/path/sciezka.json', 'w') as f:
        json.dump(data, f, indent=2)
    f.close()
   

def move():
    # START
    # 2 lines init node and publisher
    rospy.init_node('create_path_node')
    rospy.Subscriber("/gps_setpoint", FloatPoint, callback)
    rate = rospy.Rate(5) # Hz
    # END


    while not rospy.is_shutdown():
       
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
