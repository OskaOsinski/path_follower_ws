#!/usr/bin/env python

import rospy, rospkg
import time
import driveworks_msgs.msg as driveworks_msgs
import sensor_msgs.msg as sensor_msgs
import visualization_msgs.msg as vis_msg
import std_msgs.msg as std_msgs
import geometry_msgs.msg as geometry_msgs
import math
import numpy as np
from copy import copy, deepcopy

class VersorVisualization:

    def __init__(self):
        rospy.init_node('versor_vis', anonymous=True)
        self.init_topic_names()

    def init_topic_names(self):

        self.pub_versor_vis = rospy.Publisher("versor_vis", vis_msg.MarkerArray, queue_size=0)
        self.objects_distances = driveworks_msgs.EstimatedDistances()
        self.objects_list = driveworks_msgs.DetectedObjectList()
        rospy.Subscriber("/dw/FrontRight_A0_60/objects", driveworks_msgs.DetectedObjectList, self.detections_callback)

    def detections_callback(self, msg_obj_list):
        self.objects_list.data = list(msg_obj_list.data)
        self.main_functions_handler()

    def transform_versors(self):
        pass

    def main_functions_handler(self):
        self.detections_to_markers()

    def get_end_point_list(self, i, scale=20.0):
        end_point_UL = np.array([
            self.objects_list.data[i].versorUpperLeft.x,\
            self.objects_list.data[i].versorUpperLeft.y,\
            self.objects_list.data[i].versorUpperLeft.z,\
            1.0/scale])*scale
        end_point_UR = np.array([
            self.objects_list.data[i].versorUpperRight.x,\
            self.objects_list.data[i].versorUpperRight.y,\
            self.objects_list.data[i].versorUpperRight.z,\
            1.0/scale])*scale
        end_point_BR = np.array([
            self.objects_list.data[i].versorBottomRight.x,\
            self.objects_list.data[i].versorBottomRight.y,\
            self.objects_list.data[i].versorBottomRight.z,\
            1.0/scale])*scale
        end_point_BL = np.array([
            self.objects_list.data[i].versorBottomLeft.x,\
            self.objects_list.data[i].versorBottomLeft.y,\
            self.objects_list.data[i].versorBottomLeft.z,\
            1.0/scale])*scale
        return [end_point_UL, end_point_UR, end_point_BR, end_point_BL]

    def detections_to_markers(self):
        marker_array = vis_msg.MarkerArray()
        geometry_point = geometry_msgs.Point()
        
        for i in range(len(self.objects_list.data)):
            tf_mat = np.asarray(self.objects_list.data[i].actualCameraTrans).reshape((4, 4))
            tf_mat = np.transpose(tf_mat)
            end_point_list = self.get_end_point_list(i)
           
        
            for j in range(len(end_point_list)):
                start_point = np.array([0.0, 0.0, 0.0, 1.0])
                trans_start_point = tf_mat.dot(start_point)
		print("trans_start_point: ", trans_start_point)
                trans_end_point = tf_mat.dot(end_point_list[j])
		print("trans_end_point: ", trans_end_point)
                marker = vis_msg.Marker()
                marker.header.frame_id = "car_rear_axel"
                marker.ns = "basic_shapes"
                marker.lifetime = rospy.Duration(0.1)
                marker.type = vis_msg.Marker.ARROW
                geometry_point.x = trans_start_point[0]
                geometry_point.y = trans_start_point[1]
                geometry_point.z = trans_start_point[2]
                marker.points.append(copy(geometry_point))
                geometry_point.x = trans_end_point[0]
                geometry_point.y = trans_end_point[1]
                geometry_point.z = trans_end_point[2]
                marker.points.append(copy(geometry_point))
                marker.scale.x = 0.07
                marker.scale.y = 0.07
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = (j==1)*1.0 + (j==0)*1.0
                marker.color.g = (j==2)*1.0 + (j==0)*1.0
                marker.color.b = (j==3)*1.0 + (j==0)*1.0
                marker.id = i*len(end_point_list)+j
                marker_array.markers.append(marker)
            
        self.pub_versor_vis.publish(marker_array)

    def run_visualization(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        verVis = VersorVisualization()
        verVis.run_visualization()
    except rospy.ROSInterruptException:
        pass


