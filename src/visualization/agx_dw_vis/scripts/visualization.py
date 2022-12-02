#!/usr/bin/env python

import rospy, rospkg
import time
import driveworks_msgs.msg as driveworks_msgs
import sensor_msgs.msg as sensor_msgs
import visualization_msgs.msg as vis_msg
import std_msgs.msg as std_msgs
import math
import numpy as np

class DwVisualization:

    def __init__(self):
        rospy.init_node('agx_dw_vis', anonymous=True)
        self.init_topic_names()
        self.load_obj_type_data()

    def init_topic_names(self):
        dwTopicNames = rospy.get_param('~/agx_dw_vis/dw_topic_names')
        self.data_signs_path = rospy.get_param('~/agx_dw_vis/data_signs_path')
        self.data_objects_types_path = rospy.get_param('~/agx_dw_vis/data_objects_types')

        rospy.Subscriber(dwTopicNames,  driveworks_msgs.DatectionTopicsNames, self.c_topics_name)
        rospy.Subscriber("/objects_distances",  driveworks_msgs.EstimatedDistances, self.obj_dist_callback)

        rospy.Subscriber("/detection_pos",  sensor_msgs.PointCloud, self.obj_pos_callback)

        self.pub_vis_objects = rospy.Publisher("object_sign", vis_msg.MarkerArray, queue_size=0)
        self.pub_vis_lines = rospy.Publisher("lines", vis_msg.MarkerArray, queue_size=0)
        self.objects_distances = driveworks_msgs.EstimatedDistances()
        self.objects_pos = sensor_msgs.PointCloud()
        self.objects_list = driveworks_msgs.DetectedObjectList()
        self.NAME_DIST_ABOVE_OBJ = 0.7

    
    def c_topics_name(self, topic_names):
        for subName in topic_names.topicNames:
            splitTopic = subName.split("/")
            splitTopic[1] = "rviz"
            rvizTopicName = '/'.join(splitTopic)
            if splitTopic[2] == "objects":
                rospy.Subscriber(subName,  driveworks_msgs.DetectedObjectList, self.c_received_obj)
              #  self._pub_obj = rospy.Publisher(rvizTopicName, OccupancyGrid, queue_size=1000)
            elif splitTopic[2] == "lines":
                rospy.Subscriber(subName,  driveworks_msgs.DetectedLaneList, self.c_received_lane)
                '/'.join(splitTopic) 
               # self._pub_obj = rospy.Publisher(rvizTopicName, OccupancyGrid, queue_size=1000)
            elif splitTopic[2] == "video":
                rospy.Subscriber(subName,  sensor_msgs.Image, self.c_received_image)
                '/'.join(splitTopic)
                #self._pub_obj = rospy.Publisher(rvizTopicName, OccupancyGrid, queue_size=1000)
            else:
                print("Not known callback")
    
    def obj_dist_callback(self, msg):
        del self.objects_distances.data[:]
        self.objects_distances.data = list(msg.data)

    def obj_pos_callback(self, msg):
        del self.objects_pos.points[:]
        self.objects_pos.points = list(msg.points)

    def fake_obj_dist(self):
	pass       
	#del self.objects_distances.data[:]
        #[self.objects_distances.data.append(el+10) for el in range(len(self.objects_list.data))]

    def load_obj_type_data(self):
        d = {}
        with open(self.data_signs_path) as file:
            content = file.readlines()
            for line in content:
                (val, key) = line.split(":")
                d[int(key[-3:])] = val
                self.signs_data = d
        d = {}
        with open(self.data_objects_types_path) as file:
            content = file.readlines()
            for line in content:
                (val, key) = line.split(":")
                d[int(key[-3:])] = val
                self.data_objects_types = d

    def get_object_position(self, ind):
        x_v = (self.objects_list.data[ind].versorUpperLeft.x + \
            self.objects_list.data[ind].versorUpperRight.x + \
            self.objects_list.data[ind].versorBottomLeft.x + \
            self.objects_list.data[ind].versorBottomRight.x)/4.0

        y_v = (self.objects_list.data[ind].versorUpperLeft.y + \
            self.objects_list.data[ind].versorUpperRight.y + \
            self.objects_list.data[ind].versorBottomLeft.y + \
            self.objects_list.data[ind].versorBottomRight.y)/4.0

        z_v = (self.objects_list.data[ind].versorUpperLeft.z + \
            self.objects_list.data[ind].versorUpperRight.z + \
            self.objects_list.data[ind].versorBottomLeft.z + \
            self.objects_list.data[ind].versorBottomRight.z)/4.0
    
        #(x_v, y_v, z_v) = self.apply_transform([(x_v, y_v, z_v)])[0]

        if(np.isnan(x_v)): x_v = 0.0
        if(np.isnan(y_v)): y_v = 0.0
        if(np.isnan(z_v)): z_v = 0.0

        xy_angle = math.atan2(y_v, x_v)
        if(xy_angle < 0):
            xy_angle = 2.0*math.pi + xy_angle
        xz_angle = math.atan2(z_v, x_v)
        if(xz_angle < 0):
            xz_angle = 2.0*math.pi + xz_angle

        y = math.sin(xy_angle)*self.objects_distances.data[ind]
        x = math.cos(xy_angle)*self.objects_distances.data[ind]
        z = math.sin(xz_angle)*self.objects_distances.data[ind]


        return (x, y, z)


    def get_objects_positions(self):
        if(len(self.objects_list.data) == len(self.objects_distances.data)):
            return [self.get_object_position(ind) for ind in range(len(self.objects_list.data)) ]
        else:
            print("ERROR: objects_list and objects_distances are lists of different size\n")
            return []

    def apply_transform(self, points_vect):
        tf_matrix = self.tf_matrix
        w = 0.0;
        transformed_list = []
        for point in points_vect:
            x = tf_matrix[0] * point[0] + tf_matrix[4] * point[1] +\
                tf_matrix[8] * point[2] + tf_matrix[12] * w;
            y = tf_matrix[1] * point[0] + tf_matrix[5] * point[1] +\
                tf_matrix[9] * point[2] + tf_matrix[13] * w;
            z = tf_matrix[2] * point[0] + tf_matrix[6] * point[1] +\
                tf_matrix[10] * point[2] + tf_matrix[14] * w;
            p_x = x;
            p_y = y;
            p_z = z;
            transformed_list.append((p_x, p_y, p_z))
        return transformed_list

    def get_objects_positions2(self):
        L = []
        for p in self.objects_pos.points:
            L.append((p.x, p.y, p.z))
        return L

    def visualise_detected_objects(self, msg_obj_list):
        del self.objects_list.data[:]
        self.objects_list.data = list(msg_obj_list.data)
        
        if len(self.objects_list.data) > 0:
            self.tf_matrix = self.objects_list.data[0].actualCameraTrans
        
        
        ##change
        objects_positions = self.get_objects_positions2()
        ##change

        marker_array = vis_msg.MarkerArray()
        if len(objects_positions) == len(msg_obj_list.data):
            for i in range(len(msg_obj_list.data)):
                for j in range(2):
                    marker = vis_msg.Marker()
                    marker.header.frame_id = "car_rear_axel"
                    #marker.header.stamp = ros.Time.now()
                    marker.ns = "basic_shapes"
                    marker.id = i*2 + j
                    marker.lifetime = rospy.Duration(0.1)

                    #print(j)
                    if j == 0:
                        marker.type = vis_msg.Marker.TEXT_VIEW_FACING
                        marker.pose.position.x = objects_positions[i][0]
                        marker.pose.position.y = objects_positions[i][1]
                        marker.pose.position.z = objects_positions[i][2] + self.NAME_DIST_ABOVE_OBJ
                        
                        if self.objects_list.data[i].type == 1:
                            marker.text = "".join([ str( self.data_objects_types[self.objects_list.data[i].type] )])    #, \
                                #str("\n"),
                                #str( self.signs_data[self.objects_list.data[i].subtype] )])
                        else:
                            marker.text = str( self.data_objects_types[self.objects_list.data[i].type])
                    else:
                        marker.type = vis_msg.Marker.SPHERE
                        marker.pose.position.x = objects_positions[i][0]
                        marker.pose.position.y = objects_positions[i][1]
                        marker.pose.position.z = objects_positions[i][2]

                    #marker.action = vis_msg.Marker.ADD

                    marker.scale.x = 0.3
                    marker.scale.y = 0.3
                    marker.scale.z = 0.3

                    marker.color.r = 25 / 255.0
                    marker.color.g = 255 / 255.0
                    marker.color.b = 240 / 255.0
                    marker.color.a = 1.0
                    marker_array.markers.append(marker)

        self.pub_vis_objects.publish(marker_array)
        

    def visualise_lines(self, msg):
        #self.pub_vis_lines
        marker_array = vis_msg.MarkerArray()
        k=0
        line_color = std_msgs.ColorRGBA()
        for j, datum in enumerate(msg.data):
                #change to datum.worldPoints
            marker = vis_msg.Marker()
            marker.id = j
            marker.header.frame_id = "car_rear_axel"
            marker.type = vis_msg.Marker.LINE_STRIP
            marker.action = marker.ADD
            #marker.ns = "LinesList"
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.0
            marker.color.r = 255.0/255.0
            marker.color.g = (10.0*j)/255.0
            marker.color.b = (5.0*j)/255.0
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            #marker.pose.position.x = 0.0
            #marker.pose.position.y = 0.0
           # marker.pose.position.z = 0.0
            marker.points = []
            for point in datum.worldPoints:
                point.x = point.x
                point.y = point.y
                point.z = point.z
                marker.points.append(point)

            marker_array.markers.append(marker)
        self.pub_vis_lines.publish(marker_array)


    def c_received_obj(self, msg_obj_list):
        self.visualise_detected_objects(msg_obj_list)
        # print("received object")
    def c_received_lane(self, msg_lane_list):
        self.visualise_lines(msg_lane_list)
        # print("received lane")
    def c_received_image(self, msg_image):
        pass # print("received image")
    
    def run_visualization(self):
        rospy.spin()




if __name__ == '__main__':
    try:
        visDw = DwVisualization()
        visDw.run_visualization()
    except rospy.ROSInterruptException:
        pass


