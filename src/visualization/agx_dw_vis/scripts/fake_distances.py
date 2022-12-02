#!/usr/bin/env python

import rospy, rospkg
import time
import driveworks_msgs.msg as driveworks_msgs
import sensor_msgs.msg as sensor_msgs
import matplotlib.image as img
from random import randrange

class DwFakeObjDetection:

    def __init__(self):
        rospy.init_node('dw_fake_objects_detection', anonymous=True)
        self.init_topic_names_publisher()
        self.FREQUENCY = 10
        self.NUM_OF_OBJECTS = 0

    def init_topic_names_publisher(self):
        topic_name = "dw/camera_front_center_60fov/objects"
        dwTopicNames = rospy.get_param('~/agx_dw_vis/dw_topic_names')
        self.pub_topic_names = rospy.Publisher(dwTopicNames, driveworks_msgs.DatectionTopicsNames, queue_size=0)
        
        #self.names_list = driveworks_msgs.DatectionTopicsNames()
        self.names_list = []
        self.names_list.append(topic_name)

        self.pub_objects_distances = rospy.Publisher("/prev_obj_dist", driveworks_msgs.EstimatedDistances, queue_size=0)
        self.fake_objects_list = driveworks_msgs.DetectedObjectList()
        
        rospy.Subscriber(topic_name,  driveworks_msgs.DetectedObjectList, self.detections_callback)
           
    def detections_callback(self, msg):
        self.NUM_OF_OBJECTS = len(msg.data)


    def create_fake_distance(self, seed):
        return 5 + seed*4 + randrange(70)/1000.0

    def create_fake_distances(self):
        self.fake_distnces = driveworks_msgs.EstimatedDistances()
        [self.fake_distnces.data.append(self.create_fake_distance(seed)) for seed in range(self.NUM_OF_OBJECTS)]

    def create_fake_object(self, seed = 1):
        detected_object = driveworks_msgs.DetectedObject()
        detected_object.versorUpperLeft.x = 2.0 + seed*2
        detected_object.versorUpperLeft.y = 2.0 + seed
        detected_object.versorUpperLeft.z = 0.0

        detected_object.versorUpperRight.x = 1.0 + seed*2
        detected_object.versorUpperRight.y = 1.0 + seed
        detected_object.versorUpperRight.z = 0.0

        detected_object.versorBottomLeft.x = 2.0 + seed*2
        detected_object.versorBottomLeft.y = 2.0 + seed
        detected_object.versorBottomLeft.z = 0.0

        detected_object.versorBottomRight.x = 1.0 + seed*2
        detected_object.versorBottomRight.y = 1.0 + seed
        detected_object.versorBottomRight.z = 0.0
        
        detected_object.type = 1
        detected_object.subtype = seed
        return detected_object

    def fake_objects_detection(self):
        del self.fake_objects_list.data[:]
        
        [self.fake_objects_list.data.append(self.create_fake_object(seed)) for seed in range(self.NUM_OF_OBJECTS)]
        
    def publish_fake_distances(self):
        #print("fake_distnces", type(self.fake_distnces.data), self.fake_distnces.data)
        self.pub_objects_distances.publish(self.fake_distnces)
    
    def publish_topic_names(self):
        self.pub_topic_names.publish(self.names_list)


    def run(self):
        #r = rospy.Rate(self.FREQUENCY)
        while not rospy.is_shutdown():
            self.fake_objects_detection()
            self.create_fake_distances()
            
            self.publish_fake_distances()
            self.publish_topic_names()
            #r.sleep()
            time.sleep(1/self.FREQUENCY)

if __name__ == '__main__':
    try:
        visDw = DwFakeObjDetection()
        visDw.run()
    except rospy.ROSInterruptException:
        pass


