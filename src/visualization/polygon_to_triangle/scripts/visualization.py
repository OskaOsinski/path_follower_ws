#!/usr/bin/env python

import rospy, rospkg
import time
#from termcolor import colored
import tripy
import sensor_msgs.msg as sensor_msgs
import visualization_msgs.msg as visualization_msg
import geometry_msgs.msg as geometry_msgs
import sensor_msgs.msg as sensor_msgs

def time_it(method):
    """Decorator for functions/methods time efficiency measurement"""
    def timed(*args, **kw):
        time_start = time.time()
        result = method(*args, **kw)
        time_end = time.time()
        time_difference = time_end - time_start
        time_difference_ms = int(1000*time_difference)
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = time_difference_ms
        else:
            #msg = colored('{}\t{} ms'.format(method.__name__, time_difference_ms), 'yellow', attrs=['bold'])
            msg = '{}\t{} ms'.format(method.__name__, time_difference_ms)
            print(msg)
        return result
    return timed

class PolygonToTriagnles:

    def __init__(self):
        rospy.init_node('polygon_to_triangle', anonymous=True)
        # self._rate = rospy.Rate(rospy.get_param('~/polygon_to_triangle/ros_rate'))
        self.init_topic_names()

    def init_topic_names(self):
        rospy.Subscriber("point_list",  geometry_msgs.PolygonStamped, self.convert_to_triangle_list)
        self.trianglesPub = rospy.Publisher('triangle_list', visualization_msg.Marker, queue_size=10)

        rospy.Subscriber("front_lidar_free_space",  sensor_msgs.PointCloud, self.convert_pcl_to_triangle_list)
        self.frontLidarPub = rospy.Publisher('front_lidar_free_space_poly', visualization_msg.Marker, queue_size=10)
    
    @time_it
    def make_triangulation(self, polygon_raw):
        return tripy.earclip(polygon_raw)

    def make_marker(self, triangles, header, color):
        marker = visualization_msg.Marker()
        marker.type = visualization_msg.Marker.TRIANGLE_LIST
        marker.header = header
        marker.ns = ""
        marker.action = visualization_msg.Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = (1.0, 1.0, 1.0)
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color["r"], color["g"], color["b"], color["a"]

        for triangle in triangles:            
            for trianPoint in triangle:    
                new = geometry_msgs.Point()
                new.x, new.y = trianPoint
                new.z = 0           
                marker.points.append( new ) 
        return marker

    def convert_to_triangle_list(self, polygon_msg ):
        polygon_raw = [(point.x, point.y) for index,point in enumerate(polygon_msg.polygon.points) if index%4==0]
        triangles = self.make_triangulation(polygon_raw)
        marker = self.make_marker(triangles, polygon_msg.header, {"r":1.0, "g":1.0, "b":1.0, "a":1.0})
        self.trianglesPub.publish(marker)

    
    def convert_pcl_to_triangle_list(self, pcl_msg ):
        polygon_raw = [(point.x, point.y) for index,point in enumerate(pcl_msg.points) if index%8==0]
        print len(polygon_raw)
        triangles = self.make_triangulation(polygon_raw)
        marker = self.make_marker(triangles, pcl_msg.header, {"r":1.0, "g":1.0, "b":1.0, "a":1.0})          
        self.frontLidarPub.publish(marker)
 
    def run_visualization(self):
        rospy.spin()

if __name__ == '__main__':
    visDw = PolygonToTriagnles()
    visDw.run_visualization()


