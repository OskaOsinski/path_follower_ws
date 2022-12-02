#!/usr/bin/env python
import os, sys
import rospy, rospkg
from pykml import parser as kml_parser
import pymap3d as pm
import ros_communication

from jsk_recognition_msgs.msg import PolygonArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Point, PolygonStamped
from std_msgs.msg import Empty

import kml_parser_publisher.msg as kml_parser_msgs
import kml_parser_publisher.srv as kml_parser_srv

from ros_communication import RosCommunication

class FileParser:

    def __init__(self):
        self._marker_colors = self.enum_colors( \
        AVIABLE=0.5, UNAVIABLE=0.66, BUILDING = 0.92, GRASS=0.77, UN_ROAD=0.66)
        kml_path = rospy.get_param('~/kml_parser_publisher/kml_map_path')
        self._kml_file = os.path.join(kml_path)
        self.reference_point = kml_parser_msgs.coordinates()
        self.reference_point.altitude = rospy.get_param('~/kml_parser_publisher/ref_altitude')
        self.reference_point.longitude = rospy.get_param('~/kml_parser_publisher/ref_longitude')
        self.reference_point.latitude = rospy.get_param('~/kml_parser_publisher/ref_latitude')
    
    def enum_colors(self, **enums):
        return type('Enum', (), enums)

    def parse_file(self):
        with open(self._kml_file) as f:
            doc = kml_parser.parse(f).getroot()
            global msg_pts, msg_lines, msg_poly, point_mark, path_mark, poly_mark
            msg_pts = kml_parser_msgs.kmlPointArray()
            msg_lines = kml_parser_msgs.kmlLineStringArray()
            msg_poly = kml_parser_msgs.kmlPolygonArray()
            point_mark = MarkerArray()
            path_mark = MarkerArray()
            poly_mark = PolygonArray()
            poly_mark.header.frame_id = rospy.get_param('~/kml_parser_publisher/topic_name')

            ref_lat = self.reference_point.latitude
            ref_long = self.reference_point.longitude
            ref_alt = self.reference_point.altitude
            
            for index, placemark in enumerate(doc.iter('Placemark')):
                try:
                    v = placemark.Point
                    msgPoint = kml_parser_msgs.kmlPoint()
                except AttributeError as ae:
                    pass
                else:
                    z = v.coordinates.text
                    list_map = z.split(',')
                    msgPoint.coord.longitude = float(list_map[0])
                    msgPoint.coord.latitude = float(list_map[1])
                    msgPoint.coord.altitude = float(list_map[2])

                    e, n, u = pm.geodetic2enu( \
                        msgPoint.coord.latitude, msgPoint.coord.longitude, msgPoint.coord.altitude, \
                        ref_lat, ref_long, ref_alt \
                    )
                    marker = Marker()
                    marker.type = marker.SPHERE
                    marker.id=len(point_mark.markers)
                    marker.lifetime=rospy.Duration(100)
                    marker.pose.orientation = Quaternion(0, 0, 0, 1)
                    marker.pose.position.x = e
                    marker.pose.position.y = n
                    marker.pose.position.z = 0.0
                    marker.scale.x = rospy.get_param('~/kml_parser_publisher/scale_x')
                    marker.scale.y = rospy.get_param('~/kml_parser_publisher/scale_y')
                    marker.scale.z = rospy.get_param('~/kml_parser_publisher/scale_z')
                    marker.header.frame_id=rospy.get_param('~/kml_parser_publisher/topic_name')
                    marker.color.a = 1.0
                    marker.color.r = 1.0 
                    marker.color.g = 1.0 
                    marker.color.b = 1.0 

                    msgPoint.coord.x = e
                    msgPoint.coord.y = n
                    msgPoint.coord.z = u
                    try:
                        msgPoint.extrude = int(v.extrude)
                    except AttributeError as ae:
                        pass
                    try:
                        msgPoint.description = str(placemark.description.text)
                    except AttributeError as ae:
                        pass
                    try:
                        msgPoint.altitudeMode = str(v.altitudeMode)
                    except AttributeError as ae:
                        pass
                    msg_pts.kmlPoints.append(msgPoint)
                    point_mark.markers.append(marker)
                try:
                    v = placemark.LineString
                    msgLine = kml_parser_msgs.kmlLineString()
                except AttributeError as ae:
                    pass
                else:
                    z = v.coordinates.text
                    list_map = z.split()
                    if(len(list_map) > 100):
                        list_map = list_map[0::3]
                    it = len([e for e in list_map])

                    marker = Marker()
                    #marker.header.stamp = rospy.Time.now()
                    marker.ns = ""
                    marker.action = marker.ADD
                    marker.type = marker.LINE_STRIP
                    marker.id=len(path_mark.markers)
                    marker.scale.x = rospy.get_param('~/kml_parser_publisher/scale_x')
                    marker.header.frame_id= "map" #rospy.get_param('~/kml_parser_publisher/topic_name')
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0

                    for i in range(0, it):
                        local_list = list_map[i].split(',')
                        msgCoord = kml_parser_msgs.coordinates()
                        msgCoord.longitude = float(local_list[0])
                        msgCoord.latitude = float(local_list[1])
                        msgCoord.altitude = float(local_list[2])

                        e, n, u = pm.geodetic2enu( \
                            msgCoord.latitude, msgCoord.longitude, msgCoord.altitude, \
                            ref_lat, ref_long, ref_alt \
                        )
                        new = Point()
                        new.x = e
                        new.y = n
                        new.z = 0
                        marker.points.append(new)

                        msgCoord.x = e
                        msgCoord.y = n
                        msgCoord.z = u
                        msgLine.coord.append(msgCoord)
                        path_mark.markers.append(marker)
                    try: 
                        msgLine.extrude = int(v.extrude)
                    except AttributeError as ae:
                        pass
                    try:
                        msgLine.description = str(placemark.description.text)
                        pol_des = msgLine.description
                        if(pol_des in {"middleRoadLane", "roadLane"}):
                            marker.color.r = 1.0
                            marker.color.g = 1.0
                            marker.color.b = 1.0
                        else:
                            marker.color.r = 1.0
                            marker.color.g = 1.0
                    except AttributeError as ae:
                        pass
                    try:
                        msgLine.altitudeMode = str(v.altitudeMode)
                    except AttributeError as ae:
                        pass
                    try: 
                        msgLine.name = str(placemark.name.text)
                    except AttributeError as ae:
                        pass
                    try:
                        msgLine.tessellate = int(v.tessellate)
                    except AttributeError as ae:
                        pass
                    msg_lines.kmlLineStrings.append(msgLine)
    
                try:
                    v = placemark.Polygon
                    msgPolygon = kml_parser_msgs.kmlPolygon()                
                except AttributeError as ae:
                    pass
                else:
                    z = v.outerBoundaryIs.LinearRing.coordinates.text
                    list_map = z.split()
                    it = len([e for e in list_map])
                    newPolygon = PolygonStamped()
                    newPolygon.header.frame_id=rospy.get_param('~/kml_parser_publisher/topic_name')
                    for i in range(0, it):
                        local_list = list_map[i].split(',')
                        msgCoord = kml_parser_msgs.coordinates()
                        msgCoord.longitude = float(local_list[0])
                        msgCoord.latitude = float(local_list[1])
                        msgCoord.altitude = float(local_list[2])

                        e, n, u = pm.geodetic2enu( \
                            msgCoord.latitude,msgCoord.longitude,msgCoord.altitude, \
                            ref_lat, ref_long, ref_alt \
                        )

                        new = Point()
                        new.x = e
                        new.y = n
                        new.z = 0

                        msgCoord.x = e 
                        msgCoord.y = n
                        msgCoord.z = u
                        msgPolygon.coord.append(msgCoord)
                        newPolygon.polygon.points.append(new)
                    try: 
                        msgPolygon.extrude = int(v.extrude)
                    except AttributeError as ae:
                        pass
                    try:
                        msgPolygon.description = str(placemark.description.text)
                        pol_des = msgPolygon.description
                        like = poly_mark.likelihood
                        if(pol_des in {"availableRoadArea", "availableOther"}): 
                            like.append(self._marker_colors.AVIABLE)
                        elif(pol_des == "building"):
                            like.append(self._marker_colors.BUILDING)
                        elif(pol_des == "grass"):
                            like.append(self._marker_colors.GRASS)
                        elif(pol_des == "unavailableRoadArea"):
                            like.append(self._marker_colors.UN_ROAD)
                        else:
                            like.append(self._marker_colors.UNAVIABLE)
                    except AttributeError as ae:
                        pass
                    try:
                        msgPolygon.altitudeMode = str(v.altitudeMode)
                    except AttributeError as ae:
                        pass
                    try: 
                        msgPolygon.name = str(placemark.name.text)
                    except AttributeError as ae:
                        pass
                    try:
                        msgPolygon.tessellate = int(v.tessellate)
                    except AttributeError as ae:
                        pass
                    msg_poly.kmlPolygons.append(msgPolygon)
                    poly_mark.polygons.append(newPolygon)

class KmlParser:

    def __init__(self, ros_publisher):
        self.ros_publisher = ros_publisher
        self.file_parser = FileParser()
        
    def set_new_reference_point(self, request):
        point = kml_parser_msgs.referencePoint()
        point.longitude = request.longitude
        point.latitude = request.latitude
        point.altitude = request.altitude
        self.file_parser.reference_point = point
        occ_grid_clear.publish()
        self.parse_file_and_publish()
        return 1

    def parse_file_and_publish(self):
        self.file_parser.parse_file()
        self.ros_publisher.pub_to_grid(msg_pts, msg_lines, msg_poly)
        self.ros_publisher.pub_to_vis(point_mark, path_mark, poly_mark)

    def logic_set_new_reference_point(self, request):
        return kml_parser_srv.logic_reference_pointResponse( \
            self.file_parser.reference_point.longitude, \
            self.file_parser.reference_point.latitude, \
            self.file_parser.reference_point.altitude)


if __name__ == '__main__':
    try:
        rospy.init_node('parser', anonymous=True)
        rate_hz = rospy.get_param('~/kml_parser_publisher/ros_rate')
        ros_publisher = RosCommunication(rate_hz)
        ros_publisher.initialize_publisher()
        rate = rospy.Rate(rate_hz)

        parser_object = KmlParser(ros_publisher)
        rospy.Service('kml_new_ref_point', kml_parser_srv.reference_point, \
            parser_object.set_new_reference_point)
        rospy.Service('swiftnav_new_ref_point', kml_parser_srv.logic_reference_point, \
            parser_object.logic_set_new_reference_point)
        occ_grid_clear = rospy.Publisher('kml_occup_grid_new_ref_point', \
            Empty, queue_size=10)
        while not rospy.is_shutdown():
            parser_object.parse_file_and_publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


