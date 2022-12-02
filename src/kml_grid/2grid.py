#!/usr/bin/env python
import pymap3d as pm
import numpy as np
import sys
import rospy
import kml_parser_publisher.msg as kml_pars_msgs
from skimage.draw import polygon, line
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from kml_parser_publisher.srv import *

import kml_parser_publisher.srv as kml_parser_srv
import kml_grid.srv as kml_grid_srv


class GridDraw:

    def __init__(self):
        self._map_width = rospy.get_param('~/kml_grid/map_width')
        self._map_height = rospy.get_param('~/kml_grid/map_height')
        self.set_map()


    def set_map(self):
        self._grid = OccupancyGrid()
        self._grid.header.frame_id= rospy.get_param('~/kml_grid/topic_name')
        self._grid.info.resolution = rospy.get_param('~/kml_grid/map_resolution')
        self._grid.info.width = self._map_width
        self._grid.info.height = self._map_height
        self._grid.info.origin.position.x = \
            0 - self._grid.info.resolution*self._grid.info.width/2
        self._grid.info.origin.position.y = \
            0 - self._grid.info.resolution*self._grid.info.height/2
        self._grid.info.origin.position.z = 0
        self.occ_grid_clear('clear')

    def occ_grid_clear(self, cb):
        self._grid.data = [ \
            0 for i in range(self._grid.info.width * self._grid.info.height)]

    def c_points(self, data):
        self.process_points(data.kmlPoints,"point")

    def c_lines(self, data):
        self.process_points(data.kmlLineStrings,"line")

    def c_polygons(self, data):
        self.process_points(data.kmlPolygons, "polygon")

    def process_points(self, data, name):
        it = len([e for e in data])
      #  filteredPoints = kml_pars_msgs.coordinatesArray()
      #  for i in range(0, it):
      #      if(name == "point"):
      #          filteredPoints.enuCoordinates.append(data[i].coord)
      #          r,c = self.calc_index(filteredPoints)
      #          index = int(c.item(i)*self._map_width)+r.item(i)
      #          self.draw_thickened_grid_point(index)
      #          del filteredPoints.enuCoordinates[:]
      #          return
      #      else:
      #          moreit = len([e for e in data[i].coord])
      #          for j in range(0, moreit):
      #              filteredPoints.enuCoordinates.append \
      #                  (data[i].coord[j])
#
#            r,c = self.calc_index(filteredPoints)
#            if(name=="polygon"):
#                rr, cc = polygon(-r, -c)
#                for i in range(0, rr.size):
#                    index = int(-cc.item(i)*self._map_width)-rr.item(i)
#                    self._grid.data[index] = 100
#            else:
#                for i in range(1, r.size):
#                    rr, cc = line(-r[i-1], -c[i-1], -r[i], -c[i])
#                    for i in range(0, rr.size):
#                        index = int(-cc.item(i)*self._map_width)-rr.item(i)
#                        self.draw_thickened_grid_point(index)
#
#            del filteredPoints.enuCoordinates[:]

    def calc_index(self, points):
        it = len(points.enuCoordinates)
        r = []
        c = []
        for i in range(0,it):
            self.e = points.enuCoordinates[i].x/self._grid.info.resolution
            self.n = points.enuCoordinates[i].y/self._grid.info.resolution
            new_e = self.e - self._map_width/2
            new_n = self.n - self._map_height/2
            c.append((int(new_n) - 1))
            r.append(int(new_e))
        return np.asarray(r), np.asarray(c)


    def draw_thickened_grid_point(self, index):
        indexes_arround = []
        indexes_arround.append(index + 1)
        indexes_arround.append(index - 1)
        indexes_arround.append(index + self._map_width)
        indexes_arround.append(index - self._map_width)
        indexes_arround.append(index + self._map_width+1)
        indexes_arround.append(index + self._map_width-1)
        indexes_arround.append(index - self._map_width+1)
        indexes_arround.append(index - self._map_width-1)

        self._grid.data[int(index)] = 100
        for ind in indexes_arround:
            if(self._grid.data[int(ind)] == 0):
                self._grid.data[int(ind)] = 30


    def service_deliver_grid(self, request):
        return kml_grid_srv.occupancy_gridResponse(self._grid)

    def publish_result_grid(self):
        publish_grid.publish(self._grid)

if __name__ == '__main__':
    rospy.init_node('kml_grid', anonymous=True)

    grid_obj = GridDraw()

    rospy.Subscriber("kml_occup_grid_new_ref_point", Empty, grid_obj.occ_grid_clear)
    rospy.Subscriber("kml_points_to_grid",   kml_pars_msgs.kmlPointArray, grid_obj.c_points)
    rospy.Subscriber("kml_lines_to_grid",    kml_pars_msgs.kmlLineStringArray, grid_obj.c_lines)
    rospy.Subscriber("kml_polygons_to_grid", kml_pars_msgs.kmlPolygonArray, grid_obj.c_polygons)

    rospy.Service('occupancy_grid', kml_grid_srv.occupancy_grid, grid_obj.service_deliver_grid)

    publish_grid = rospy.Publisher( 'map', OccupancyGrid, queue_size=1000)

    rate = rospy.Rate(rospy.get_param('~/kml_grid/ros_rate'))
    while not rospy.is_shutdown():
        grid_obj.publish_result_grid()
        rate.sleep()

