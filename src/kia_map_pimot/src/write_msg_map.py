#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import bisect
import cv2
import numpy as np
import utm
import json
import rospy
from azsp_msgs.msg import FloatPoint
from gps_msgs.msg import SystemState


class AStarPathfinding(object):
    class Node(object):
        # item must be hashable
        def __init__(self, item, parent=None, gscore=0, heuristic=0):
            self.item = item
            self.parent = parent
            self.gscore = gscore
            self.heuristic = heuristic

        def __eq__(self, other):
            return other.item == self.item

        def __lt__(self, other):
            return self.gscore + self.heuristic < other.gscore + other.heuristic

        def __hash__(self):
            return hash(self.item)

        def __repr__(self):
            return '<%s(%s)>' %(self.__class__.__name__, repr(self.item))

    def heuristic(self, src, des):
        raise NotImplementedError

    def getchildren(self, node):
        raise NotImplementedError

    def available(self, node):
        return True

    def generate(self, node):
        path = list()
        while node:
            path.append(node.item)
            node = node.parent
        return path

    def findpath(self, start, end):
        path = list()
        openpath = dict()
        queue = list()
        closepath = set()
        found = False
        target = self.Node(end)
        node = self.Node(start)
        node.heuristic = self.heuristic(node, target)

        openpath[start] = node
        queue.append(node)
        while openpath and not found:
            current = queue.pop(0)
            openpath.pop(current.item)
            closepath.add(current)
            for node in self.getchildren(current):
                if not self.available(node):
                    continue
                elif node in closepath:
                    continue
                elif node == target:
                    path = self.generate(node)
                    found = True
                    break
                else:
                    duplicated = openpath.get(node.item)
                    if not duplicated:
                        node.heuristic = self.heuristic(node, target)
                        openpath[node.item] = node
                        bisect.insort_left(queue, node)
                    elif duplicated.gscore > node.gscore:
                        left = bisect.bisect_left(queue, duplicated)
                        right = bisect.bisect_right(queue, duplicated)
                        queue.pop(queue.index(duplicated, left, right))
                        node.heuristic = self.heuristic(node, target)
                        openpath[node.item] = node
                        bisect.insort_left(queue, node)
        return path


class D2(AStarPathfinding):
    def __init__(self, matrix, rows, cols):
        self.matrix = matrix
        self.rows = rows
        self.cols = cols

    def heuristic(self, src, des):
        x1, y1 = src.item
        x2, y2 = des.item
        return 1
        #np.sqrt(np.power(x2-x1,2)+np.power(y2-y1,2))
        #abs(x1 - x2) + abs(y1 - y2)

    def getchildren(self, node):
        x, y = node.item
        return [
            self.Node((x + 1, y), parent=node, gscore=node.gscore + 1),
            self.Node((x - 1, y), parent=node, gscore=node.gscore + 1),
            self.Node((x, y + 1), parent=node, gscore=node.gscore + 1),
            self.Node((x, y - 1), parent=node, gscore=node.gscore + 1),
        ]

    def available(self, node):
        x, y = node.item
        return 0 <= x < self.cols and 0 <= y < self.rows and self.matrix[y][x] == ' '




class PathSaver:
    def __init__(self):
        self.car_position = (0,0)
        rospy.init_node('create_path_node')
        setpoint_topic = rospy.get_param('setpoint_topic')
        actual_kia_position_topic = rospy.get_param('actual_kia_position_topic')
        rospy.Subscriber(setpoint_topic, FloatPoint, self.callback)
        rospy.Subscriber(actual_kia_position_topic, SystemState, self.callback_act_pos)

        self.xmin = rospy.get_param('coordinate_system/xmin')
        self.xmax = rospy.get_param('coordinate_system/xmax')
        self.ymin = rospy.get_param('coordinate_system/ymin')
        self.ymax = rospy.get_param('coordinate_system/ymax')
        self.x_img = rospy.get_param('coordinate_system/image_size_x')
        self.y_img = rospy.get_param('coordinate_system/image_size_y')

        rospy.spin()

    @staticmethod
    def printmatrix(matrix, width, height):
        for i in range(height):
            msg = []
            for j in range(width):
                msg.append(matrix[i][j])
            print(str().join(msg))

    @staticmethod
    def setpoint(matrix, point, char='X'):
        x, y = point
        matrix[y][x] = char

    @staticmethod
    def setline(matrix, p1, p2, char='X'):
        if p1 != p2:
            x1, y1 = p1
            x2, y2 = p2
            if x1 == x2:
                for y in range(min(y1, y2), max(y1, y2) + 1):
                    matrix[y][x1] = char
            elif y1 == y2:
                for x in range(min(x1, x2), max(x1, x2) + 1):
                    matrix[y1][x] = char
    @staticmethod
    def method(x,y,img):
        if img[x][y] == 0:
            
            return ' '
            
        else:
            return 'X'

    def callback_act_pos(self,data):

        x = (data.nav_sat_fix.longitude-self.xmin)/(self.xmax-self.xmin)*self.x_img
        y = self.y_img-(data.nav_sat_fix.latitude-self.ymin)/(self.ymax-self.ymin)*self.y_img
        self.car_position = (int(x),int(y))


    def callback(self,data):
        temp_car_position = self.car_position
        img = cv2.imread(rospy.get_param('background_path'),0)
        img = img/255
        rows = img.shape[0]
        cols = img.shape[1]
        matrix = [
            [ self.method(i,j,img) for j in range(cols)]
            for i in range(rows)
        ]
        ilosc = False
        finder = D2(matrix, rows, cols)
        # algorytm a star
        for point in finder.findpath((int(data.x),int(data.y)), (temp_car_position[0], temp_car_position[1])):
            # wyrzucanie sciezki do pliku
            
            point_A = (temp_car_position[0], temp_car_position[1])
            point_B = point#(int(data.x),int(data.y))
            if point == point_A:
                w = point_A[0]/self.x_img * (self.xmax-self.xmin) + self.xmin
                h = (1-point_A[1]/self.y_img)* (self.ymax-self.ymin) + self.ymin
                # print(h,w)
                u = utm.from_latlon(h, w)
                data = {
                    'gps_ref_lat': h, 
                    'gps_ref_lon': w,
                    'utm_ref_x': u[0], 
                    'utm_ref_y': u[1],
                    'waypoints' : [
                    ]
                }
            else:
                ilosc = True
                wB = point_B[0]/self.x_img * (self.xmax-self.xmin) + self.xmin
                hB = (1-point_B[1]/self.y_img)* (self.ymax-self.ymin) + self.ymin
                uB = utm.from_latlon(hB, wB)
                entry = {'x': (uB[0]-u[0]),'y': (uB[1]-u[1])}
                data['waypoints'].append(entry)
        if ilosc: # zapis sciezki
            with open(rospy.get_param('path_follower_path'), 'w') as f:
                
                json.dump(data, f, indent=2)
            f.close()
            rospy.logwarn('utworzono sciezke')
        else:
            rospy.logwarn('jeden z punktow nie lezy na sciezce!')


if __name__ == '__main__':
    try:
        PathSaver()
    except rospy.ROSInterruptException:
        pass