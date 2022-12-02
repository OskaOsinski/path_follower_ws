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


def printmatrix(matrix, width, height):
    for i in range(height):
        msg = []
        for j in range(width):
            msg.append(matrix[i][j])
        print(str().join(msg))

def setpoint(matrix, point, char='X'):
    x, y = point
    matrix[y][x] = char

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

def method(x,y,img):
    if img[x][y] == 0:
        
        return ' '
        
    else:
        return 'X'


def callback(data):
    img = cv2.imread('/home/devpc/path_follower_ws/src/template_gui_package-main/media/dilate.png',0)
    img = img/255
    rows = img.shape[0]
    cols = img.shape[1]
    matrix = [
        [ method(i,j,img) for j in range(cols)]
        for i in range(rows)
    ]
    ilosc = False
    finder = D2(matrix, rows, cols)
    # algorytm a star
    for point in finder.findpath((int(data.x),int(data.y)), (315, 362)):
        # wyrzucanie sciezki do pliku
        xmin = 21.0110884233579
        xmax = 21.0190682401996
        ymin = 52.2679688744638
        ymax = 52.2701910569562

        point_A = (315, 362)
        point_B = point#(int(data.x),int(data.y))
        if point == point_A:
            w = point_A[0]/880.0 * (xmax-xmin) + xmin
            h = (1-point_A[1]/404.0)* (ymax-ymin) + ymin
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
            wB = point_B[0]/880.0 * (xmax-xmin) + xmin
            hB = (1-point_B[1]/404.0)* (ymax-ymin) + ymin
            uB = utm.from_latlon(hB, wB)
            entry = {'x': (uB[0]-u[0]),'y': (uB[1]-u[1])}
            data['waypoints'].append(entry)
    if ilosc: # zapis sciezki
        with open('/home/devpc/path_follower_ws/src/template_gui_package-main/nodes/path/sciezka.json', 'w') as f:
            json.dump(data, f, indent=2)
        f.close()
    else:
        print("wybrany punkt nie lezy na sciezce!!!!!!!!")
   

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
