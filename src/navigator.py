#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, pi, sin, cos
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from copy import deepcopy
import tf
import cv2
import numpy as np

class Node:
    def __init__(self, parent = None, position = None):
        self.parent = parent
        self.position = position
        self.f = 0
        self.g = 0
        self.h = 0
def heuristic(node, goal):
    h = abs(node.position[0] - goal.position[0]) + abs(node.position[1] - goal.position[1])
    return h
def astar(map, start, end):
    startNode = Node(None, start)
    endNode = Node(None, end)

    openList = [startNode]
    closeList = [[False for i in range(len(map[0]))]for j in range(len(map))]

    while openList:
        currentNode = openList[0]
        currentIndex = 0
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentIndex = index
                currentNode = item
        
        openList.pop(currentIndex)
        closeList[currentNode.position[0]][currentNode.position[1]] = True
        if currentNode.position == endNode.position:
            print("Find")
            path= [currentNode.position]
            while currentNode.parent:
                currentNode = currentNode.parent
                path.append(currentNode.position)
            return path
            break
        for newPostion in [(0,-1), (0,1), (-1,0), (1,0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nodePostion = (currentNode.position[0] - newPostion[0], currentNode.position[1] - newPostion[1])
            if nodePostion[0] < 0 or nodePostion[1] < 0:
                continue
            if nodePostion[0] > len(map) -1 or nodePostion[1] > len(map[0]) -1:
                continue
            if map[nodePostion[0]][nodePostion[1]] != 255:
                continue
            if closeList[nodePostion[0]][nodePostion[1]]:
                continue
            newNode = Node(currentNode, (nodePostion[0],nodePostion[1]))
            newNode.g = currentNode.g + 10
            if newPostion[0] != 0 and newPostion[1] != 0:
                newNode.g += 4
            
            newNode.h = heuristic(newNode, endNode)
            newNode.f = newNode.g + newNode.h
            check = False
            nodeIndex = -1
            for index, openNode in enumerate(openList):
                if openNode.position == newNode.position:
                    if openNode.g < newNode.g:
                        check = True
                        break
                    else:
                        nodeIndex = index
            if check:
                continue
            if nodeIndex != -1:
                openList.pop(nodeIndex)
            openList.append(newNode)
    return None
def odom_cb(msg):
    current_pose = msg
    #print(current_pose.pose.pose.position)
def map_cb(msg):
    height,width = msg.info.height, msg.info.width
    Map = np.zeros((height, width), np.uint8)
    for i in range(height):
        for j in range(width):
            OccProb = msg.data[i*width + j]
            if OccProb<0:   OccProb =0
            elif OccProb== 100: OccProb = 0
            else:   OccProb = 255
            Map[i][j] = OccProb
    start = [0,-1]
    goal = [0,2]
    startPosition = (height//2 - (20*start[0]) ,  width//2 - (20* start[1]))
    endPosition = (height//2 - (20*goal[0]), width//2 - (20*goal[1]))
    Map1 = cv2.GaussianBlur(Map, (0,0), 1)
    cv2.imshow("map", Map)
    cv2.imshow("map1", Map1)
    path = astar(Map1, startPosition, endPosition)


    dst = cv2.cvtColor(Map, cv2.COLOR_GRAY2RGB)
    
    for x,y in path:
        dst = cv2.line(dst, (y,x),(y,x), (0,255,0), 1)
    dst = cv2.line(dst, (endPosition[1],endPosition[0]),(endPosition[1],endPosition[0]),(0,0,255),0)
    dst = cv2.line(dst, (startPosition[1],startPosition[0]),(startPosition[1],startPosition[0]),(255,0,0),0)
    dst = cv2.resize(dst, dsize=(0,0), fx=5,fy=5, interpolation=cv2.INTER_AREA)
    
    cv2.imshow('Astar', dst)
    cv2.waitKey()

    
if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    current_pose = Odometry()
    odom_sub = rospy.Subscriber('/odom', 
                            Odometry, 
                            odom_cb)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_cb)

    
    rospy.spin()