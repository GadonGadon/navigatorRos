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
# 시작 : map과 시작, 종료지점의 좌표값을 받음
def astar(map, start, end):
    startNode = Node(None, start)
    endNode = Node(None, end)
	#1. openList에 startNode를 만들어서 사용
    openList = [startNode]
	#endList는 map과 동일하게 width*height크기로 배열 생성(True일경우 해당 노드는 close)
    closeList = [[False for i in range(len(map[0]))]for j in range(len(map))]
	#openList가 모두 빌때까지 반복
    while openList:
		#2. openList내의 노드들 중 f값이 최소인 노드 탐색 후 선택후 openList에서 삭제
        currentNode = openList[0]
        currentIndex = 0
        for index, item in enumerate(openList):
            if item.f < currentNode.f:
                currentIndex = index
                currentNode = item
        openList.pop(currentIndex)
		#3. closeList에 넣음
        closeList[currentNode.position[0]][currentNode.position[1]] = True
		#7. 현재 선택한 노드가 목표노드일 경우 종료
        if currentNode.position == endNode.position:
            print("Find")
            path= [currentNode.position]
			# 부모노드로 거슬러 올라가면서 path저장
            while currentNode.parent:
                currentNode = currentNode.parent
                path.append(currentNode.position)
            return path
            break
		
		#4. 현재 노드를 기준으로 8방위중 이동 가능한 위치에 대한 F, G, H값 계산
        for newPostion in [(0,-1), (0,1), (-1,0), (1,0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nodePostion = (currentNode.position[0] - newPostion[0], currentNode.position[1] - newPostion[1])
            # 좌표가 맵의 영역을 벗어나는것 방지
			if nodePostion[0] < 0 or nodePostion[1] < 0:
                continue
            if nodePostion[0] > len(map) -1 or nodePostion[1] > len(map[0]) -1:
                continue
			# 해당 좌표가 장애물이 있는 노드인지 검사
            if map[nodePostion[0]][nodePostion[1]] != 255:
                continue
			# 해당 좌표가 closeList에 포함되어있는지 검사
            if closeList[nodePostion[0]][nodePostion[1]]:
                continue
            newNode = Node(currentNode, (nodePostion[0],nodePostion[1]))
            # 새로운 노드의 g값 계산(부모노드의 g값 + 10, 대각선일 경우 +14)
			newNode.g = currentNode.g + 10
            if newPostion[0] != 0 and newPostion[1] != 0:
                newNode.g += 4
            # 새로운 노드의 h값 계산(맨하탄 거리)
            newNode.h = heuristic(newNode, endNode)
			# 새로운 노드의 f값 계산(g + h)
            newNode.f = newNode.g + newNode.h
			# 새로운 노드가 이미 openList에 포함되어있는지 검사
            check = False
            nodeIndex = -1
            for index, openNode in enumerate(openList):
                if openNode.position == newNode.position:
					# 새로운 노드의 g값이 이미 있던 노드의 g값보다 큰경우 중지
                    if openNode.g < newNode.g:
                        check = True
                        break
                    else:
                        nodeIndex = index
            if check:
                continue
			# 새로운 노드의 g값이 이미 있던 노드의 g값 보다 작은경우 이미 존재하던 노드 삭제
            if nodeIndex != -1:
                openList.pop(nodeIndex)
			# 새로운 노드를 openList에 넣음
            openList.append(newNode)
	#목표지점까지 path를 찾지 못한경우 None값 반환
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
