#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 2 - PATH PLANNING BY A-STAR
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the A* algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "Villanueva_Aragon_Gabriel"

msg_path = Path()

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write the A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return a set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # indicating the indices (cell coordinates) of the path cells.
    # If path cannot be found, return an empty tuple []
    #
	 	   
    [height, width] = grid_map.shape
    path = []
    OL=[]
    CL=	[]
    g=[]
    f=[]
    p=[];

    # Inicializamos todo en infinito, en este caso sera 100
    for i in range(0,height):
	g.append([])
	f.append([])
	p.append([])
	for j in range(0,width):
		g[i].append(100)
		f[i].append(100)
		p[i].append([0,0])

    ns=[start_r,start_c]
    ng=[goal_r,goal_c]
    n=ns;
    OL.append(ns)
    f[start_r][start_c]=0
    g[start_r][start_c]=0
    while OL and n != ng:
	faux=f[OL[0][0]][OL[0][1]]
	tm=OL[0][0]
	pm=OL[0][1]
	for i in OL:
		if f[i[0]][i[1]]<faux:
			faux=f[i[0],i[1]]
	 		tm=i[0]
			pm=i[1]
	n=[tm,pm]
	CL.append(OL.pop(OL.index(n)))
	for i in range(n[0]-1,n[0]+1):
		for j in range(n[1]-1,n[1]+1):
			if i==n[0] or j==n[1]:#Se ignoran las esquinas
				if cost_map[i,j] < 50: #Se ignoran puntos ocupados
					if i >= 0 and j>=0: #Se ignoran puntos fuera del mapa
						na=[i,j]
						gr=g[n[0]][n[1]]+cost_map[na[0]][na[1]]+1
						hr=abs(ns[0]-na[0])+abs(ns[1]-na[1])
						fr=hr+gr
						if gr < g[na[0]][na[1]]:
							
							g[na[0]][na[1]]=gr
							f[na[0]][na[1]]=fr
							p[na[0]][na[1]]=n
						if OL.count(na) != 0 and CL.count(na)!=0:
							OL.append(na)		
    
    if n != ng:
	return -1
    
   
    while p[n[0]][n[1]] != [0,0]:
	path.insert(0,n)
	n=p[n[0],n[1]]
    
    print(path)
    return path
def get_maps():
    print("Getting inflated and cost maps...")
    clt_static_map = rospy.ServiceProxy("/static_map"  , GetMap)
    clt_cost_map   = rospy.ServiceProxy("/cost_map"    , GetMap)
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        static_map = clt_static_map().map
    except:
        print("Cannot get static map. Terminating program. ")
        exit()
    try:
        inflated_map = clt_inflated().map
        cost_map     = clt_cost_map().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
        print("Using cost map with "     +str(len(cost_map.data))     + " cells.")
    except:
        inflated_map = static_map
        cost_map     = static_map
        print("Cannot get augmented maps. Using static map instead.")
    inflated_map = numpy.reshape(numpy.asarray(inflated_map.data), (static_map.info.height, static_map.info.width))
    cost_map     = numpy.reshape(numpy.asarray(cost_map.data)    , (static_map.info.height, static_map.info.width))
    return [static_map, inflated_map, cost_map]

def callback_a_star(req):
    [s_map, inflated_map, cost_map] = get_maps()
    res = s_map.info.resolution
    [sx, sy] = [req.start.pose.position.x, req.start.pose.position.y]
    [gx, gy] = [req.goal .pose.position.x, req.goal .pose.position.y]
    [zx, zy] = [s_map.info.origin.position.x, s_map.info.origin.position.y]
    print("Calculating path by A* from " + str([sx, sy])+" to "+str([gx, gy]))
    path = a_star(int((sy-zy)/res), int((sx-zx)/res), int((gy-zy)/res), int((gx-zx)/res), inflated_map, cost_map)
    print(path)
    msg_path.poses = []
    for [r,c] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=(c*res + zx), y=(r*res + zy)))))
    return GetPlanResponse(msg_path)

def main():
    print "PRACTICE 02 - " + NAME
    rospy.init_node("practice02")
    rospy.wait_for_service('/static_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_a_star)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
