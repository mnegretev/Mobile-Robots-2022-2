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

NAME = "Moctezuma Contreras"

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
    ns = [start_r,start_c]
    ng = [goal_r,goal_c]
    dimensiones = grid_map.shape
    infinito = float('inf')
    fn = np.ones(dimensiones)*infinito
    gn = np.ones(dimensiones)*infinito
    pn_row = np.zeros(dimension) #pn_row y pn_col para guardar coordenas de nodo previo
    pn_col = np.zeros(dimension) #
    OL = []
    CL = []
    OL.insert(0,ns)
    fn[ns[0],ns[1]] = 0
    gn[ns[0],ns[1]] = 0
    nc=ns #Nodo actual
    OLfn = [] #Lista auxiliar que contendra los valores fn de los elementos de OL
    OLgn = [] #Lista auxiliar que contendra los valores gn de los elementos de OL
    while OL and nc != ng:    
        for i in range(len(OL)):
            auxind = OL[i] #auxind contiene el nodo iesimo de OL 
            OLfn.append(fn[auxind[0],auxind[1]])  
            OLgn.append(gn[auxind[0],auxind[1]])
        indmin = np.argmin(OLfn) #indice del valor minimo fn de los elementos de OL
        #print('Indice del Minimo de OL es: {0}'.format(indmin))
        #print('Ol es: \n {0}'.format(OL))
        nc = OL.pop(indmin) #El nuevo nodo sera el del valor minimo y se saca de OL
        valormin=OLfn.pop(indmin) #Se saca el valor fn del nuevo nodo
        CL.append(nc) #Se agregar nc a la lista cerrada
        na_aux = [[-1,0],[1,0],[0,-1],[0,1]]#Nodos vecinos de nc conectividad 4
        for i in range(len(na_aux)):
            auxind_na = na_aux[i] #auxind_na contiene el nodo vecino iesimo
            na_row = nc[0]+auxind_na[0]
            na_col = nc[1]+auxind_na[1]
            na = [na_row,na_col]
            g = gn[nc[0],nc[1]]+1 #Mas el costo
            h = abs(na_row-ng[0])+abs(na_col-ng[1])
            f= g + h
#--------------------
        #--
        if g < gn[na_row,na_col] :
            gn[na_row,na_col]= g
            fn[na_row,na_col]= f
            pn_row[na_row,na_col] = nc[0]
            pn_col[na_row,na_col] = nc[1]
        if (na not in OL) and (na not in CL):
            OL.append(na)
#---------------------------------            
    if nc != ng:
        print('-----------No existe una solucion-----------')
    else:
        path = []
        while nc != ns:
            path.insert(0,nc)
            nc = [pn_row[nc[0],nc[1]],pn_col[nc[0],nc[1]]]
        path.insert(0,nc)
        
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
    
