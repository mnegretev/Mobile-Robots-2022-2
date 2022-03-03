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

from operator import index
from sqlite3 import Timestamp
import numpy
import heapq
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "SOLANO GONZALEZ FELIPE DE JESUS"

msg_path = Path()

def distancia(pInicial,pFinal):
    return float(math.sqrt((pInicial[0]-pFinal[0])**2 + (pInicial[1]-pFinal[1])**2))

def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # TODO:
    # Write the A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return a set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # indicating the indices (cell coordinates) of the path cells.
    # If path cannot be found, return an empty tuple []
    #

    #Valores de g y f -> Inicializados en infinito
    valores_g = numpy.full(grid_map.shape, float("inf"))
    valores_f = numpy.full(grid_map.shape, float("inf"))
    n_previo  = numpy.full((grid_map.shape[0], grid_map.shape[0], 2), -1)

    #Definicion de nodos
    nodoInicial = [start_r,start_c]
    nodoMeta = [goal_r,goal_c]
    nodosAdyacentes = [[1,0],[0,1],[-1,0],[0,-1],[1,1],[-1,1],[-1,-1],[1,-1]]

    #Definicion de lista abierta y lista cerrada
    openList = []
    closedList = []

    #Valores iniciales
    valores_g[start_r,start_c] = 0
    valores_f[start_r,start_c] = 0
    openList.append(nodoInicial)

    #Se inicializa
    nodoActual = nodoInicial

    while len(openList)>0 and nodoActual != nodoMeta:
        #Se elige el nodo con menor valor de F
        f_values = []
        #Se busca todos los valores de F de la lista abierta
        for i in range(0,len(openList)):
            [x,y] = openList[i]
            f_values.append(valores_f[x,y])
        index_min_f = f_values.index(min(f_values)) 
        #Se quita el de menor valore de F
        nodoActual = openList.pop(index_min_f)
        #Se inserta el nodo con menor valor
        closedList.append(nodoActual)

        #Recorriendo nodos adyacentes
        for i in range(0,len(nodosAdyacentes)):
            #Definiendo nodo actual y vecinos
            [aX,aY] = nodoActual
            [x,y]   = list(numpy.array(nodoActual)+numpy.array(nodosAdyacentes[i]))
            #Variables auxiliares
            if grid_map[x, y] != 0 or [x,y] in closedList:
                continue
            g = valores_g[aX,aY] + distancia([x,y],nodoMeta) + cost_map[x,y]
            h = distancia([x,y],nodoMeta)
            f = g+h
            #Se da parametros a adyancentes que nos los tengan
            if g < valores_g[x,y]:
                valores_g[x,y] = g
                valores_f[x,y] = f
                n_previo [x,y] = [aX,aY]
            
            #Se agrega los nodos adyacentes posibles a la lista abierta
            if (([x,y] not in openList) and ([x,y] not in closedList)):
                openList.append([x,y])

    #Comprobando la existencia de la ruta
    if nodoActual != nodoMeta:
        print("No es posible calcular la ruta")
        return []

    else:
        print("Se calculo la ruta por el algoritmo de A*")
        path = []
        [x,y] = nodoActual
        anterior = True
        while(anterior != False):
            path.insert(0,[x,y])
            [x,y] = list(n_previo[x,y])
            if ([x,y] == nodoInicial):
                path.insert(0,[x,y])
                anterior = False

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
    print ("PRACTICE 02 - " + NAME)
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
    
