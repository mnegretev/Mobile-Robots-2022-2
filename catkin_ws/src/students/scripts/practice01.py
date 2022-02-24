#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 1 - INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get an inflated map and a cost map given
# an inflation radius and a cost radius.
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "Carrillo Salazar Jose Armando"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    #variables
    ri=inflation_cells		# ri es el radio de inflación
    for i in range(0,height,1): # Se recorrer la matriz
	for j in range(0,width,1):
		if inflated[i,j]>50: #si el valor dentro de la matriz es mayor a 50 se considera lugar ocupado
			for k1 in range(-ri,ri,1): #Se recorre alredor de ese lugar e inflar el mapa	
				for k2 in range(-ri,ri,1):
					c=ri-max(abs(k1),abs(k2))
					inflated[i+k1,j+k2]=50 #con un valor de 50 se ve en gris la parte afectada
    return inflated

def get_cost_map(static_map, cost_radius):
    if cost_radius > 20:
        cost_radius = 20
    print "Calculating cost map with " +str(cost_radius) + " cells"
    cost_map = numpy.copy(static_map)
    [height, width] = static_map.shape
    
    for i in range(0,height,1): # Se recorre la matriz
	for j in range(0,width,1):
		if static_map[i,j]>49: #si el valor es mayor a 49 se asegura que pertenece al mapa inflado	
			for x in range(i-cost_radius,i+cost_radius,1): #Se recorre alredor de celda ocupada o inflada
				for y in range(j-cost_radius,j+cost_radius,1):
					cost=cost_radius-max(abs(x-i),abs(y-j)) # Se calcula el costo de 0 a 9
					cost_map[x,y]=max(cost,cost_map[x,y] #Usamos el costo mas alto entre  anterior y  actual
    return cost_map

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def callback_cost_map(req):
    global cost_map
    return GetMapResponse(map=cost_map)
    
def main():
    global cost_map, inflated_map
    print "PRACTICE 01 - " + NAME
    rospy.init_node("practice01")
    rospy.wait_for_service('/static_map')
    pub_map  = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    rospy.Service('/cost_map'    , GetMap, callback_cost_map)
    loop = rospy.Rate(1)
    
    cost_radius      = 0.1
    inflation_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/cost_radius"):
            new_cost_radius = rospy.get_param("/path_planning/cost_radius")
        if rospy.has_param("/path_planning/inflation_radius"):
            new_inflation_radius = rospy.get_param("/path_planning/inflation_radius")
        if new_cost_radius != cost_radius:
            cost_radius   = new_cost_radius
            cost_map_data = get_cost_map(grid_map, int(cost_radius/res))
            cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (width*height, 1)))
            cost_map      = OccupancyGrid(info=map_info, data=cost_map_data)    
        if new_inflation_radius != inflation_radius:
            inflation_radius  = new_inflation_radius
            inflated_map_data = get_inflated_map(grid_map, int(inflation_radius/res))
            inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (width*height, 1)))
            inflated_map      = OccupancyGrid(info=map_info, data=inflated_map_data)
            pub_map.publish(callback_inflated_map(GetMapRequest()).map)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
