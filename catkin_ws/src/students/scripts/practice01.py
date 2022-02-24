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

NAME = "DIANA PAOLA SANJUAN ALDAPE"

def get_inflated_map(static_map, inflation_cells): 
    print("Inflating map by " + str(inflation_cells) + " cells") 
    #inflation_cells = numero de celdas que se va a aumentar (radio de inflado)
    inflated = numpy.copy(static_map) #Copia el mapa
    [height, width] = static_map.shape #Height = renglones, width = columnas de static map
    #
    # TODO:
    # Write the code necessary to inflate the obstacles in the map a radius
    # given by 'inflation_cells' (expressed in number of cells)
    # Map is given in 'static_map' as a bidimensional numpy array.
    # Consider as occupied cells all cells with an occupation value greater than 50
    #
    for i in range (0, height): #Recorre renglones
    	for j in range (0, width): #Recorre columnas
    	   if (static_map[i,j] == 100 ): #Verifica si esta ocupada
	      #Recorre desde el -radio de inflado hasta el radio de inflado
    	      for k1 in range (-inflation_cells, inflation_cells+1): 
    		for k2 in range (-inflation_cells, inflation_cells+1):
    		   inflated[i+k1,j+k2]=100 #Se iguala a 100 en cuanto se ocupe
    return inflated

def get_cost_map(static_map, cost_radius):
    if cost_radius > 20:
        cost_radius = 20
    print "Calculating cost map with " +str(cost_radius) + " cells"
    cost_map = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # TODO:
    # Write the code necessary to calculate a cost map for the given map.
    # To calculate cost, consider as example the following map:    
    # [[ 0 0 0 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 0 0 X 0 0]]
    # Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
    # Cost is an integer indicating how near cells and obstacles are:
    # [[ 3 3 3 2 2 1]
    #  [ 3 X 3 3 2 1]
    #  [ 3 X X 3 2 1]
    #  [ 3 X X 3 2 2]
    #  [ 3 X 3 3 3 2]
    #  [ 3 3 3 X 3 2]]
    # Cost_radius indicate the number of cells around obstacles with costs greater than zero.
    for i in range (0, height): #Recorre renglones
     	for j in range (0, width): #Recorre columnas
     	   if (static_map[i,j] == 100 ): #Verifica si esta ocupada
	      #Recorre desde el -radio de costo hasta el +radio de costo
     	      for k1 in range (-cost_radius, cost_radius+1):
    		for k2 in range (-cost_radius, cost_radius+1):
		   c = cost_radius - max(abs(k1),abs(k2))
		   #El mapa de costo se iguala al maximo entre el costo calculado (c) y el mapa de costo que ya se tenia. 
		   cost_map[i+k1, j+k2] = max(c, cost_map[i+k1,j+k2])
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
    
