#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 3 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from custom_msgs.srv import SmoothPath
from custom_msgs.srv import SmoothPathResponse

NAME = "Lopez Gutierrez"

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    
    P = numpy.copy(Q) 
    tol = 0.00001 
    mag = tol + 1
    epsilon = 0.1

    nabla = numpy.full(Q.shape, float("inf")) #Se define nabla con el mismo tamano de Q y con sus extremos en vallor 0.
    nabla[0], nabla[len(Q)-1] = 0, 0

    while mag > tol:                   # Iteracion hasta llegar a la tolerancia.
	for i in range(1, len(Q)-1):   #La iteracion no considera los extremos de la matriz.                 
            nabla[i] = alpha*(2*P[i]-P[i-1]-P[i+1]) + beta*(P[i]-Q[i]) #Se asigna un valor de gradiente a cada nabla i.
        mag = numpy.linalg.norm(nabla) # Norma del gradiente.
	P = P - epsilon*nabla          # Se recorre P contrario al gradiente.            
    
    return P

def callback_smooth_path(req):
    alpha = rospy.get_param('/path_planning/smoothing_alpha')
    beta  = rospy.get_param('/path_planning/smoothing_beta' )
    P = smooth_path(numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in req.path.poses]), alpha, beta)
    msg_smooth_path.poses = []
    for i in range(len(req.path.poses)):
        msg_smooth_path.poses.append(PoseStamped(pose=Pose(position=Point(x=P[i,0],y=P[i,1]))))
    return SmoothPathResponse(smooth_path=msg_smooth_path)

def main():
    print "PRACTICE 03 - " + NAME
    rospy.init_node("practice03", anonymous=True)
    rospy.Service('/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    pub_path = rospy.Publisher('/path_planning/smooth_path', Path, queue_size=10)
    loop = rospy.Rate(1)
    msg_smooth_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_smooth_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
