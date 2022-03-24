#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 5 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants alpha and beta to get a smooth movement. 
#

from numpy import angle
import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "SOLANO GONZALEZ FELIPE DE JESUS"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # TODO:
    # Implement the control law given by:
    
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.

    v_max = 0.4
    w_max = 0.5
    alpha = 0.1
    beta  = 0.1

    # calculing angle error
    # global error - robot angle
    error_a = (math.atan2(goal_y-robot_y,goal_x-robot_x)) - robot_a

    #Adjust angle error
    #Domain [-pi,pi]

    if error_a > math.pi:
        error_a = error_a - 2*math.pi
    elif error_a < (-math.pi):
        error_a = error_a + 2*math.pi

    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    # Store the resulting v and w in the Twist message cmd_vel
    # # and return it (check online documentation for the Twist message).
    cmd_vel.linear.x  = v
    cmd_vel.angular.z = w
    # Remember to keep error angle in the interval (-pi,pi]
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given the robot and goal positions.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    #

    #Constante
    alfa = 1

    #Calculando la magnitud 
    magnitud = math.sqrt((robot_x-goal_x)**2+(robot_y-goal_y)**2)

    #Calculando fuerza
    force_x = alfa * (robot_x - goal_x) / magnitud
    force_y = alfa * (robot_y - goal_y) / magnitud
    
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # TODO:
    # Calculate the total rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # both measured w.r.t. robot's frame.
    # See lecture notes for equations to calculate rejection forces.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force w.r.t. map.
    #

    #Defining variables
    [force_x, force_y] = [0, 0]
    mag = 0
    beta = 4
    d0 = 1

    #Looking for the obstacles detected
    for [distance,angle] in laser_readings:
        if distance < d0:
            mag = beta * math.sqrt(1/distance - 1/d0)
        else:
            mag = 0
        force_x += mag * math.cos( angle + robot_a )
        force_y += mag * math.sin( angle + robot_a ) 

    n = len(laser_readings)
    [force_x,force_y] = [ force_x/n , force_y/n ]

    return [force_x, force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print ("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields")
    loop = rospy.Rate(20)
    global laser_readings

    #
    # TODO:
    # Move the robot towards goal point using potential fields.
    # Remember goal point is a local minimun in the potential field, thus,
    # it can be reached by the gradient descend algorithm.
    # Sum of attraction and rejection forces is the gradient of the potential field,
    # then, you can reach the goal point with the following pseudocode:
    #
    # Set constant epsilon (0.5 is a good start)
    # Set tolerance  (0.1 is a good start)
    epsilon = 0.5
    tol = 0.1
    # Get robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate distance to goal as math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    d_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    # WHILE distance_to_goal_point > tolerance and not rospy.is_shutdown():
    #     Calculate attraction force Fa by calling [fax,fay]=attraction_force(robot_x,robot_y,goal_x, goal_y)
    #     Calculate rejection  force Fr by calling [frx,fry]=rejection_force (robot_x,robot_y,robot_a,laser_readings)
    #     Calculate resulting  force F = Fa + Fr
    #     Calculate next local goal point P = [px, py] = Pr - epsilon*F
    #
    #     Calculate control signals by calling msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    #     Send the control signals to mobile base by calling pub_cmd_vel.publish(msg_cmd_vel)
    #     Call draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)  to draw all forces
    #
    #     Wait a little bit of time by calling loop.sleep()
    #     Update robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    #     Recalculate distance to goal position
    while d_to_goal > tol and not rospy.is_shutdown():
        #atracction force 
        [fax,fay]=attraction_force(robot_x,robot_y,goal_x, goal_y)
        #rejection force
        [frx,fry]=rejection_force (robot_x,robot_y,robot_a,laser_readings)
        #resulting force
        [fresx,fresy] = [ fax + frx , fay + fry ]
        #P = [px, py] = Pr - epsilon*F
        [px , py] = [ robot_x - epsilon*fresx , robot_y - epsilon*fresy ]

        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fresx, fresy, pub_markers)
        
        loop.sleep()

        robot_x, robot_y, robot_a = get_robot_pose(listener)
        d_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

    #  Publish a zero speed (to stop robot after reaching goal point)
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    pub_cmd_vel.publish(stop_msg)
    print("Goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    hdr = Header(frame_id="map", stamp=rospy.Time.now())
    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=robot_x, y=robot_y))
    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print ("PRACTICE 05 - " + NAME)
    rospy.init_node("practice05")
    rospy.Subscriber("/scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
