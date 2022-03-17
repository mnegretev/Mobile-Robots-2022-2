#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 4 - PATH FOLLOWING
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import rospy
import tf
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

pub_goal_reached = None
pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # TODO:
    # Implement the control law given by:

    error_a=(math.atan2(goal_y-robot_y,goal_x-robot_x)-robot_a+math.pi)%(2*math.pi)-math.pi
    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    
    cmd_vel.linear.x=v
    cmd_vel.linear.angular.z=w

    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:
    #
    # Set local goal point as the first point of the path
    
    local_goal_x,local_goal_y=path[0]
    # Set global goal point as the last point of the path
    global_goal_x,global_goal_y=path[-1]
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    global_error=math.sqrt((global_goal_x-robot_x)**2+(global_goal_y-robot_y)**2)
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    local_error=math.sqrt((local_goal_x-robot_x)**2+(local_goal_y-robot_y)**2)
    #
    i=0
    while global_error > 0.2 and not rospy.is_shutdown(): #This keeps the program aware of signals such as Ctrl+C
    #     Calculate control signals v and w and publish the corresponding message
	pub_goal_publish(calculate_control(robot_x, robot_y, robot_a, local_goal_x, local_goal_y))
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
        loop.sleep()
    #     Get robot position
	[robot_x, robot_y, robot_a] = get_robot_pose(listener)
    #     Calculate local error
	local_error=math.sqrt((local_goal_x-robot_x)**2+(local_goal_y-robot_y)**2)
    #     If local error is less than 0.3 (you can change this constant)
	if local_error<0.3:
    #         Change local goal point to the next point in the path
		local_goal_x,local_goal_y=path[i+1]
    #     Calculate global error
	global_error=math.sqrt((global_goal_x-robot_x)**2+(global_goal_y-robot_y)**2)
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    cmd_vel = Twist()
    cmd_vel.linear.x=0
    cmd_vel.linear.angular.z=0
    pub_goal_publish(cmd_vel)
    # Publish a 'True' using the pub_goal_reached publisher
    #
    pub_goal_reached.publish(True)
    return
    
def callback_global_goal(msg):
    print "Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y])
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print "Following path with " + str(len(path.poses)) + " points..."
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print "Global goal point reached"

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, pub_goal_reached, loop, listener
    print "PRACTICE 04 - " + NAME
    rospy.init_node("practice04")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_goal_reached = rospy.Publisher('/navigation/goal_reached', Bool, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
