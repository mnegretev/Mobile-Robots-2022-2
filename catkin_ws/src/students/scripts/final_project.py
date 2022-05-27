#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instructions:
# Write the code necessary to make the robot to perform the following possible commands:
# * Robot take the <pringles|drink> to the <table|kitchen>
# You can choose where the table and kitchen are located within the map.
# Pringles and drink are the two objects on the table used in practice 07.
# The Robot must recognize the orders using speech recognition.
# Entering the command by text or similar way is not allowed.
# The Robot must announce the progress of the action using speech synthesis,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.
# Publishers and suscribers to interact with the subsystems (navigation,
# vision, manipulation, speech synthesis and recognition) are already declared. 
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from custom_msgs.srv import *

NAME = "SOLANO GONZALEZ FELIPE DE JESUS"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.data

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [0,0] if "TABLE" in cmd else [7, 0]
    return obj, loc

#
# This function sends the goal articular position to the left arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the left gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal articular position to the right arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the right gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal pan-tilt angles to the head and sleeps 1 second
# to allow the head to reach the goal position. 
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float32MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# This function sends a linear and angular speed to the mobile base to perform
# low-level movements. The mobile base will move at the given linear-angular speeds
# during a time given by 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

#
# This function publishes a global goal position. This topic is subscribed by
# pratice04 and performs path planning and tracking.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# This function sends a text to be synthetized.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

def transform_point_to_left_arm(x,y,z):
    listener = tf.TransformListener()
    listener.waitForTransform("shoulders_left_link", "kinect_link", rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = "kinect_link"
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    target_frame = "shoulders_left_link"
    obj_p = listener.transformPoint(target_frame, obj_p)
    return obj_p.point.x, obj_p.point.y, obj_p.point.z

def transform_point_to_right_arm(x,y,z):
    listener = tf.TransformListener()
    listener.waitForTransform("shoulders_left_link", "kinect_link", rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = "kinect_link"
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    target_frame = "shoulders_right_link"
    obj_p = listener.transformPoint(target_frame, obj_p)
    return obj_p.point.x, obj_p.point.y, obj_p.point.z

def find_object(obj_name):
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)
    req_find_object = FindObjectRequest()
    req_find_object.cloud = rospy.wait_for_message("/kinect/points", PointCloud2)
    req_find_object.name  = obj_name
    resp_find_object = clt_find_object(req_find_object)
    print("Object found at: " + str([resp_find_object.x, resp_find_object.y, resp_find_object.z]))
    return resp_find_object.x, resp_find_object.y, resp_find_object.z

def ik_left_arm(x,y,z):
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    req_ik = InverseKinematicsRequest()
    req_ik.x, req_ik.y, req_ik.z = x,y,z
    req_ik.roll, req_ik.pitch, req_ik.yaw = 3.0, -1.57, -3.0
    resp_ik = clt_la_inverse_kin(req_ik)
    return resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7

def ik_right_arm(x,y,z):
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    req_ik = InverseKinematicsRequest()
    req_ik.x, req_ik.y, req_ik.z = x,y,z
    req_ik.roll, req_ik.pitch, req_ik.yaw = 3.0, -1.57, -3.0
    resp_ik = clt_ra_inverse_kin(req_ik)
    return resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_exercise")
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float32MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float32MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float32MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float32, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float32, queue_size=10);
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available.")
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]
    #Kinect position
    obj_kinect_x = 0
    obj_kinect_y = 0
    obj_kinect_z = 0
    #Object real position
    obj_real_x = 0
    obj_real_y = 0
    obj_real_z = 0
    #Cinematica inversa
    q = 0

    #
    # FINAL PROJECT
    # 
    #
    
    while not rospy.is_shutdown():
        
        #Primer estado
        if current_state == "SM_INIT":
            print("Iniciando maquina de estados...")
            current_state = "SM_NEW_TASK"
        
        #Segundo estado
        elif current_state == "SM_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("Nuevo comando: " + requested_object + " to  " + str(requested_location))
                print("Comando: " + recognized_speech)
                say(recognized_speech)
                current_state = "SM_MHEAD"
                new_task = False
                executing_task = True

        #Tercer estado
        elif current_state == "SM_MHEAD":
            print("Ajustando posicion de la cabeza...")
            move_head(0, -0.9)
            current_state = "SM_FIND_OBJECT"

        #Cuarto estado
        elif current_state == "SM_FIND_OBJECT":
            print("Tratando de encontrar objeto: " + requested_object)
            obj_kinect_x , obj_kinect_y, obj_kinect_z = find_object(requested_object)
            current_state = "SM_KINEMATICS"

        #Quinto estado
        elif current_state == "SM_KINEMATICS":
            #Para las pringles 
            if requested_object == "pringles":
                #Obteniendo posicion real
                obj_real_x,obj_real_y,obj_real_z = transform_point_to_left_arm(obj_kinect_x,obj_kinect_y,obj_kinect_z)
                print("Posicion de objeto desde la referencia del brazo izquierdo:")
                print(str([obj_real_x,obj_real_y,obj_real_z]))
                print("Valores de articulaciones con cinematica inversa: ")
                q = ik_left_arm(obj_real_x + 0.1 ,obj_real_y,obj_real_z) #Ajuste
                #Mover brazo izquierdo
                current_state = "SM_MLA"
            #Para drink
            else:
                obj_real_x,obj_real_y,obj_real_z = transform_point_to_right_arm(obj_kinect_x,obj_kinect_y,obj_kinect_z)
                print("Posicion de objeto desde la referencia del brazo derecho:")
                print(str([obj_real_x,obj_real_y,obj_real_z]))
                print("Valores de articulaciones con cinematica inversa: ")
                q = ik_right_arm(obj_real_x + 0.1 ,obj_real_y,obj_real_z) #Ajuste
                #Mover brazo derecho
                current_state = "SM_MRA" 
        
        #Mover brazo izquierdo
        elif current_state == "SM_MLA":
            
            #Sequence for pringles
            #Acomodando el brazo
            q1,q2,q3,q4,q5,q6,q7 = ik_left_arm(0.20,-0.05,-0.25)
            move_left_arm(q1,q2,q3,q4,q5,q6,q7)
            move_left_gripper(0.7)
            #Acercando el brazo
            move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            move_left_gripper(-0.3)
            #Alejando el brazo de la mesa
            q1,q2,q3,q4,q5,q6,q7 = ik_left_arm(0.20,-0.05,-0.25)
            move_left_arm(q1,q2,q3,q4,q5,q6,q7)

            current_state = "SM_INIT_MOVE"
        
        #Mover brazo derecho
        elif current_state == "SM_MRA":

            #Sequence for drink
            #Acomodando el brazo
            q1,q2,q3,q4,q5,q6,q7 = ik_right_arm(0.20,0.05,-0.25)
            move_right_arm(q1,q2,q3,q4,q5,q6,q7)
            move_right_gripper(0.7)
            #Acercando el brazo
            move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            move_right_gripper(-0.3)
            #Alejando el brazo de la mesa
            q1,q2,q3,q4,q5,q6,q7 = ik_right_arm(0.20,0.05,-0.25)
            move_right_arm(q1,q2,q3,q4,q5,q6,q7)
            #To be code

            current_state = "SM_INIT_MOVE"
        
        #Salir del mapa de costo
        elif current_state == "SM_INIT_MOVE":
            move_base(-1,0,2)
            current_state = "SM_FOLLOW_PATH"
            pass

        #Mover a punto deseado
        elif current_state == "SM_FOLLOW_PATH":
            print("Llendo a: " + str(requested_location))
            go_to_goal_pose(requested_location[0],requested_location[1])
            goal_reached = False
            current_state = "SM_WAIT"
            pass
        
        #Se espera mientras deja el objeto
        elif current_state == "SM_WAIT":
            if goal_reached:
                print("Se ha llegado a: " + str(requested_location))
                goal_reached = False
                current_state = "SM_LEAVE_RETURN"

        #Suelta el objeto y regresa a la posicion inicial
        elif current_state == "SM_LEAVE_RETURN":
            print("Soltando el objeto...")
            if requested_object == "pringles":
                q1,q2,q3,q4,q5,q6,q7 = ik_left_arm(0.40,-0.05,-0.30)
                move_left_arm(q1,q2,q3,q4,q5,q6,q7)
                move_left_gripper(0.0)
                q1,q2,q3,q4,q5,q6,q7 = ik_left_arm(0.20,-0.05,-0.25)
                move_left_arm(q1,q2,q3,q4,q5,q6,q7)
            else: #Drink
                q1,q2,q3,q4,q5,q6,q7 = ik_right_arm(0.40,0.05,-0.37)
                move_right_arm(q1,q2,q3,q4,q5,q6,q7)
                move_right_gripper(0.0)
                q1,q2,q3,q4,q5,q6,q7 = ik_right_arm(0.20,0.05,-0.25)
                move_right_arm(q1,q2,q3,q4,q5,q6,q7)
            go_to_goal_pose(3.32,5.93)
            print("Regresando a casa...")
            goal_reached = False
            current_state = "SM_WAIT_RETURN"
        
        #Se regresa y ajusta posicion
        elif current_state == "SM_WAIT_RETURN":
            if goal_reached == True:
                print("Se regreso a casa...")
                goal_reached = False
                print("Ajustando posicion...")
                move_base(0.2,0,1.3)
                current_state = "SM_INIT"
                executing_task = False


        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
