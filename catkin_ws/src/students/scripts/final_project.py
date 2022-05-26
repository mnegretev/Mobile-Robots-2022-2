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

    #
    # FINAL PROJECT
    # SE AGREGA LA MAQUINA DE ESTADOS
    #
   
    while not rospy.is_shutdown():
        if current_state == "SM_INIT": #State 0
            print("Starting state machine :D ...")
            if(new_task == True):
                new_task = False
                current_state = "SM_SAY_HELLO"

        elif current_state == "SM_SAY_HELLO": #STATE 1
            print("Saying hello")
            say("Hello world")
            #current_state = "SM_MOVE_ARM"
            current_state = "SM_OBJECT_DETECTION"

        elif current_state == "SM_OBJECT_DETECTION":#STATE 2
            print("Recognizing Object")
            say("Robot is recognizing object")
            obj,loc = parse_command(recognized_speech) #Se reconce el voz
            print(obj)
            print(loc)
            current_state = "SM_HEAD_DOWN"

        elif current_state == "SM_HEAD_DOWN":#STATE 3
            print("Moving head")
            say("Im sad") #CAMBIAR
            move_head(0,-0.9)
            current_state = "SM_FIND_OBJECT"

        elif current_state == "SM_FIND_OBJECT":
            print("Finding object")
            say("Finding object")
            x_obj, y_obj, z_obj = find_object(obj)
            print(x_obj, y_obj, z_obj)
            current_state == "SM_POINT_TRANSFORM"

        elif current_state == "SM_POINT_TRANSFORM":
            if obj == "pringles":
                xt, yt, zt = transform_point_to_left_arm(x_obj, y_obj, z_obj)

            else:
                xt, yt, zt = transform_point_to_right_arm(x_obj, y_obj, z_obj)
            print(xt, yt, zt)
            current_state = "SM_INVERSE_KINEMATICS"

        elif current_state == "SM_INVERSE_KINEMATICS":
            if obj == "pringles":
                q = ik_left_arm(xt,yt,zt)
            else:
                q = ik_right_arm(xt,yt,zt)
            print(q)
            current_state == "SM_MOVE_ARM_TO_START"

        elif current_state == "SM_MOVE_ARM_TO_START":
             print("Moving the robot's arm")
             move_left_arm(-0.5,0,0,2.1,0,0,0)
             current_state = "SM_MOVE_ARM_TO_TAKE_OBJ"

        elif current_state == "SM_MOVE_ARM_TO_TAKE_OBJ":
            print("moving_arm_obj")
            say("Robot is moving its arm")
            if obj == "pringles":
                move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            else:
                move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            current_state = "SM_CLOSE_HAND"
        
        elif current_state == "SM_CLOSE_HAND":
            print("closing hand")
            if obj == "pringles":
                move_left_gripper(q)
            else:
                move_right_gripper(q)
            current_state == "SM_FINISH"
        loop.sleep()
 
if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
