#!/usr/bin/env python3

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("AIRbotPlay PickPlace param set.")
    parser.add_argument('-gn','--group_name',type=str,default='airbot_play_gripper',help='target move group name')
    parser.add_argument('-rd','--robot_description',type=str,default='/airbot_play/robot_description',help='name of robot description')
    parser.add_argument('-ns','--name_space',type=str,default='/airbot_play',help='namespace of moveit')
    args, unknown = parser.parse_known_args()

    import rospy
    NODE_NAME = 'airbot_play_moveit_inspire_gripper_control'
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Initializing {} Node.".format(NODE_NAME))

    from moveit_commander import MoveGroupCommander
    from std_msgs.msg import String, Bool
    from threading import Thread

    airbot_play_gripper = MoveGroupCommander(args.group_name,args.robot_description,args.name_space)
    airbot_play_gripper.set_max_acceleration_scaling_factor(0.1)
    airbot_play_gripper.set_max_acceleration_scaling_factor(0.1)

    gripper_state = "open"

    def gripper_control(pp:Bool):
        global gripper_state
        if gripper_state == "moving":
            return
        gripper_state = "moving"
        if pp.data:
            SCALE_CLOSE = 0.7
            delta = 0.04
            CLOSE = (-SCALE_CLOSE,SCALE_CLOSE,SCALE_CLOSE,-SCALE_CLOSE + delta)
            airbot_play_gripper.go(CLOSE)
            gripper_state = "close"
        else:
            airbot_play_gripper.set_named_target("open")
            airbot_play_gripper.go(wait=True)
            gripper_state = "open"

    gripper_suber = rospy.Subscriber("/airbot_play/gripper/state_cmd", Bool, gripper_control, queue_size=10)

    gripper_state_puber = rospy.Publisher("/airbot_play/gripper/current_state", String, queue_size=10)
    def gripper_state_pub():
        rate = rospy.Rate(200)
        st = String()
        while not rospy.is_shutdown():
            st.data = gripper_state
            gripper_state_puber.publish(st)
            rate.sleep()
    Thread(target=gripper_state_pub,daemon=True).start()

    rospy.spin()
