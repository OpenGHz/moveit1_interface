#!/usr/bin/env python3

import argparse

parser = argparse.ArgumentParser("AIRBOT_TWINS control node config.")

parser.add_argument(
    "-tpi",
    "--target_position_topic",
    type=str,
    default="/airbot_twins/position_cmd",
    help="topic name of target position, the type should be geometry_msgs/Twist",
)

parser.add_argument(
    "-tpe",
    "--target_pose_topic",
    type=str,
    default="/airbot_twins/pose_cmd",
    help="namespace of target pose topic, the raw name is /letf and /right, the type of both should be geometry_msgs/PoseStamped",
)

parser.add_argument(
    "-jc",
    "--joint_cmd_topic",
    type=str,
    default="/airbot_play/joint_cmd",
    help="topic name of joint cmd, the type should be sensor_msgs/JointState",
)

args, unknown = parser.parse_known_args()

target_position_topic = args.target_position_topic
joint_cmd_topic = args.joint_cmd_topic
target_pose_topic = args.target_pose_topic

from moveit_commander import MoveGroupCommander
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState

rospy.init_node("airbot_twins_control")

airbot_twins_left = MoveGroupCommander("airbot_play_left_arm")
airbot_twins_right = MoveGroupCommander("airbot_play_right_arm")
last_twins_cmd = Twist()
joint_cmd_puber = rospy.Publisher(joint_cmd_topic, JointState, queue_size=1)
arm_joint_pos_target = [0] * 12
joint_cmd = JointState()
joint_cmd.name = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
]
joint_cmd.header.frame_id = "success"

success_dict = {
    0x00: "success",
    0x01: "right_faild",
    0x10: "left_faild",
    0x11: "both_faild",
}


def arm_ik_position_callback(cmd_msg: Twist):
    global last_twins_cmd, arm_joint_pos_target
    if last_twins_cmd == cmd_msg:
        return
    failure = 0x00
    left_position = [cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z]
    right_position = [cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z]
    # left arm
    if left_position == [0, 0, 0]:
        plan_success = False
    else:
        airbot_twins_left.set_position_target(left_position)
        plan_success, traj, planning_time, error_code = airbot_twins_left.plan()
        airbot_twins_left.clear_pose_targets()
        print("left ik success with planning time:", planning_time, "seconds")
    if plan_success:
        left_target = list(traj.joint_trajectory.points[-1].positions)
    else:
        failure = 0x10
        left_target = arm_joint_pos_target[0:6]
        print("left ik failed with target:", left_position)
    # right arm
    if right_position == [0, 0, 0]:
        plan_success = False
    else:
        airbot_twins_right.set_position_target(right_position)
        plan_success, traj, planning_time, error_code = airbot_twins_right.plan()
        airbot_twins_right.clear_pose_targets()
        print("right ik success with planning time:", planning_time, "seconds")
    if plan_success:
        right_target = list(traj.joint_trajectory.points[-1].positions)
    else:
        failure += 0x01
        right_target = arm_joint_pos_target[6:12]
        print("right ik failed with target:", right_position)
    joint_cmd.header.frame_id = success_dict[failure]
    arm_joint_pos_target = left_target + right_target
    last_twins_cmd = cmd_msg


def arm_ik_pose_callback_left(cmd_msg: PoseStamped):
    global last_twins_cmd
    if last_twins_cmd == cmd_msg:
        return
    try:
        airbot_twins_left.set_joint_value_target(
            cmd_msg, airbot_twins_left.get_end_effector_link(), False
        )
    except Exception as e:
        print("set_joint_value_target error:", e)
    else:
        arm_joint_pos_target[:6] = airbot_twins_left.get_joint_value_target()

    last_twins_cmd = cmd_msg


def arm_ik_pose_callback_right(cmd_msg: PoseStamped):
    global last_twins_cmd
    if last_twins_cmd == cmd_msg:
        return
    try:
        airbot_twins_right.set_joint_value_target(
            cmd_msg, airbot_twins_right.get_end_effector_link(), False
        )
    except Exception as e:
        print("set_joint_value_target error:", e)
    else:
        arm_joint_pos_target[6:] = airbot_twins_right.get_joint_value_target()

    last_twins_cmd = cmd_msg


position_cmd_suber = rospy.Subscriber(
    target_position_topic, Twist, arm_ik_position_callback, queue_size=1
)

pose_cmd_suber_left = rospy.Subscriber(
    target_pose_topic + "/left",
    PoseStamped,
    arm_ik_pose_callback_left,
    queue_size=1,
)

pose_cmd_suber_right = rospy.Subscriber(
    target_pose_topic + "/right",
    PoseStamped,
    arm_ik_pose_callback_right,
    queue_size=1,
)

# wait for first cmd
print("waiting for first cmd...")
while last_twins_cmd == Twist():
    if rospy.is_shutdown():
        exit()
    rospy.sleep(0.5)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    joint_cmd.position = arm_joint_pos_target
    joint_cmd.header.stamp = rospy.Time.now()
    joint_cmd_puber.publish(joint_cmd)
    rospy.sleep(0.1)
