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

parser.add_argument(
    "-pt",
    "--planning_time",
    type=float,
    default=2,
    help="specify the amount of time to be used for motion planning",
)

parser.add_argument(
    "-pa",
    "--num_planning_attempts",
    type=int,
    default=1,
    help="set the number of times the motion plan is to be computed from scratch before the shortest solution is returned",
)

args, unknown = parser.parse_known_args()

target_position_topic = args.target_position_topic
joint_cmd_topic = args.joint_cmd_topic
target_pose_topic = args.target_pose_topic
planning_time = args.planning_time
num_planning_attempts = args.num_planning_attempts

from moveit_commander import MoveGroupCommander
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import JointState
from typing import Union

rospy.init_node("airbot_twins_control")


def arm_init(group_name: str):
    arm = MoveGroupCommander(group_name)
    arm.set_pose_reference_frame("body_link")
    arm.set_planning_time(planning_time)
    arm.set_num_planning_attempts(num_planning_attempts)
    return arm


airbot_twins_left = arm_init("airbot_play_left_arm")
airbot_twins_right = arm_init("airbot_play_right_arm")
last_twins_cmd = Twist()
last_left_pose_cmd = Pose()
last_right_pose_cmd = Pose()
left_frame_id = "i"
right_frame_id = "i"
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
joint_cmd.header.frame_id = "waiting"

try:
    joint_cmd.position = list(airbot_twins_left.get_current_joint_values()) + list(
        airbot_twins_right.get_current_joint_values()
    )
except:
    joint_cmd.position = (0,) * 12
target_kind = "position"

success_dict = {
    0x00: "success",
    0x01: "right_faild",
    0x10: "left_faild",
    0x11: "both_faild",
}

result_dict = {
    "success": 0,
    "faild": 1,
    "ignore": 2,
}

def single_ik_process(
    target: Union[list, tuple, Pose], arm: MoveGroupCommander, target_type="position"
):
    """
    target: [x,y,z] or Pose
    target_type: "position" or "pose"
    return: target, result;
        target is None if failed or ignored;
        result is 0, 1 or 2;
    """
    if (target_type == "position" and target == [0, 0, 0]) or (
        target_type == "pose" and target == Pose()
    ):
        print(f"{arm.get_name()} ik ignored because target is all 0")
        return None, result_dict["ignore"]
    else:
        joint_cmd.header.frame_id = "planning"
        if target_type == "position":
            arm.set_position_target(target)
            plan_success, traj, planning_time, error_code = arm.plan()
            arm.clear_pose_targets()
        elif target_type == "pose":
            try:
                start_time = rospy.Time.now()
                arm.set_joint_value_target(target, arm.get_end_effector_link(), False)
                planning_time = (rospy.Time.now() - start_time).to_sec()
            except Exception as e:
                plan_success = False
            else:
                plan_success = True
    if plan_success:
        print(f"{arm.get_name()} ik success with time: {planning_time:.4f} seconds")
        if target_type == "position":
            target = list(traj.joint_trajectory.points[-1].positions)
        elif target_type == "pose":
            target = list(arm.get_joint_value_target())
        result = result_dict["success"]
    else:
        print(f"{arm.get_name()} ik failed with target:", target)
        target = None
        result = result_dict["faild"]
    return target, result


def twins_ik_process(
    left_target: Union[list, tuple, Pose, None],
    right_target: Union[list, tuple, Pose, None],
    target_type,
):
    global arm_joint_pos_target, left_frame_id, right_frame_id
    failure = 0x00
    # left arm
    left_target, result = single_ik_process(left_target, airbot_twins_left, target_type)
    if left_target is None:
        left_target = arm_joint_pos_target[:6]
        if result == result_dict["faild"]:
            failure = 0x10
    # right arm
    right_target, result = single_ik_process(
        right_target, airbot_twins_right, target_type
    )
    if right_target is None:
        right_target = arm_joint_pos_target[6:]
        if result == result_dict["faild"]:
            failure += 0x01
    # joint cmd
    joint_cmd.header.frame_id = success_dict[failure]
    arm_joint_pos_target = left_target + right_target
    joint_cmd.header.stamp = rospy.Time.now()


def arm_ik_position_callback(cmd_msg: Twist):
    global last_twins_cmd, target_kind
    target_kind = "position"
    if last_twins_cmd == cmd_msg:
        return
    left_position = [cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z]
    right_position = [cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z]
    twins_ik_process(left_position, right_position, "position")
    last_twins_cmd = cmd_msg


def arm_ik_pose_callback_left(cmd_msg: PoseStamped):
    global last_left_pose_cmd, arm_joint_pos_target, left_frame_id, target_kind
    target_kind = "pose"
    if last_left_pose_cmd == cmd_msg.pose:
        return
    target, result = single_ik_process(cmd_msg.pose, airbot_twins_left, "pose")
    if target is None:
        target = arm_joint_pos_target[:6]
        if result == result_dict["faild"]:
            frame_id = "f"
        else:
            frame_id = "i"
    else:
        frame_id = "s"

    arm_joint_pos_target[:6] = target
    left_frame_id = frame_id
    last_left_pose_cmd = cmd_msg.pose
    joint_cmd.header.stamp = rospy.Time.now()


def arm_ik_pose_callback_right(cmd_msg: PoseStamped):
    global last_right_pose_cmd, arm_joint_pos_target, right_frame_id, target_kind
    target_kind = "pose"
    if last_right_pose_cmd == cmd_msg.pose:
        return
    target, result = single_ik_process(cmd_msg.pose, airbot_twins_right, "pose")
    if target is None:
        target = arm_joint_pos_target[6:]
        if result == result_dict["faild"]:
            frame_id = "f"
        else:
            frame_id = "i"
    else:
        frame_id = "s"

    arm_joint_pos_target[6:] = target
    right_frame_id = frame_id
    last_right_pose_cmd = cmd_msg.pose
    joint_cmd.header.stamp = rospy.Time.now()


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
while (
    last_twins_cmd == Twist()
    and last_left_pose_cmd == Pose()
    and last_right_pose_cmd == Pose()
):
    if rospy.is_shutdown():
        exit()
    rospy.sleep(0.5)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if target_kind == "pose":
        joint_cmd.header.frame_id = left_frame_id + right_frame_id
    joint_cmd.position = tuple(arm_joint_pos_target)
    joint_cmd_puber.publish(joint_cmd)
    rospy.sleep(0.1)

""" An example of receiving ik result """
RECEIVE_EXAMLPE = False
if RECEIVE_EXAMLPE:
    joint = JointState()
    joint_last = JointState()

    def callback(msg: JointState):
        global joint
        joint = msg

    joint_cmd_suber = rospy.Subscriber("/airbot_play/joint_cmd", JointState, callback)
    pose_cmd_puber = rospy.Publisher(
        "/airbot_twins/pose_cmd", PoseStamped, queue_size=1
    )
    # wait for the target to be computed
    while joint.header.stamp == joint_last.header.stamp:
        target_pose = PoseStamped()
        pose_cmd_puber.publish(target_pose)
    # over
    joint_last = joint
