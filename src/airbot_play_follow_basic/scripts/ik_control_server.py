#!/usr/bin/env python3

from moveit_basic_controller import MoveItBasicController
from airbot_play_ik_service.srv import (
    airbot_play_ik,
    airbot_play_ikRequest,
    airbot_play_ikResponse,
)
import rospy
from moveit_commander import conversions


class IkControlServer(MoveItBasicController):
    """不允许多线程调用服务"""

    def __init__(
        self, name, robot_description="robot_description", ns="", wait_for_servers=5
    ):
        super().__init__(name, robot_description, ns, wait_for_servers)
        self.__ns = ns
        self.ik_server = rospy.Service(
            f"{ns}/ik_service", airbot_play_ik, self._handle_ik
        )
        self._no_plan = False

    def no_plan(self):
        self._no_plan = True

    def _handle_ik(self, req: airbot_play_ikRequest):
        response = airbot_play_ikResponse()
        attempts_ik = 20
        attempts_plan = 20
        success = 0
        while attempts_ik > 0:
            attempts_ik -= 1
            if req.target_pose.header.frame_id != "":
                self.set_pose_reference_frame(req.target_pose.header.frame_id)
            pose_list = conversions.pose_to_list(req.target_pose.pose)
            if pose_list[3:] == [0, 0, 0, 0]:
                joint_target = self.position_ik(tuple(pose_list[:3]))
            else:
                joint_target = self.pose_ik(tuple(pose_list))
            if joint_target is not None:
                rospy.set_param(f"{self.__ns}/joint_target", joint_target)
            if joint_target is None:
                rospy.set_param(f"{self.__ns}/joint_target", "None")
                continue
            elif not self._no_plan:
                attempts_ik = 20  # reset attempts_ik
                attempts_plan -= 1
                # `go()` returns a boolean indicating whether the planning and execution was successful.
                if self.go(joint_target, wait=True):
                    # TODO: 不用必须等待执行
                    # Calling `stop()` ensures that there is no residual movement
                    self.stop()
                    success = 1
                    break
                elif attempts_plan <= 0:
                    rospy.logerr(
                        f"Planning failed after {attempts_plan} attempts with target:\n {req.target_pose.pose}."
                    )
                    break
                else:
                    continue
            else:
                success = 1
                break
        else:
            rospy.logerr(
                f"IK failed after {attempts_ik} attempts with target:\n {req.target_pose.pose}."
            )
        response.result = success
        return response


if __name__ == "__main__":
    import argparse
    from geometry_msgs.msg import PoseStamped

    parser = argparse.ArgumentParser("AIRbotPlay PickPlace param set.")
    parser.add_argument(
        "-gn",
        "--group_name",
        type=str,
        default="airbot_play_arm",
        help="target move group name",
    )
    parser.add_argument(
        "-rd",
        "--robot_description",
        type=str,
        default="/airbot_play/robot_description",
        help="name of robot description param name, the name space won't be auto added to it",
    )
    parser.add_argument(
        "-ns",
        "--name_space",
        type=str,
        default="/airbot_play",
        help="namespace of moveit",
    )
    parser.add_argument(
        "-pr",
        "--pose_reference_frame",
        type=str,
        default="auto",
        help="pose reference frame",
    )
    parser.add_argument("-p", "--use_plan", action="store_true", help="test mode")
    parser.add_argument("-t", "--test", action="store_true", help="test mode")
    args, unknown = parser.parse_known_args()

    NODE_NAME = "airbot_play_moveit_ik_control_server"
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Initializing {} Node.".format(NODE_NAME))

    mc = IkControlServer(args.group_name, args.robot_description, args.name_space)
    if args.pose_reference_frame != "auto":
        mc.set_pose_reference_frame(args.pose_reference_frame)
    if not args.use_plan:
        mc.no_plan()

    # print args
    print("********************************")
    print("name_space: ", args.name_space)
    print("group_name: ", args.group_name)
    print("robot_description: ", args.robot_description)
    print("pose_reference_frame: ", mc.get_pose_reference_frame())
    print("use_plan: ", args.use_plan)
    print("test: ", args.test)
    print("********************************")

    if args.test:
        client = rospy.ServiceProxy(f"{args.name_space}/ik_service", airbot_play_ik)
        client.wait_for_service(timeout=5)
        req = airbot_play_ikRequest()
        req.target_pose = mc.get_current_pose()
        req.target_pose.pose.position.z += 0.05
        res: airbot_play_ikResponse = client.call(req)
        # TODO: 响应是否改成joint_states
        print("ik response is：", res.result, ", 1 means success; 0 means fail")
        joint_target = rospy.get_param(f"{args.name_space}/joint_target", "None")
        print("joint_target is：", joint_target)
        req.target_pose.pose.orientation = PoseStamped().pose.orientation
        res: airbot_play_ikResponse = client.call(req)
        print("ik response is：", res.result, ", 1 means success; 0 means fail")
        joint_target = rospy.get_param(f"{args.name_space}/joint_target", "None")
        print("joint_target is：", joint_target)

    # 发布当前末端位姿话题
    current_pose_pub = rospy.Publisher(
        f"{args.name_space}/current_pose", PoseStamped, queue_size=1
    )

    rt = rospy.Rate(200)
    while not rospy.is_shutdown():
        eef_pose = mc.get_current_pose()
        current_pose_pub.publish(eef_pose)
        rt.sleep()
