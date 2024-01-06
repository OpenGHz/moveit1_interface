from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import (
    GetPositionFK,
    GetPositionFKRequest,
    GetPositionFKResponse,
    GetPositionIK,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
from typing import Tuple, Union


class MoveItBasicController(MoveGroupCommander):
    """基于MoveIt! original controller 的 basic controller"""

    def __init__(
        self, name, robot_description="robot_description", ns="", wait_for_servers=5
    ):
        super().__init__(name, robot_description, ns, wait_for_servers)
        self.__robot = RobotCommander(robot_description, ns)
        self.__link_names = self.__robot.get_link_names(self.get_name())
        self.set_default_configs()
        self.__ns = ns
        self.__fk_service = rospy.ServiceProxy(self.__ns + "/compute_fk", GetPositionFK)
        self.__ik_service = rospy.ServiceProxy(self.__ns + "/compute_ik", GetPositionIK)

    def get_link_names(self) -> list:
        return self.__link_names

    def get_name_space(self) -> str:
        return self.__ns

    def set_default_configs(self) -> None:
        """设置默认参数"""
        self.set_pose_reference_frame(self.get_link_names()[0])
        # 规划时间、尝试次数等设置
        self.allow_replanning(False)  # 避障重规划
        self.set_planning_time(2)  # float ok
        self.set_num_planning_attempts(10)  # 规划失败尝试次数
        # 精度控制
        self.set_goal_position_tolerance(0.00005)  # 目标位置公差
        self.set_goal_orientation_tolerance(0.00001)  # 目标四元数公差
        self.set_goal_joint_tolerance(0.000017)  # 目标关节角度公差
        # 速度和加速度系数初始化
        self.set_max_acceleration_scaling_factor(1.0)  # 加速度限制
        self.set_max_velocity_scaling_factor(1.0)  # 速度限制

    def compute_forward_kinematics(
        self, joint_values: list, fk_link_names: list, reference_frame: str
    ) -> Tuple[tuple]:
        """正运动学：根据关节角度计算指定fk_link_names相对reference_frame的位姿"""
        robot_state = RobotState()
        joint_state = JointState()
        joint_state.name = self.get_active_joints()
        joint_state.position = joint_values
        robot_state.joint_state = joint_state

        fk_request = GetPositionFKRequest()
        fk_request.header.stamp = rospy.get_rostime()
        fk_request.header.frame_id = reference_frame
        fk_request.fk_link_names = fk_link_names
        fk_request.robot_state = robot_state

        fk_response: GetPositionFKResponse = self.__fk_service.call(fk_request)
        if fk_response.error_code.val == fk_response.error_code.SUCCESS:
            list_pose7 = []
            for pose in fk_response.pose_stamped:
                list_pose7.append(
                    (
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w,
                    )
                )
            return tuple(list_pose7)
        else:
            raise Exception(
                f"FK service failed with error code: {fk_response.error_code.val}"
            )

    # def compute_inverse_kinematics(self,pose:tuple,ik_link_name,reference_frame,time_out=1) -> Tuple[tuple]:
    #     """ 逆运动学：将pose(x,y,z,x,y,z,w)转换为joint角度目标 """
    #     pose = self.__tuple_to_pose(pose)
    #     ik_request = GetPositionIKRequest()
    #     ik_request.ik_request.group_name = self.get_name()
    #     ik_request.ik_request.ik_link_name = ik_link_name
    #     ik_request.ik_request.pose_stamped.header.frame_id = reference_frame
    #     ik_request.ik_request.pose_stamped.pose = pose
    #     ik_request.ik_request.avoid_collisions = True
    #     ik_request.ik_request.timeout = rospy.Duration(time_out)
    #     ik_response:GetPositionIKResponse = self.__ik_service.call(ik_request)
    #     if ik_response.error_code.val == ik_response.error_code.SUCCESS:
    #         return tuple(ik_response.solution.joint_state.name), tuple(ik_response.solution.joint_state.position)
    #     else:
    #         raise Exception(f"IK service failed with error code: {ik_response.error_code.val}")

    def compute_inverse_kinematics(  # TODO: basic中不应该有这个全功能函数，应迁移到upper中
        self, pose: tuple, approximate=False
    ) -> Union[tuple, None]:
        """逆运动学：将pose(x,y,z,x,y,z,w)转换为joint角度目标；如果失败，将打印错误信息并返回None"""
        pose_type = Pose()
        len_pose = len(pose)
        if len_pose == 3:
            pose_type.position.x, pose_type.position.y, pose_type.position.z = pose
            pose_kind = "position"
        elif len_pose == 4:
            (
                pose_type.orientation.x,
                pose_type.orientation.y,
                pose_type.orientation.z,
                pose_type.orientation.w,
            ) = pose
            pose_kind = "orientation"
        elif len_pose == 7:
            pose_type.position.x, pose_type.position.y, pose_type.position.z = pose[:3]
            (
                pose_type.orientation.x,
                pose_type.orientation.y,
                pose_type.orientation.z,
                pose_type.orientation.w,
            ) = pose[3:]
            pose_kind = "pose"
        else:
            raise Exception("pose must be a tuple of length 3, 4 or 7")
        if pose_kind == "pose":
            joints_value = self.pose_ik(pose, approximate)
        elif pose_kind == "position":
            joints_value = self.position_ik(pose)
        elif pose_kind == "orientation":
            joints_value = self.orientation_ik(pose)
        return joints_value

    def pose_ik(self, pose: tuple, approximate=False) -> Union[tuple, None]:
        """逆运动学：将pose(x,y,z,x,y,z,w)转换为joint角度目标；如果失败，将打印错误信息并返回None"""
        pose_type = Pose()
        pose_type.position.x, pose_type.position.y, pose_type.position.z = pose[:3]
        (
            pose_type.orientation.x,
            pose_type.orientation.y,
            pose_type.orientation.z,
            pose_type.orientation.w,
        ) = pose[3:]
        # if not ever set, the default value is all zero
        joints_target_last = list(self.get_joint_value_target())
        eef_link = self.get_end_effector_link()
        try:
            self.set_joint_value_target(pose_type, eef_link, approximate)
        except:
            joints_value = None
        else:
            joints_value = self.get_joint_value_target()
            self.set_joint_value_target(joints_target_last)
        return joints_value

    def position_ik(self, position: tuple) -> Union[tuple, None]:
        self.set_position_target(position)
        plan_success, traj, planning_time, error_code = self.plan()
        self.clear_pose_targets()
        if plan_success:
            joints_value = traj.joint_trajectory.points[-1].positions
        else:
            joints_value = None
        return joints_value

    def orientation_ik(self, orientation: tuple) -> Union[tuple, None]:
        self.set_orientation_target(orientation)
        plan_success, traj, planning_time, error_code = self.plan()
        self.clear_pose_targets()
        if plan_success:
            joints_value = traj.joint_trajectory.points[-1].positions
        else:
            joints_value = None
        return joints_value

    def __tuple_to_pose(self, pose: tuple) -> Pose:
        """将tuple转换为Pose"""
        pose_type = Pose()
        pose_type.position.x, pose_type.position.y, pose_type.position.z = pose[:3]
        (
            pose_type.orientation.x,
            pose_type.orientation.y,
            pose_type.orientation.z,
            pose_type.orientation.w,
        ) = pose[3:]
        return pose_type

    def get_joint_states_by_names(
        joint_names: tuple, all_joint_names: tuple, all_joint_values: tuple
    ) -> tuple:
        joint_states = [0 for _ in range(len(joint_names))]
        for i, name in enumerate(joint_names):
            joint_states[i] = all_joint_values[all_joint_names.index(name)]
        return tuple(joint_states)

    def get_joint_limits(self) -> tuple:
        pass
