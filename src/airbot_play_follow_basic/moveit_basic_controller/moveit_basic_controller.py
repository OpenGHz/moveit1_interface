from moveit_commander import MoveGroupCommander,RobotCommander
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse, GetPositionIK
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import rospy
from typing import Tuple, Optional


class MoveItBasicController(MoveGroupCommander):
    """ 基于MoveIt! original controller 的 basic controller """
    def __init__(self, name, robot_description="robot_description", ns="", wait_for_servers=5):
        super().__init__(name, robot_description, ns, wait_for_servers)
        self.__robot = RobotCommander(robot_description,ns)
        self.__link_names = self.__robot.get_link_names(self.get_name())
        self.set_default_configs()
        self.__ns = ns
        self.__fk_service = rospy.ServiceProxy(self.__ns + '/compute_fk', GetPositionFK)

    def get_link_names(self) -> list:
        return self.__link_names

    def get_name_space(self) -> str:
        return self.__ns

    def set_default_configs(self) -> None:
        """ 设置默认参数 """
        self.set_pose_reference_frame("base_link")
        # 规划时间、尝试次数等设置
        self.allow_replanning(False)  # 避障重规划
        self.set_planning_time(2)  # float ok
        self.set_num_planning_attempts(10) # 规划失败尝试次数
        # 精度控制
        self.set_goal_position_tolerance(0.00005)  # 目标位置公差
        self.set_goal_orientation_tolerance(0.00001)  # 目标四元数公差
        self.set_goal_joint_tolerance(0.000017)  # 目标关节角度公差
        # 速度和加速度系数初始化
        self.set_max_acceleration_scaling_factor(1.0) # 加速度限制
        self.set_max_velocity_scaling_factor(1.0)     # 速度限制

    def compute_forward_kinematics(self, joint_values:list, fk_link_names:list, reference_frame:str) -> Optional[Tuple[tuple]]:
        """ 正运动学：根据关节角度计算指定fk_link_names相对reference_frame的位姿; 错误则返回None"""
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

        fk_response:GetPositionFKResponse = self.__fk_service.call(fk_request)
        if fk_response.error_code.val == fk_response.error_code.SUCCESS:
            list_pose7 = []
            for pose in fk_response.pose_stamped:
                list_pose7.append((pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
                                   pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))
            return tuple(list_pose7)
        else:
            rospy.logerr(f"FK service failed with error code: {fk_response.error_code.val}")
            return None

    def compute_inverse_kinematics(self,pose:tuple,approximate=False) -> tuple:
        """ 逆运动学：将pose(x,y,z,x,y,z,w)转换为joint角度目标 """
        pose_type = Pose()
        pose_type.position.x, pose_type.position.y, pose_type.position.z = pose[:3]
        pose_type.orientation.x, pose_type.orientation.y, pose_type.orientation.z, pose_type.orientation.w = pose[3:]
        joints_target_last = self.get_joint_value_target()
        self.set_joint_value_target(pose_type, self.get_end_effector_link(), approximate)
        joints_value = tuple(self.get_joint_value_target())
        self.set_joint_value_target(joints_target_last)
        return joints_value

    def get_joint_states_by_names(joint_names:tuple,all_joint_names:tuple,all_joint_values:tuple) -> tuple:
        joint_states = [0 for _ in range(len(joint_names))]
        for i, name in enumerate(joint_names):
            joint_states[i] = all_joint_values[all_joint_names.index(name)]
        return tuple(joint_states)
