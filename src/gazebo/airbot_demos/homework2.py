import numpy as np
from math import atan2, sqrt
import rospy
from airbot_play_control.control import RoboticArmAgent, AirbotPlayConfig
from moveit_commander import MoveGroupCommander

# ROS节点初始化
NODE_NAME = 'airbot_play_gazebo'
rospy.init_node(NODE_NAME)
# 初始化机械臂本体
arm = RoboticArmAgent(control_mode=AirbotPlayConfig.normal,
                      node_name=NODE_NAME, other_config=("", "airbot_play_arm"))

# 创建一个4x4的变换矩阵
transform_matrix = np.array([
    [0, 0, 1, 0.270704],
    [0, 1, 0, 0],
    [-1, 0, 0, -0.292618],
    [0, 0, 0, 1]
])
# 提取平移部分（xyz坐标）
translation = transform_matrix[:3, 3]
print("Translation (xyz):", translation)


# 提取旋转部分，将旋转矩阵转换为欧拉角（RPY角）
def rotation_matrix_to_rpy(matrix):
    # 计算欧拉角（RPY角）
    sy = sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = atan2(matrix[2, 1], matrix[2, 2])
        y = atan2(-matrix[2, 0], sy)
        z = atan2(matrix[1, 0], matrix[0, 0])
    else:
        x = atan2(-matrix[1, 2], matrix[1, 1])
        y = atan2(-matrix[2, 0], sy)
        z = 0

    return x, y, z

rotation_matrix = transform_matrix[:3, :3]
rpy_angles = rotation_matrix_to_rpy(rotation_matrix)
print("RPY Angles (degrees):", rpy_angles)
arm.set_and_go_to_pose_target(translation, rpy_angles, sleep_time=2)  # 到达指定位置

# 初始化夹爪(夹爪只能使用关节空间控制，因为是闭链结构，无法使用逆解)
gripper = MoveGroupCommander("airbot_play_gripper")
gripper.set_max_acceleration_scaling_factor(0.1)
gripper.set_max_acceleration_scaling_factor(0.1)
gripper.set_named_target("pick_big_cube")
gripper.go()
rospy.sleep(6)
gripper.set_max_acceleration_scaling_factor(1)
gripper.set_max_acceleration_scaling_factor(1)
arm.go_to_named_or_joint_target('Home')  # 机械臂到达初始位置

rospy.spin()

# # 控制机械臂末端到达指定的工作空间位置（欧拉角法）
# position = [0.270704, 0, -0.292618]  # xyz位置值
# rotation = [0, 1.5707963267948966, 0]  # rpy角度值或xyzw四元数
# arm.set_and_go_to_pose_target(position, rotation, sleep_time=2)

# # 控制机械臂末端到达指定的工作空间位置（四元数法）
# position = [0.270704,0,-0.292618]  # xyz位置值
# rotation = [0,0.7071,0,0.7071]  # rpy角度值或xyzw四元数
# arm.set_and_go_to_pose_target(position,rotation,sleep_time=2)

# # 控制所有关节转动到指定的角度位置（关节角法）
# arm.go_to_named_or_joint_target([0, -2.460914, 0.907571211, 0, 0, 0],sleep_time=2)