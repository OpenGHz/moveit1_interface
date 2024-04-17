#!/usr/bin/env python3
""" 通用版 """
import math
import rospy
from actionlib import SimpleActionServer
from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from typing import List, Union
import numpy as np
import functools

from robot_tools.interpolate import Interpolate


class MoveItAction(object):
    CONTROL_MODE = ["torque", "position"]  # 控制模式选择
    _result = FollowJointTrajectoryResult()

    # Action initialisation
    def __init__(self, name, interpolation, default_position, topic, frequency):
        """
        name：action服务器名（由相应ros_controller配置文件yaml定义）。
        interpolation支持：1为线性插值；2-5为n次样条插值。
        """
        self.interpolation = interpolation
        self.t_delta = 1 / frequency  # 单位:s
        self.init_pose = default_position  # 初始状态（单位为rad，注意防止零位干涉）
        self.joint_nums = len(default_position)

        self.new_action = True  # do not send init cmd
        self.times = 0
        self.max_times = 100

        self.cmd_publisher = rospy.Publisher(topic, JointState, queue_size=10)
        self.cmd = JointState()
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.header.frame_id = "airbot_play_moveit1"
        self.cmd.position = self.init_pose
        # 初始化速度值23度/s，从而控制电机缓慢移动到0位
        self.cmd.velocity = [0.8 for _ in range(self.joint_nums)]
        self.cmd.effort = [0 for _ in range(self.joint_nums)]
        self.is_sending_cmd = False

        rospy.Timer(rospy.Duration(self.t_delta), self.control_continue)

        self._action_server = SimpleActionServer(
            name,
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._action_server.start()

    def execute_cb(self, goal_handle: FollowJointTrajectoryGoal):
        """Action Callback"""
        # 避免函数被重复运行
        if self.is_sending_cmd:
            return
        else:
            self.is_sending_cmd = True

        # 获得目标途径上的所有轨迹点（无任何外在约束的情况下一般只有两个点）
        points: List[JointTrajectoryPoint] = goal_handle.trajectory.points
        joint_names = goal_handle.trajectory.joint_names
        assert len(joint_names) == self.joint_nums
        self.joint_names = joint_names
        # 构造初始矩阵
        times = len(points)
        time_array = np.zeros(times)  # 关节位置的时间信息列表
        # 存储关节位置插值数据用的列表
        joints_matrix = np.zeros((times, self.joint_nums))
        joints_vel_matrix = joints_matrix.copy()
        joints_acc_matrix = joints_matrix.copy()
        # 分别获得各个点的时间信息和位置信息，放到两个列表中
        for time, point in enumerate(points):  # 单位为rad；每个point对应了某个时间点
            # 不同关节具有相同的时间点
            time_array[time] = point.time_from_start.to_sec()
            # point.positions[j]存储了关节j在当前时间的角度值；joints_matrix每一行存储了某个时间点所有关节的角度目标
            joints_matrix[time] = point.positions
            joints_vel_matrix[time] = point.velocities
            joints_acc_matrix[time] = point.accelerations
        # 轨迹插值
        execute_time_array = Interpolate.time_clip(
            0, time_array[times - 1], self.t_delta
        )  # 将轨迹首尾点的时间以t_delta来分段，与rospy.Rate(1/t_delta)对应上
        new_times = len(execute_time_array)
        if isinstance(self.interpolation, int):
            if self.interpolation == 1:  # 线性插值
                interpolate = functools.partial(
                    Interpolate.linear_interpolate, t=time_array, t_i=execute_time_array
                )
            elif self.interpolation == 3:  # 三次B样条插值
                interpolate = functools.partial(
                    Interpolate.cubic_spline, t=time_array, t_i=execute_time_array
                )
            elif 1 < self.interpolation <= 5:  # 五次B样条插值
                interpolate = functools.partial(
                    Interpolate.spline,
                    t=time_array,
                    t_i=execute_time_array,
                    k=self.interpolation,
                )
            else:
                raise NotImplementedError(f"暂不支持{self.interpolation}插值")
        else:
            raise NotImplementedError(f"暂不支持{self.interpolation}插值")
        # 速度限制
        min_vel = 2.593
        max_vel = 4 * math.pi
        # axis=0表示对各个列施加函数
        joints_matrix_new = np.apply_along_axis(interpolate, axis=0, arr=joints_matrix)
        speed_matrix = np.apply_along_axis(
            Interpolate.linear_speed_calculate,
            axis=0,
            arr=joints_matrix_new,
            t=self.t_delta,
        )  # 速度计算
        joints_list = joints_matrix_new.tolist()

        # 对速度求绝对值，因为底层仅接收正速度；限制最小速度
        speed_min_limit_matrix = np.abs(speed_matrix) + min_vel
        # 限制最大速度
        speed_list: list = np.where(
            speed_min_limit_matrix > max_vel, max_vel, speed_min_limit_matrix
        ).tolist()
        # 发送第一个点的期望速度设置为与第二个点相同
        speed_list.insert(0, list(speed_list[0]))

        # 遍历所有插值时间发送cmd
        self.new_action = True
        while self.continue_ok:
            pass
        r = rospy.Rate(1 / self.t_delta)  # 控制频率设置
        for i in range(new_times):  # 遍历每一个插值点
            # 若有新的action中断发生，则立刻终止当前处理，转而下一个处理
            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                self.is_sending_cmd = False
                self.new_action = False
                return
            # 控制命令发布
            self.cmd.velocity = speed_list[i]
            self.cmd.position = joints_list[i]
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd.name = self.joint_names
            self.cmd_publisher.publish(self.cmd)
            r.sleep()
        self.new_action = False
        self.times = 0
        # 设置结果通知MoveIt执行完毕
        self._result.error_code = 0
        self._action_server.set_succeeded(self._result)
        self.is_sending_cmd = False

    def control_continue(self, event):
        """接收到控制消息后，发送完仍继续发送一段时间最后的目标值，防止丢包造成位置误差增大"""
        self.continue_ok = False
        if not self.new_action:
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd_publisher.publish(self.cmd)
            self.continue_ok = True
            if self.times >= self.max_times:
                self.times = 0
                self.new_action = True
                self.max_times = 20  # 初始化为100，因为初始化的时候由于启动不同步问题容易丢包；之后都是20；
            else:
                self.times += 1


if __name__ == "__main__":
    NODE_NAME = "follow_joint_trajectory_server"

    def loginfo(msg: str):
        rospy.loginfo("[{}] {}".format(NODE_NAME, msg))

    rospy.init_node(NODE_NAME)
    loginfo("Initializing {} node.".format(NODE_NAME))
    action_name = rospy.get_param(
        "~action_name", default="arm_position_controller/follow_joint_trajectory"
    )
    default_position = rospy.get_param("~default_position", default="0,0,0,0,0,0")
    interpolation_type = rospy.get_param("~interpolation_type", default=5)
    cmd_topic = rospy.get_param("~cmd_topic", default="/airbot_play/joint_cmd")
    default_position = [float(pos) for pos in default_position.split(",")]
    frequency = rospy.get_param("~frequency", default=200)
    MoveItAction(action_name, interpolation_type, default_position, cmd_topic, frequency)

    rospy.loginfo("Ready to follow joint trajectory.")
    rospy.spin()
