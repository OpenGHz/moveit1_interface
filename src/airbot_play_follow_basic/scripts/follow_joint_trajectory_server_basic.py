#!/usr/bin/env python3
""" 通用版 """
import math
import rospy
from actionlib import SimpleActionServer

from scipy.interpolate   import CubicSpline,make_interp_spline,interp1d
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from typing import List,Union

import matplotlib.pyplot as plt
import numpy as np
import functools
from sensor_msgs.msg import JointState


class MoveItAction(object):
	# TODO：这两个类目前还没有怎么用起来
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()
    CONTROL_MODE = ['torque','position']  # 控制模式选择
	# Action initialisation
    def __init__(self,name,control_mode,interpolation=5):
        """
            name：action服务器名。
            interpolation支持：1为线性插值；2-5为n次样条插值。
            control_mode：力控或位控（CONTROL_MODE = ['torque','position']  # 控制模式选择）
        """
        self.interpolation = interpolation
        self.t_delta = 0.005  # 单位:s
        self.arm_joint_nums = 6
        self.gripper_joint_nums = 0
        self.all_joint_nums = self.arm_joint_nums + self.gripper_joint_nums
        self.init_pose = [0,-0.025,0.025,0,0,0]  # 初始状态（单位为rad，防止零位干涉）

        self.new_action = False
        self.times = 0; self.max_times = 100

        self.cmd_publisher = rospy.Publisher('/airbot_play/joint_cmd', JointState, queue_size=10)
        self.cmd = JointState()
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.header.frame_id = 'airbot_play'
        # self.cmd.mode = control_mode
        # self.cmd.kp = [100 for _ in range(self.arm_joint_nums)]
        # self.cmd.kd = [10 for _ in range(self.arm_joint_nums)]
        self.cmd.position = self.init_pose
        self.cmd.velocity = [0.8 for _ in range(self.arm_joint_nums)]  # 初始化速度值23度/s，从而控制电机缓慢移动到0位
        self.cmd.effort = [0 for _ in range(self.arm_joint_nums)]
        # self.cmd.id = [id for id in range(1,self.arm_joint_nums+1)]

        rospy.Timer(rospy.Duration(1. / 200.),self.control_continue)

        self._action_server = SimpleActionServer(name,FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._action_server.start()

    def execute_cb(self,goal_handle:Union[FollowJointTrajectoryGoal,list]):
        """ Action Callback """
        # 避免函数被重复运行
        if not hasattr(self.execute_cb,'running'):
            self.execute_cb.__dict__['running'] = False
        if self.execute_cb.__dict__['running']: return
        else: self.execute_cb.__dict__['running'] = True

        # 不同类型处理（获得统一的points变量）
        points:List[JointTrajectoryPoint] = goal_handle.trajectory.points  # 获得目标途径上的所有轨迹点（无任何外在约束的情况下一般只有两个点）

        # 构造初始矩阵
        times = len(points)
        time_array = np.zeros(times)  # 关节位置的时间信息列表
        joints_matrix = np.zeros((times,self.arm_joint_nums))  # 存储关节位置插值数据用的列表
        joints_vel_matrix = joints_matrix.copy()
        joints_acc_matrix = joints_matrix.copy()
        # 分别获得各个点的时间信息和位置信息，放到两个列表中
        for time,point in enumerate(points):  # 单位为rad；每个point对应了某个时间点
            time_array[time] = point.time_from_start.to_sec()  # 不同关节具有相同的时间点
            joints_matrix[time] = point.positions  # point.positions[j]存储了关节j在当前时间的角度值；joints_matrix每一行存储了某个时间点所有关节的角度目标
            joints_vel_matrix[time] = point.velocities
            joints_acc_matrix[time] = point.accelerations

        # 轨迹插值
        execute_time_array = Interpolate.time_clip(0,time_array[times-1],self.t_delta)  # 将轨迹首尾点的时间以5ms来分段，与rospy.Rate(200)对应上
        new_times = len(execute_time_array)
        if isinstance(self.interpolation,int):
            if self.interpolation == 1:  # 线性插值
                interpolate = functools.partial(Interpolate.linear_interpolate,t=time_array,t_i=execute_time_array)
            elif self.interpolation == 3:  # 三次B样条插值
                interpolate = functools.partial(Interpolate.cubic_spline,t=time_array,t_i=execute_time_array)
            elif 1 < self.interpolation <= 5:  # 五次B样条插值
                interpolate = functools.partial(Interpolate.spline,t=time_array,t_i=execute_time_array,k=self.interpolation)
            else: exit('暂不支持该类型的插值方式')
        else: exit('暂不支持该类型的插值方式')

        min_vel = 2.593
        max_vel = 4*math.pi
        
        joints_matrix_new = np.apply_along_axis(interpolate,axis=0,arr=joints_matrix)  # axis=0表示对各个列施加函数
        speed_matrix = np.apply_along_axis(Interpolate.linear_speed_calculate,axis=0,arr=joints_matrix_new,t=self.t_delta)  # 速度计算
        joints_list = joints_matrix_new.tolist()

        speed_min_limit_matrix = np.abs(speed_matrix) + min_vel  # 对速度求绝对值，因为底层仅接收正速度；限制最小速度
        speed_list:list = np.where(speed_min_limit_matrix>max_vel,max_vel,speed_min_limit_matrix).tolist()  # 限制最大速度
        speed_list.insert(0,list(speed_list[0]))  # 发送第一个点的期望速度设置为与第二个点相同

        # 遍历所有插值时间发送cmd
        self.new_action = True
        while self.continue_ok: pass
        r = rospy.Rate(200)  # 200Hz，5ms控制频率
        self.target_mode = rospy.get_param('/airbot_play/target_mode',default=-1)  # 控制的对象确定（是否fake）
        for i in range(new_times):  # 遍历每一个插值点
            # 若有新的action中断发生，则立刻终止当前处理，转而下一个处理
            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                self.execute_cb.__dict__['running'] = False
                self.new_action = False
                return
            # 控制命令发布（只发6个关节的，即cmd长度为6）
            self.cmd.velocity = speed_list[i]
            self.cmd.position = joints_list[i]
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd_publisher.publish(self.cmd)
            r.sleep()
        self.new_action = False
        self.times = 0
	    # 设置结果通知MoveIt执行完毕
        self._result.error_code = 0
        self._action_server.set_succeeded(self._result)
        self.execute_cb.__dict__['running'] = False

    def control_continue(self,event):
        """ 接收到控制消息后，发送完仍继续发送一段时间最后的目标值，防止丢包造成位置误差增大 """
        self.continue_ok = False
        if not self.new_action:
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd_publisher.publish(self.cmd)
            self.continue_ok = True
            if self.times >= self.max_times:
                self.times = 0
                self.new_action = True
                self.max_times = 20  # 初始化为100，因为初始化的时候由于启动不同步问题容易丢包；之后都是20；
            else: self.times += 1


class Interpolate(object):
    """ 轨迹插值 """
    @staticmethod
    def time_clip(start,end,interval,unit='s',end_control=False):
        """ms级的时间细化
            unit表示所给三个时间的单位，s或ms两种。
            end_control决定是否根据插分后的最后一个值与差分前的最后值相等进行末尾时间控制
        """
        if unit == 's': precision = 0.001
        elif unit == 'ms': precision = 1
        time_line = (np.array([start,end,interval])/precision).astype('int32')
        time_clipped = np.arange(time_line[0],time_line[1],step=time_line[2])
        if end_control:
            if time_clipped[-1] != time_line[1]:
                time_clipped = np.append(time_clipped,time_line[1])
        else: time_clipped = np.append(time_clipped,time_line[1])
        time_clipped = time_clipped.astype('float64')
        time_clipped*=precision
        return time_clipped

    @classmethod
    def no_interpolate(cls,y,t,t_i,unit='s',sort=False,plot=False):
        """ 无插值 """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:
            flag = False
            t_interp = t_i
        # 无插值
        stage_num = len(t)
        def stage(t_):
            for i in range(stage_num-1):
                if t[i] <= t_ <= t[i+1]:
                    return y[i+1]
        y_interp = []
        for t_ in t_interp:
            y_interp.append(stage(t_))
        # 是否绘制曲线
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @classmethod
    def linear_interpolate(cls,y,t,t_i,unit='s',sort=False,plot=False):
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:
            flag = False
            t_interp = t_i
        # 使用LinearNDInterpolator进行线性插值
        y_interp = interp1d(t,y)(t_interp)
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @classmethod
    def cubic_spline(cls,y,t,t_i,unit='s',bc_type='clamped',sort=False,plot=False):
        """
        使用 UnivariateSpline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
            ti:
            unit: 时间单位是s还是ms
            bc_type: 'not-a-knot','natural'，'clamped'
        返回值:
            返回一个二元组，包含插值后的时间序列和对应的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:
            flag = False
            t_interp = t_i
        # 使用CubicSpline进行3次样条插值
        cnt = 1
        while True:
            try: y_interp = CubicSpline(t,y,bc_type=bc_type)(t_interp)
            except: t[-cnt] += 0.0003/cnt  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else: break
            cnt += 1
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @classmethod
    def spline(cls,y,t:np.ndarray,t_i,k=5,unit='s',sort=False,plot=False):
        """
        使用 make_interp_spline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
        返回值:
            当t_i为间隔时，返回一个二元组，包含插值后的时间序列和对应的函数值
            当t_i为时间序列时，仅返回插值后的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t,y = t[idx],y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i,(float,int)):  # 指定频率
            flag = True
            t_interp = cls.time_clip(t[0],t[-1],t_i,unit)
        else:  # 指定已经细化的时间点
            flag = False
            t_interp = t_i
        # 使用make_interp_spline进行样条插值
        cnt = 1
        while True:
            try: y_interp = make_interp_spline(t,y,k=k,bc_type=(((1, 0), (2, 0)),((1, 0), (2, 0))))(t_interp)
            except: t[-cnt] += 0.0003/cnt  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else: break
            cnt += 1
        if plot: cls.plot(t,y,t_interp,y_interp)
        # 返回插值后的时间序列和函数值
        if flag: return t_interp, y_interp
        else: return y_interp

    @staticmethod
    def linear_speed_calculate(y:np.ndarray,t):
        """ 计算曲线两点间的直线变化率 """
        y_cp = y.copy()
        y1 = np.delete(y,0)
        y2 = np.delete(y_cp,-1)
        if isinstance(t,(np.ndarray,list,tuple)):
            return (y1-y2)/(t[1:]-t[:-1])
        else: return (y1-y2)/t

    @staticmethod
    def interval_limit(y_i:np.ndarray,min_delta=0.001,max_delta=3):
        """ 控制插值后y方向的间隔 """
        end_index = y_i.shape[0] - 1
        i = 1
        # 中间元素限幅
        while i < end_index:
            if math.fabs(y_i[i] - y_i[i-1]) < min_delta:
                y_i = np.delete(y_i,i)
                end_index -= 1  # 删除中间元素后有效长度-1（即有效长度始终记录的是0-原末尾元素的长度）
                y_i = np.append(y_i,y_i[end_index])
            else: i += 1  # 未删除元素计数+1
        # 末尾（原）两元素限幅
        if math.fabs(y_i[end_index] - y_i[end_index-1]) < min_delta:
            y_i[end_index-1] = y_i[end_index]
        return y_i

    @staticmethod
    def plot(t,y,t_interp,y_interp,pause=0,clear=True,ion=False,block=False):
        """
        绘制样条插值的结果:
            主要参数:
                t: 时间序列，一个一维数组
                y: 时间序列对应的函数值，一个一维数组
                t_interp: 插值后的时间序列，一个一维数组
                y_interp: 插值后的时间序列对应的函数值，一个一维数组
            辅助参数:
                block: 图片显示后是否阻塞
                pause: 图片显示后的延迟时间,block为False时生效
                clear: 本次图片是否覆盖上次图片
                ion: 是否开启交互模式
            默认是“不阻塞+无等待+覆盖+无交互”，用于实时显示最新的插值结果
        """
        if ion: plt.ion()  # 开启交互模式
        if clear: plt.clf()
        # 绘制原始数据点
        plt.plot(t,y,'ro',label='original',markersize=3)
        # 绘制样条插值的结果
        plt.plot(t_interp,y_interp,'g-',label='interpolated',markersize=3)
        # 添加图例和标签
        plt.legend(loc='best')
        plt.xlabel('t')
        plt.ylabel('y')
        plt.title('Interpolation Result')
        # 显示图形
        plt.show(block=block)  # 非block并不会清除原来显示的图片，而是等下次图片来覆盖（前提是使用了plt.clf()）
        if pause > 0 and not block: plt.pause(pause)


if __name__ == '__main__':

    NODE_NAME = "follow_joint_trajectory_server"

    def loginfo(msg: str):
        rospy.loginfo("[{}] {}".format(NODE_NAME, msg))

    rospy.init_node(NODE_NAME)
    loginfo("Initializing {} node.".format(NODE_NAME))
    action_name = rospy.get_param('~action_name',default='arm_position_controller/follow_joint_trajectory')
    interpolation_type = rospy.get_param('~interpolation_type',default=5)
    MoveItAction(action_name,MoveItAction.CONTROL_MODE[1],interpolation=interpolation_type)

    rospy.loginfo("Ready to follow joint trajectory.")
    rospy.spin()