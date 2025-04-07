import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from dataclasses import dataclass, asdict
from airbot_py.arm import AIRBOTPlay, RobotMode


@dataclass
class AIRBOTPlayCfg:
    url: str = "localhost"
    port: int = 50051


class AirbotPlayRosInterface:
    def __init__(self, config: AIRBOTPlayCfg):
        self.robot = AIRBOTPlay(**asdict(config))
        assert self.robot.connect(), "Failed to connect to robot"
        assert self.robot.connect(), "Failed to connect to robot"
        while not self.robot._feedback_jointstates:
            rospy.loginfo("Waiting for robot feedback...")
            rospy.sleep(0.5)
        rospy.loginfo("Robot feedback received.")
        assert self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.eef_factor = 0.07
        self.joint_state_puber = rospy.Publisher(
            "/airbot_play/joint_states", JointState, queue_size=5
        )
        self.gripper_position_puber = rospy.Publisher(
            "/airbot_play/gripper/position", Float64, queue_size=5
        )
        self.arm_joint_cmd_suber = rospy.Subscriber(
            "/airbot_play/joint_cmd", JointState, self.arm_cmd_cb
        )
        self.eef_joint_cmd_suber = rospy.Subscriber(
            "/airbot_play/end_effector/command", JointState, self.eef_cmd_cb
        )
        self.gripper_bool_cmd_suber = rospy.Subscriber(
            "/airbot_play/gripper/state_cmd", Bool, self.gripper_bool_cmd_cb
        )
        self.gripper_float_cmd_suber = rospy.Subscriber(
            "/airbot_play/gripper/set_position", Float64, self.gripper_float_cmd_cb
        )

        self.joint_state = JointState()
        self.joint_state.name = [f"joint{i+1}" for i in range(6)] + ["endleft", "endright"]
        self.js_timer = rospy.Timer(rospy.Duration(1 / 200), self.pub_joint_states)

    def arm_cmd_cb(self, msg: JointState):
        self.robot.servo_joint_pos(msg.position)

    def pub_joint_states(self, event):
        qpos = self.robot.get_joint_pos()
        epos = (self.eef_factor - self.robot.get_eef_pos()[0]) / 2
        evel = 0.0
        eeef = self.robot.get_eef_eff()[0]
        self.joint_state.position = qpos + [0.01, -0.01]
        self.joint_state.velocity = self.robot.get_joint_vel() + [evel] * 2
        self.joint_state.effort = self.robot.get_joint_eff() + [eeef] * 2
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_puber.publish(self.joint_state)
        self.gripper_position_puber.publish(Float64(data=epos / self.eef_factor))


if __name__ == "__main__":
    rospy.init_node("airbot_play_ros_interface", anonymous=True)
    airbot_player = AirbotPlayRosInterface(AIRBOTPlayCfg(url=rospy.get_param("~url", "localhost"), port=int(rospy.get_param("~port", 50051))))
    rospy.spin()
    airbot_player.robot.disconnect()