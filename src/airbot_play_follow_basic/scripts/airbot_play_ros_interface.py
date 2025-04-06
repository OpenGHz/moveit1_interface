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
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)

        while not self.robot._feedback_jointstates:
            rospy.loginfo("Waiting for robot feedback...")
            rospy.sleep(0.5)
        rospy.loginfo("Robot feedback received.")

        self.eef_factor = 0.07

        self.tar_jq = np.zeros(7)

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

        self.joint_state.position = self.robot.get_joint_pos() + [self.robot.get_eef_pos()]
        self.joint_state.velocity = self.robot.get_joint_vel() + [self.robot.get_eef_vel()]
        self.joint_state.effort = self.robot.get_joint_eff() + [self.robot.get_eef_eff()]

        self.js_timer = rospy.Timer(rospy.Duration(1 / 200), self.pub_joint_states)

    def arm_cmd_cb(self, msg: JointState):
        self.robot.servo_joint_pos(msg.position)

    def eef_cmd_cb(self, msg: JointState):
        self.tar_jq[6] = msg.position[0] * self.eef_factor

    def gripper_bool_cmd_cb(self, msg: Bool):
        self.tar_jq[6] = 0.0 if msg.data else self.eef_factor

    def gripper_float_cmd_cb(self, msg: Float64):
        self.tar_jq[6] = msg.data * self.eef_factor

    def pub_joint_states(self, event):
        qpos = self.robot.get_joint_pos()
        epos = (self.eef_factor - self.robot.get_eef_pos()) / 2
        evel = self.robot.get_eef_vel()
        eeef = self.robot.get_eef_eff()
        self.joint_state.position = qpos + [epos, -epos]
        self.joint_state.velocity = self.robot.get_joint_pos() + [evel] * 2
        self.joint_state.effort = self.robot.get_joint_eff() + [eeef] * 2
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_puber.publish(self.joint_state)
        self.gripper_position_puber.publish(Float64(data=epos / self.eef_factor))


if __name__ == "__main__":
    rospy.init_node("airbot_play_ros_interface", anonymous=True)
    airbot_player = AirbotPlayRosInterface(AIRBOTPlayCfg(url=rospy.get_param("~url", "localhost"), port=int(rospy.get_param("~port", 50051))))
    rospy.spin()
