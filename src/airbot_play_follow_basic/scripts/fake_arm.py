import rospy
from sensor_msgs.msg import JointState
from threading import Thread


class AirbotPlayFake(object):
    """Fake arm to publish joint states just using received joint cmd."""

    def __init__(self, cmd_topic: str, states_topic: str) -> None:
        self.cmd_suber = rospy.Subscriber(
            cmd_topic,
            JointState,
            self._joint_cmd_callback,
            queue_size=10,
        )
        self.states_puber = rospy.Publisher(states_topic, JointState, queue_size=1)
        print(f"Subscribe to {cmd_topic} and publish to {states_topic}.")
        self._joint_cmd_msg = JointState()
        self._use_default = False
        Thread(target=self._control_continue, daemon=True).start()

    def set_default_joint_states(
        self,
        joint_names: list,
        joint_positions: list,
        joint_velocities: list,
        joint_efforts: list,
    ):
        """Set default joint states."""
        self._use_default = True
        self._joint_cmd_msg.name = tuple(joint_names)
        self._joint_cmd_msg.position = tuple(joint_positions)
        self._joint_cmd_msg.velocity = tuple(joint_velocities)
        self._joint_cmd_msg.effort = tuple(joint_efforts)
        print("Set default joint states.")
        print("Joint names: ", self._joint_cmd_msg.name)
        print("Joint positions: ", self._joint_cmd_msg.position)
        print("Joint velocities: ", self._joint_cmd_msg.velocity)
        print("Joint efforts: ", self._joint_cmd_msg.effort)

    def _joint_cmd_callback(self, msg: JointState):
        if self._use_default:
            # 关节名称使用默认的
            self._joint_cmd_msg.position = msg.position
            self._joint_cmd_msg.velocity = msg.velocity
            self._joint_cmd_msg.effort = msg.effort
        else:
            self._joint_cmd_msg = msg

    def _control_continue(self):
        rospy.loginfo("Waiting for the first joint cmd...")
        while self._joint_cmd_msg == JointState():
            rospy.sleep(0.5)
        rospy.loginfo("Start to publish joint states...")
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self._joint_cmd_msg.header.stamp = rospy.Time.now()
            self.states_puber.publish(self._joint_cmd_msg)
            rate.sleep()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("AIRbotPlay PickPlace param set.")
    parser.add_argument(
        "-jct",
        "--joint_cmd_topic",
        type=str,
        default="/airbot_play/joint_cmd",
        help="topic name of joint cmd, the type should be sensor_msgs/JointState",
    )
    parser.add_argument(
        "-jst",
        "--joint_states_topic",
        type=str,
        default="/airbot_play/joint_states",
        help="topic name of joint states, the type should be sensor_msgs/JointState",
    )
    parser.add_argument(
        "-nsd",
        "--not_set_default",
        action="store_true",
        help="not set default joint states, so you should publish joint cmd first",
    )
    parser.add_argument(
        "-jn",
        "--joint_names",
        type=str,
        nargs="+",
        default=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        help="set default joint names",
    )
    parser.add_argument(
        "-jp",
        "--joint_positions",
        type=float,
        nargs="+",
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="set default joint positions",
    )

    parser.add_argument(
        "-jv",
        "--joint_velocities",
        type=float,
        nargs="+",
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="set default joint velocities",
    )

    parser.add_argument(
        "-je",
        "--joint_efforts",
        type=float,
        nargs="+",
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="set default joint efforts",
    )

    args, unknown = parser.parse_known_args()

    rospy.init_node("fake_arm", anonymous=True)
    faker = AirbotPlayFake(
        cmd_topic=args.joint_cmd_topic, states_topic=args.joint_states_topic
    )
    if not args.not_set_default:
        faker.set_default_joint_states(
            args.joint_names,
            args.joint_positions,
            args.joint_velocities,
            args.joint_efforts,
        )
    rospy.spin()
