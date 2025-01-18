import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from airbot_py.airbot_play import AirbotPlay


if __name__ == "__main__":
    rospy.init_node("airbot_play_ros_interface", anonymous=True)
    ip = rospy.get_param("~ip", "localhost")
    port = int(rospy.get_param("~port", 50051))
    rospy.loginfo(f"Initializing airbot_play node: ip: {ip} port: {port}.")

    airbot_play = AirbotPlay(ip, port)

    def arm_joint_cmd_callback(data: JointState):
        airbot_play.set_target_joint_q(data.position)

    def eef_joint_cmd_callback(data: Float64):
        airbot_play.set_target_end(data.data)

    arm_states_puber = rospy.Publisher(
        "/airbot_play/joint_states", JointState, queue_size=10
    )
    eef_states_puber = rospy.Publisher(
        "/airbot_play/gripper/position", Float64, queue_size=10
    )

    arm_joint_cmd_suber = rospy.Subscriber(
        "/airbot_play/set_target_joint_q",
        JointState,
        arm_joint_cmd_callback,
        queue_size=10,
    )
    eef_joint_cmd_suber = rospy.Subscriber(
        "/airbot_play/gripper/set_position",
        Float64,
        arm_joint_cmd_callback,
        queue_size=10,
    )

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        arm_states_puber.publish(JointState(position=airbot_play.get_current_joint_q()))
        eef_states_puber.publish(Float64(data=airbot_play.get_current_end()))
        rate.sleep()
