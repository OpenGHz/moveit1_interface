import rospy
from threading import Thread
from sensor_msgs.msg import JointState


class RosAgent(object):
    """
    通过rostopic接收state，通过rostopic发布action；通过接口获取state、获取并设置action；
    action的名称等基本数据通过获取state的第一条消息来设置；
    """

    def __init__(self, state_topic, action_topic, rate=50):
        self.state_topic = state_topic
        self.action_topic = action_topic
        self.rate = rospy.Rate(rate)
        self._state_msg = JointState()
        self.has_action = False
        self.state_suber = rospy.Subscriber(
            self.state_topic, JointState, self._state_callback
        )
        self._action_puber = rospy.Publisher(
            self.action_topic, JointState, queue_size=1
        )
        self._action_msg = rospy.wait_for_message(self.state_topic, JointState, timeout=2)
        # self._action_msg = JointState()
        if self._action_msg.header.frame_id == "":
            self._action_msg.header.frame_id = "action"
        # safe to judge with lenth
        position_lenth = len(self._action_msg.position)
        velocity_lenth = len(self._action_msg.velocity)
        effort_lenth = len(self._action_msg.effort)
        if effort_lenth == 0:
            self._action_msg.effort = [0] * position_lenth
            self._state_msg.effort = [0] * position_lenth
        if velocity_lenth == 0:
            self._action_msg.velocity = [0] * position_lenth
            self._state_msg.velocity = [0] * position_lenth
        Thread(target=self._publish_action, daemon=True).start()

    def _state_callback(self, msg: JointState):
        self._state_msg.position = msg.position
        velocity_lenth = len(msg.velocity)
        effort_lenth = len(msg.effort)
        if velocity_lenth != 0:
            self._state_msg.velocity = msg.velocity
        if effort_lenth != 0:
            self._state_msg.effort = msg.effort

    def _publish_action(self):
        while not rospy.is_shutdown():
            try:
                self._action_puber.publish(self._action_msg)
            except rospy.ROSException:
                pass
            self.rate.sleep()

    def get_current_state(self):
        return self._state_msg

    def get_current_positions(self):
        return self._state_msg.position

    def get_current_velocities(self):
        return self._state_msg.velocity

    def get_current_efforts(self):
        return self._state_msg.effort

    def get_target_action(self):
        return self._action_msg

    def get_target_positions(self):
        return self._action_msg.position

    def get_target_velocities(self):
        return self._action_msg.velocity

    def get_target_efforts(self):
        return self._action_msg.effort

    def set_target_action(self, action: JointState):
        self._action_msg = action

    def set_target_positions(self, position):
        self._action_msg.position = position

    def set_target_velocities(self, velocity):
        self._action_msg.velocity = velocity

    def set_target_efforts(self, effort):
        self._action_msg.effort = effort


if __name__ == "__main__":
    rospy.init_node("ros_agent_test")
    master_left_gripper = RosAgent(
        "/left_master_gripper_states", "/left_master_gripper_cmd", rate=50
    )
    master_right_gripper = RosAgent(
        "/right_master_gripper_states", "/right_master_gripper_cmd", rate=50
    )
    print("get_current_state")
    print(master_left_gripper.get_current_state())
    master_left_gripper.set_target_positions([1])
    master_left_gripper.set_target_velocities([2])
    master_left_gripper.set_target_efforts([3])
    print("get_target_action")
    print(master_left_gripper.get_target_action())
    rospy.sleep(1)
    print("get_current_state")
    print(master_left_gripper.get_current_state())

    master_left_arm = RosAgent("/joint_states", "/left_master_arm_cmd", rate=50)
    master_right_arm = RosAgent("/joint_states", "/right_master_arm_cmd", rate=50)
    print("get_current_state")
    print(master_left_arm.get_current_state())
    master_left_arm.set_target_positions([1, 2, 3, 4, 5, 6])
    print("get_target_action")
    print(master_left_arm.get_target_action())
    rospy.sleep(0.2)
    print("get_current_state")
    print(master_left_arm.get_current_state())
