import rospy
from airbot_play_ik_service.srv import airbot_play_ik, airbot_play_ikRequest, airbot_play_ikResponse
from geometry_msgs.msg import PoseStamped

rospy.init_node("position_ik_control_example")

client = rospy.ServiceProxy(f"airbot_play/ik_service", airbot_play_ik)
client.wait_for_service(timeout=5)
req = airbot_play_ikRequest()
req.target_pose = PoseStamped()
req.target_pose.pose.position.x = 0.1449
req.target_pose.pose.position.y = 0.0
req.target_pose.pose.position.z = 0.21457 + 0.05
res: airbot_play_ikResponse = client.call(req)

print("ik response is：", res.result, ", 1 means success; 0 means fail")
joint_target = rospy.get_param(f"airbot_play/joint_target", "None")
print("joint target is：", joint_target)
