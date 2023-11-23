#!/usr/bin/env python3

from moveit_basic_controller import MoveItBasicController
from airbot_play_ik_service.srv import airbot_play_ik, airbot_play_ikRequest, airbot_play_ikResponse
import rospy


class IkControlServer(MoveItBasicController):
    """ 不允许多线程调用服务 """
    def __init__(self, name, robot_description="robot_description", ns="", wait_for_servers=5):
        super().__init__(name, robot_description, ns, wait_for_servers)
        self.ik_server = rospy.Service('/airbot_play/ik_service', airbot_play_ik, self._handle_ik)

    def _handle_ik(self, req:airbot_play_ikRequest):
        response = airbot_play_ikResponse()
        attempts_ik = 20
        attempts_plan = 20
        success = 0
        while attempts_ik > 0:
            try:
                attempts_ik -= 1
                self.set_pose_target(req.target_pose.pose)
            except: continue
            else:
                attempts_ik = 20
                attempts_plan -= 1
                # `go()` returns a boolean indicating whether the planning and execution was successful.
                if self.go(wait=True):
                    # Calling `stop()` ensures that there is no residual movement
                    self.stop()
                    # It is always good to clear your targets after planning with poses.
                    # Note: there is no equivalent function for clear_joint_value_targets().
                    self.clear_pose_targets()
                    success = 1
                    break
                elif attempts_plan <= 0:
                    rospy.logerr("Planning failed.")
                    break
                else: continue
        else:
            rospy.logerr("IK failed.")
        response.result = success
        return response


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser("AIRbotPlay PickPlace param set.")
    parser.add_argument('-gn','--group_name',type=str,default='airbot_play_arm',help='target move group name')
    parser.add_argument('-rd','--robot_description',type=str,default='/airbot_play/robot_description',help='name of robot description')
    parser.add_argument('-ns','--name_space',type=str,default='/airbot_play',help='namespace of moveit')
    parser.add_argument('-t','--test',action='store_true',help='test mode')
    args, unknown = parser.parse_known_args()

    NODE_NAME = 'airbot_play_moveit_ik_control_server'
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Initializing {} Node.".format(NODE_NAME))

    mc = IkControlServer(args.group_name,args.robot_description,args.name_space)
    if args.test:
        client = rospy.ServiceProxy('/airbot_play/ik_service', airbot_play_ik)
        client.wait_for_service(timeout=5)
        req = airbot_play_ikRequest()
        req.target_pose = mc.get_current_pose()
        req.target_pose.pose.position.z += 0.1
        res:airbot_play_ikResponse = client.call(req)
        print("ik响应为：",res.result)
    rospy.spin()
