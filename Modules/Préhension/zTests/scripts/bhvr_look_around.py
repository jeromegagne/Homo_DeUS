#! /usr/bin/env python
from __future__ import division
import rospy
from copy import deepcopy
from std_msgs.msg import Bool
from head_control.scripts.headController import headController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import HomoDeUS_common_py.HomoDeUS_common_py as common  

BASE_MVMT_LIMIT = ""
MAX_REACHEABLE_RIGHT = -1.24
MAX_REACHEABLE_LEFT = 1.24
CENTER = 0.0
MAX_REACHEABLE_HEIGHT = 0.0
MAX_REACHEABLE_DOWN = -0.6
SECURITY_FACTOR = 0.90
MV_DURATION = 4


class lookAround:
    def __init__(self):
        self.head_controller = headController(rate=0.1, head_control_topic="head_command")
        self.output_pub = rospy.Publisher("bhvr_output_res_looking_around", Bool, queue_size=1)
        self.sub_detect = rospy.Subscriber("bhvr_output_detect_object", Bool, self.detect_objectCB, queue_size=1)
        self.stop_sending = False
        self.look_around()
    
    def look_around(self):
        trajectory = self.get_trajectory()
        index = 0
        rospy.logwarn("======================= lookAround ready =======================")
        while not rospy.is_shutdown():
            if not self.stop_sending:
                goal = trajectory[index]
                self.head_controller.goto_position(True,goal[0],goal[1],MV_DURATION)
                self.output_pub.publish(True)
                if index < len(trajectory) -1 :
                    index = index + 1   
                else:
                    index = 0
            rospy.sleep(MV_DURATION)

    def get_divided_trajectory(self,goal,divider):
        ratio = 1/divider
        goal_list = []
        for index in range(divider):
            new_goal = [goal[0] * (index+1)*ratio , goal[1] * (index+1)*ratio, MV_DURATION*(divider-index)]
            goal_list.append(new_goal)
        return goal_list

    def detect_objectCB(self,data):
        if data.data:
            self.stop_sending = True
            rospy.sleep(MV_DURATION)
            self.head_controller.stop_repeated_sending()
        else:
            self.stop_sending = False


    def get_trajectory(self):
        goal_ur_corner = [MAX_REACHEABLE_RIGHT*SECURITY_FACTOR, MAX_REACHEABLE_HEIGHT]
        goal_ul_corner = [MAX_REACHEABLE_LEFT*SECURITY_FACTOR, MAX_REACHEABLE_HEIGHT]
        goal_lr_corner = [MAX_REACHEABLE_RIGHT*SECURITY_FACTOR, MAX_REACHEABLE_DOWN]
        goal_ll_corner = [MAX_REACHEABLE_LEFT*SECURITY_FACTOR, MAX_REACHEABLE_DOWN]
        trajectory = [goal_ur_corner,goal_ul_corner,goal_lr_corner,goal_ll_corner]
        return trajectory
    
    def node_shutdown(self):
        rospy.loginfo("Closing bhvr_look_around")

if __name__ == "__main__":

    try:
        rospy.init_node('bhvr_look_around', anonymous=False)
        node = lookAround()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
