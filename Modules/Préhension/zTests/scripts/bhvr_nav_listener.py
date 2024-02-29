#!/usr/bin/env python2
import rospy
import json

from base_navigation.scripts.navigator import Navigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from custom_msgs.msg import GoToResult
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class HBBA_nav_listener(Navigator):
    def __init__(self):
        Navigator.__init__(self)

        self.registerLandmark("testPoint", 1, 0, 0)

        self.registerLandmark("origin")

        self.result_pub = rospy.Publisher("bhvr_output_res_nav_result", GoToResult, queue_size=5)
        self.add_landmark_result_pub = rospy.Publisher("bhvr_output_nav_added_landmark", String, queue_size=5)
        self.curLandmark = ""
        rospy.loginfo("bhvr_nav init")

    def gotoCallback(self, data):
        self.cancelAllGoto()
        self.curLandmark = ""
        self.goto(data.pose.position.x, data.pose.position.y, data.pose.orientation.w)

    def gotoLandmarkCallback(self, data):
        self.cancelAllGoto()
        self.curLandmark = data.data
        self.gotoLandmark(data.data)


    def doneCB(self, status):
        # Publish out the result of the goto action, along with current coordinates, called by parent class
        rospy.loginfo("In CB launched by parent navigator")
        result = GoToResult()
        curpose = self.getCurPose()
        result.result = status
        result.x = curpose.position.x
        result.y = curpose.position.y
        result.t = euler_from_quaternion((curpose.orientation.x, curpose.orientation.y, curpose.orientation.z, curpose.orientation.w))[2]
        result.landmark = self.curLandmark
        rospy.loginfo("Publishing result: ")
        rospy.loginfo(result)
        self.result_pub.publish(result)

    def addLandmarkCB(self, landmark):
        landmark_to_add = landmark.data
        landmark_name  = landmark_to_add.header.frame_id
        landmark_x = landmark_to_add.pose.position.x
        landmark_y = landmark_to_add.pose.position.y
        landmark_yaw = euler_from_quaternion((landmark_to_add.pose.orientation.x, landmark_to_add.pose.orientation.y, landmark_to_add.pose.orientation.z, landmark_to_add.pose.orientation.w))[2]
        if (x is 0 and y is 0 and w is 0):
            self.registerLandmark(landmark_name)
        else:
            self.registerLandmark(landmark_name, landmark_x, landmark_y, landmark_yaw)
        self.add_landmark_result_pub.publish(landmark.data)

    def listenGoto(self):
        rospy.Subscriber('bhvr_input_goal_nav_goal', PoseStamped, self.gotoCallback)

    def listenGotoLandmark(self):
        rospy.Subscriber('bhvr_input_goal_landmark_nav_goal', String, self.gotoLandmarkCallback)

    def listenAddLandmark(self):
        rospy.Subscriber('bhvr_input_goal_add_landmark', String, self.addLandmarkCB)

if __name__ == '__main__':
    try:
        rospy.init_node('HBBA_nav_listener', anonymous=True)
        listener = HBBA_nav_listener()
        listener.listenGoto()
        listener.listenGotoLandmark()
        listener.listenAddLandmark()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass