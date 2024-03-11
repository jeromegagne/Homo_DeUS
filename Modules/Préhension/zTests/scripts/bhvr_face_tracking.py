#! /usr/bin/env python
import time

import rospy
import roslib
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
from sensor_msgs.msg import CameraInfo
import numpy as np

from custom_msgs.msg import FacePosition
from custom_msgs.msg import FacePositions

import HomoDeUS_common_py.HomoDeUS_common_py as common  

class FaceTracking:
    def __init__(self, mode):
        rospy.loginfo("Face tracking constructing")

        rospy.Subscriber('/proc_output_face_positions', FacePositions, self._head_callback, queue_size=5)

        if mode == "remote":
            camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        else:
            camera_info = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
    
        self.img_height = camera_info.height
        self.img_width = camera_info.width
        self.pub = rospy.Publisher('tiago_head_controller', PoseStamped, queue_size=5)
        self.pubObserver = rospy.Publisher('/Face_tracking_observer', Bool, queue_size=5)

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.threshold = 55

        self.pid_x = common.PID(self.img_center_x, K_P=-0.0010, K_I=-0.0001, K_D=-0.00005)
        self.pid_y = common.PID(self.img_center_y, K_P=-0.0010, K_I=-0.0001, K_D=-0.00005)

    def _head_callback(self, detections):
        rospy.loginfo("face detected face tracking")
        main_face_x = 0
        main_face_y = 0
        main_face_dist = 1000000
        boolMsg = Bool()

        # Find the closest face to the image center (main face)
        for face in detections.faces:
            face_x, face_y = self._get_face_center_position(face)
            face_dist = self._distance_from_img_center(face_x, face_y)

            if (face_dist < main_face_dist):
                main_face_x = face_x
                main_face_y = face_y
                main_face_dist = face_dist

        # If the main face is inside the limit, don't move the head
        if main_face_dist < self.threshold:
            boolMsg.data = True
            self.pubObserver.publish(boolMsg)
            return
        else:
            boolMsg.data = False
            self.pubObserver.publish(boolMsg)

        x = self.pid_x.get_next_command(main_face_x)
        y = self.pid_y.get_next_command(main_face_y)

        poseStamped = PoseStamped()

        poseStamped.pose.position.x = x
        poseStamped.pose.position.y = y

        #rospy.loginfo("sent pose")

        self.pub.publish(poseStamped)

    def _distance_from_img_center(self, x, y):
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y - y)**2)

    def _get_face_center_position(self, face):
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)

        return x, y

if __name__ == "__main__":

    try:
        rospy.init_node('faceTracking', anonymous=False)
        mode = rospy.get_param('camera_mode', 'simul')
        print("Face tracking camera mode: " + mode)
        faceTracking = FaceTracking(mode)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
