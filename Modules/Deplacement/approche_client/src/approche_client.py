#!/usr/bin/env python

import rospy
from homodeus_precomp import *

from custom_msgs.msg import FacePositions, FacePosition
from image_geometry import StereoCameraModel
from math import sqrt, atan2, pi
from NavSelector import *
import numpy as np

class ApproachClient():

  def __init__(self) -> None :
    initRosNode("approche_client")
    
    self.bridge = CvBridge()
    self.cameraModel = StereoCameraModel()  # TODO, utiliser cet objet a la place de tout faire tf 2D -> 3D a la main ...
    self.__camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    self.__camera_P = np.reshape(self.__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(self.__camera_P)
    self.depth_image = None

    self.navigator = NavSelector()
    self.approach_dist = 1.25
    self.tolerance = 0.15

    self.tf_listener = tf.TransformListener()
    self.__camera_sub = rospy.Subscriber('xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)
    self.__face_sub = rospy.Subscriber('proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)
    rospy.loginfo("approach client initialized")
    rospy.on_shutdown(self.__node_shutdown)


  def __face_callback(self, detections : FacePositions) -> None :
    rospy.loginfo("approach client face CB")

    if self.depth_image is not None:
      faces = detections.faces[0]
      height= faces.height
      width = faces.width
      min_y, max_y = faces.y, faces.y + height
      min_x, max_x = faces.x, faces.x + width

      face_depth_view = self.depth_image[min_y: max_y, min_x: max_x]
      face_dist = np.nanmean(face_depth_view)

      print(np.nanmin(face_depth_view), np.nanmean(face_depth_view))
      print(face_dist, '>', (1+self.tolerance)*self.approach_dist,np.isnan(face_dist))

      if face_dist > (1 + self.tolerance) * self.approach_dist and not np.isnan(face_dist):
        dist_to_approach = face_dist - self.approach_dist

        px = min_x + width / 2       # central point x (pixel)
        py = min_y + height / 2      # central point y (pixel)
        point_2d = np.array([[px, py, 1]])
        (offset_x, offset_y,_,_) = self.__camera_Pinv @ (point_2d * dist_to_approach).T

        print('point_2d', point_2d)
        print('poind_3d', offset_x[0], offset_y[0], dist_to_approach)
        
        yaw : float = atan2(-offset_x[0], dist_to_approach)
        euler2quat = quaternion_from_euler(0, 0, yaw)
        quaternion : Quaternion = Quaternion(euler2quat[0], euler2quat[1], euler2quat[2], euler2quat[3])
        approach_point : Point = Point(dist_to_approach, -offset_x[0], -offset_y[0]) # l'axe des x (profondeur) reste // au plan xy de /map

        frame_from = "/head_1_link"  # l'axe des x (profondeur) reste // au plan xy de /map

        pose = PoseStamped()

        pose.pose.position = approach_point
        pose.pose.orientation = quaternion
        
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = frame_from
        
        map_pose = self.tf_listener.transformPose("/map", pose)
        
        print(map_pose)
        print(quarternion2euler(map_pose.pose.orientation).z)
        print(self.navigator.GetState(), '==', GoalStatus.SUCCEEDED, 'or', GoalStatus.LOST)

        if self.navigator.GetState() == GoalStatus.SUCCEEDED or self.navigator.GetState() == GoalStatus.LOST:
          rospy.loginfo("approaching detected client")
          center_face_in_img = quarternion2euler(map_pose.pose.orientation).z
          res = 'O' # input('Oui/Non?')
          if res == 'O':
            result = self.navigator.AddGoal(map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z, center_face_in_img,  "Approche")
            self.navigator.SendGoal()
            rospy.loginfo("result is:" + str(result))

  def __camera_callback(self, image : Image) -> None :
    self.__height = image.height
    self.__width = image.width
    self.depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough") 

  def __node_shutdown(self) -> None :
    self.__camera_sub.unregister()
    self.__face_sub.unregister()
    rospy.loginfo("approach client shutdown")