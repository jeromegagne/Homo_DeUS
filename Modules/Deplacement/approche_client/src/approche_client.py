#!/usr/bin/env python3


import rospy

from custom_msgs.msg import FacePositions, FacePosition
from image_geometry import StereoCameraModel

# from base_navigation.scripts.NavSelector import NavSelector
# from base_navigation.scripts.homodeus_precomp import *
from Navigation import NavSelector, homodeus_precomp

import numpy as np


class ApproachClient():
  def __init__(self):
    initRosNode("approche_client")

    self.__bridge = CvBridge()

    # https://docs.ros.org/en/api/image_geometry/html/python/
    self.__cameraModel = StereoCameraModel()  # TODO, utiliser cet objet a la place de tout faire tf 2D -> 3D a la main ...
    # TODO, Recuperer info sur topic (Actuellement, coder a la dure. A NE PAS FAIRE)
    # info camera intriseque : xtion/depth_registered/camera_info == /xtion/rgb/camera_info
    self.__camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    self.__camera_P = np.reshape(self.__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(self.__camera_P)
    self.__depth_image = None

    self.__navigator = NavSelector()
    self.__approach_dist = 1.375
    self.__tolerance = 0.15

    self.__tf_listener = tf.TransformListener()

    # rospy.Subscriber('bhvr_input_image', Image, self.__camera_callback, queue_size=5) # JT a commente, branche directement sur la source
    self.__camera_sub = rospy.Subscriber('xtion/depth_registered/image_raw', Float32MultiArray, self.__camera_callback, queue_size=5)
    # self.__camera_sub = rospy.Subscriber('xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)

    # rospy.Subscriber('bhvr_input_face', FacePositions, self.__face_callback, queue_size=5)
    self.__face_sub =rospy.Subscriber('proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)

    rospy.loginfo("approach client initialized")
    rospy.on_shutdown(self.__node_shutdown)


  def __face_callback(self, detections : FacePositions) -> None:
    rospy.loginfo("approach client face CB")

    if self.__depth_image is not None:
      faces: FacePosition = detections.faces[0]
      height: float       = faces.height
      width: float        = faces.width
      min_y, max_y        = faces.y, faces.y + height
      min_x, max_x        = faces.x, faces.x + width

      face_depth_view   = self.__depth_image[min_y: max_y, min_x: max_x]
      face_dist: float  = np.nanmean(face_depth_view)
      print(np.nanmin(face_depth_view), np.nanmean(face_depth_view))

      print(face_dist, '>', (1+self.__tolerance)*self.__approach_dist,np.isnan(face_dist))
      if face_dist > (1+self.__tolerance)*self.__approach_dist and not np.isnan(face_dist):
        dist_to_approach = face_dist - self.__approach_dist

        px = min_x + width / 2       # central point x (pixel)
        py = min_y + height / 2      # central point y (pixel)
        point_2d = np.array([[px, py, 1]])
        print('point_2d', point_2d)

        (offset_x, offset_y,_,_) = self.__camera_Pinv @ (point_2d * dist_to_approach).T
        print('poind_3d', offset_x[0], offset_y[0], dist_to_approach)

        approach_point : Point = Point(dist_to_approach, -offset_x[0], -offset_y[0]) # l'axe des x (profondeur) reste // au plan xy de /map
        yaw : float = atan2(-offset_x[0], dist_to_approach)
        quaternion : Quaternion = quaternion_from_euler(0, 0, yaw)

        # frame_from = "/xtion_rgb_optical_frame"     # l'axe des z (profondeur) change d'orientation en fct de la tete du robot
        frame_from = "/head_1_link"                 # l'axe des x (profondeur) reste // au plan xy de /map

        pose = PoseStamped()
        pose.pose.position    = approach_point
        pose.pose.orientation = quaternion
        pose.header.stamp     = rospy.Time(0)
        pose.header.frame_id  = frame_from
        map_pose              = self.__tf_listener.transformPose("/map", pose)
        print(map_pose)
        print(quarternion2euler(map_pose.pose.orientation).z)

        print(self.__navigator.GetState(), '==', GoalStatus.SUCCEEDED, 'or', GoalStatus.LOST)
        if self.__navigator.GetState() == GoalStatus.SUCCEEDED or \
           self.__navigator.GetState() == GoalStatus.LOST:
          rospy.loginfo("approaching detected client")

          center_face_in_img = quarternion2euler(map_pose.pose.orientation).z

          res = 'O' # input('Oui/Non?')
          if res == 'O':
            result = self.__navigator.AddGoal(map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z, center_face_in_img,  "Approche")
            self.__navigator.SendGoal()
            rospy.loginfo("result is:" + str(result))


  # Vrai fct «callback» pour l'image de profondeur
  def __camera_callback(self, image : Image) -> None:
    self.__height = image.height
    self.__width = image.width
    self.__depth_image = self.__bridge.imgmsg_to_cv2(image, "passthrough")


  # Fct «callback» utilisee pour les rosbags & PseudoDepthImage
  def __camera_callback(self, image : Float32MultiArray) -> None:
    self.__height = image.layout.dim[0].size
    self.__width = image.layout.dim[1].size
    self.__depth_image = np.reshape(image.data, (self.__height, self.__width))


  def __node_shutdown(self) -> None:
    self.__camera_sub.unregister()
    self.__face_sub.unregister()

    rospy.loginfo("approach client shutdown")
