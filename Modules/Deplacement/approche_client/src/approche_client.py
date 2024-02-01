#!/usr/bin/env python

import rospy
import actionlib

from actionlib import GoalStatus
from custom_msgs.msg import FacePositions, FacePosition
from geometry_msgs.msg import Twist, Point, PointStamped, PoseStamped, Quaternion
from image_geometry import StereoCameraModel
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Bool, Empty

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
# tf.transformations.quaternion_from_euler(q, "sxyz")
def ToEulerAngles(q:Quaternion) -> Point:
  angles : Point = Point()
  #
  # roll (x-axis rotation)
  sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
  cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
  angles.x = np.arctan2(sinr_cosp, cosr_cosp)
  #
  # pitch (y-axis rotation)
  sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
  cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
  angles.y = 2 * np.arctan2(sinp, cosp) - np.pi / 2
  #
  # yaw (z-axis rotation)
  siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  angles.z = np.arctan2(siny_cosp, cosy_cosp)
  #
  return angles

class Navigator():
  def __init__(self) -> None:
    self.__client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    self.__client.wait_for_server()
    # self.__hz = 10
    # self.__rate = rospy.Rate(self.__hz)

    rospy.on_shutdown(self.__shutdown)

  def get_state(self) -> GoalStatus:
    return self.__client.get_state()

  def goto(self, x, y, yaw) -> None:
    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time(0)

    goal.target_pose.pose.position = Point(x, y, 0)
    quaternion = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    return self.__client.send_goal(goal, self.__goto_done_cb)

  def __goto_done_cb(self, state, result):
    print("result", result, 'state', state)

  def __shutdown(self):
    if self.get_state() < GoalStatus.SUCCEEDED:
      self.__client.cancel_goal()
    rospy.loginfo("Stop")

class ApproachClient():
  def __init__(self):
    self.bridge = CvBridge()

    # https://docs.ros.org/en/api/image_geometry/html/python/
    self.cameraModel = StereoCameraModel()  # TODO, utiliser cet objet a la place de tout faire tf 2D -> 3D a la main ...
    # TODO, Recuperer info sur topic (Actuellement, coder a la dure. A NE PAS FAIRE)
    # info camera intriseque : xtion/depth_registered/camera_info == /xtion/rgb/camera_info
    self.__camera_P = [522.1910329546544, 0.0, 320.5, -0.0, 0.0, 522.1910329546544, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    self.__camera_P = np.reshape(self.__camera_P, (3,4))
    self.__camera_Pinv = np.linalg.pinv(self.__camera_P)
    self.depth_image = None

    self.navigator = Navigator()
    self.approach_dist = 1.25
    self.tolerance = 0.15

    self.tf_listener = tf.TransformListener()

    # rospy.Subscriber('bhvr_input_image', Image, self.__camera_callback, queue_size=5) # JT a commente, branche directement sur la source
    rospy.Subscriber('xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)
    # # rospy.Subscriber('bhvr_input_image/xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)
    # # rospy.Subscriber('/bhvr_input_image/xtion/depth_registered/image_raw', Image, self.__camera_callback, queue_size=5)

    # # /bhvr_approach_client/bhvr_input_image/xtion/depth_registered/image_raw
    # # rospy.Subscriber('bhvr_input_face', FacePositions, self.__face_callback, queue_size=5)
    # # rospy.Subscriber('bhvr_input_face/proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)
    # # rospy.Subscriber('/bhvr_input_face/proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)
    rospy.Subscriber('proc_output_face_positions', FacePositions, self.__face_callback, queue_size=5)

    # self.pubObserver = rospy.Publisher('/bhvr_approach_client/obs_approach_client', Bool, queue_size=5) # JT a commente

  def __face_callback(self, detections):
    rospy.loginfo("approach client face CB")

    if self.depth_image is not None:
      min_y = detections.faces[0].y
      height= detections.faces[0].height
      max_y = detections.faces[0].y + height
      min_x = detections.faces[0].x
      width = detections.faces[0].width
      max_x = detections.faces[0].x + width

      face_depth_view = self.depth_image[min_y: max_y, min_x: max_x]
      # "<stdin>:43: RuntimeWarning: All-NaN slice encountered") -> JT, profondeur infini peut est une de causes
      face_dist = np.nanmean(face_depth_view)
      print(np.nanmin(face_depth_view), np.nanmean(face_depth_view))
      # ValueError: zero-size array to reduction operation fmin which has no identity

      print(face_dist, '>', (1+self.tolerance)*self.approach_dist,np.isnan(face_dist))
      if face_dist > (1+self.tolerance)*self.approach_dist and not np.isnan(face_dist):
        # # considering that the face is centered in the optical frame # AC
        # face_point = np.array([0, 0, face_dist])                     # JT
        dist_to_approach = face_dist - self.approach_dist
        # approach_point = np.array([0, 0, dist_to_approach]) # l'axe des z (profondeur) change d'orientation en fct de la tete du robot
        px = min_x + width/2    # central point x (pixel)
        py = min_y + height/2   # central point y (pixel)
        d  = dist_to_approach         # 
        point_2d = np.array([[px, py, 1]])
        print('point_2d', point_2d)
        (offset_x, offset_y,_,_) = self.__camera_Pinv @ (point_2d * d).T
        print('poind_3d', offset_x[0], offset_y[0], d)
        approach_point = np.array([dist_to_approach, -offset_x[0], -offset_y[0]]) # l'axe des x (profondeur) reste // au plan xy de /map
        yaw : float = np.arctan2(-offset_x[0], dist_to_approach)
        euler2quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        q : Quaternion = Quaternion(euler2quat[0],euler2quat[1],euler2quat[2],euler2quat[3])

        # frame_from = "/xtion_rgb_optical_frame"     # l'axe des z (profondeur) change d'orientation en fct de la tete du robot
        frame_from = "/head_1_link"                 # l'axe des x (profondeur) reste // au plan xy de /map

        pose = PoseStamped()
        pose.pose.position.x = approach_point[0]
        pose.pose.position.y = approach_point[1]
        pose.pose.position.z = approach_point[2]
        pose.pose.orientation = q
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = frame_from
        map_pose = self.tf_listener.transformPose("/map", pose)
        print(map_pose)
        print(ToEulerAngles(map_pose.pose.orientation).z)

        # self.navigator.goto(map_point.point.x, map_point.point.y, np.pi-np.arctan(map_point.point.x/map_point.point.y)) # AC
        # AC: for megagenial only
        print(self.navigator.get_state(), '==', GoalStatus.SUCCEEDED, 'or', GoalStatus.LOST)
        if self.navigator.get_state() == GoalStatus.SUCCEEDED or self.navigator.get_state() == GoalStatus.LOST:
          rospy.loginfo("approaching detected client")
          center_face_in_img = ToEulerAngles(map_pose.pose.orientation).z
          res = 'O' # input('Oui/Non?')
          if res == 'O':
            result = self.navigator.goto(map_pose.pose.position.x, map_pose.pose.position.y, center_face_in_img)
            rospy.loginfo("result is:" + str(result))
          # if result:                      # AC
          #   msg = Bool()                  # AC
          #   msg.data = True               # AC
          #   self.pubObserver.publish(msg) # AC

  def __camera_callback(self, image):
    self.__height = image.height
    self.__width = image.width
    self.depth_image = self.bridge.imgmsg_to_cv2(image, "passthrough")
    # rospy.loginfo("camera cb")      

if __name__ == "__main__":
  try:
    rospy.init_node('approach_client', anonymous=False)
    approachClient = ApproachClient()
    rospy.spin()
  except rospy.ROSInterruptException:
    print("except")
    pass
