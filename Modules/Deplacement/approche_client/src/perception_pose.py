#!/usr/bin/env python

# https://wiki.ros.org/ROS/Tutorials/CreatingPackage
# https://docs.ros.org/en/melodic/api/catkin/html/howto/format2/installing_python.html

import rospy
from rospy import Publisher, Rate, Subscriber
from geometry_msgs.msg import Point, Pose , PoseWithCovarianceStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry

from math import atan2, pi, sqrt
from numpy import around, mean, square


# https://github.com/introlab/hbba_lite/blob/main/README.md
# https://wiki.ros.org/amcl

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
# topic : /mobile_base_controller/odom (nav_msgs/Odometry) 

# ################################################################
# Noeud Perception : AMCL (Estimation de la pose du robot + Sa confiance)
#   Possiblite de 'remap' le noeud
#   Entrees (capteur) [topic]
#     - pose initiale (texte) [/initialpose]
#     - numerisation env. 2D (lidar) (/scan_raw)
#     - grille de la carte (image) [/map]
#   Sorties (perception) [topic]
#     - estimation de la pose () [/amcl_pose/pose/pose]
#     ou calcul de la pose () [/mobile_base_controller/odom/pose]
#     - sa confiance () [/amcl_pose/pose/covariance]


def quarternion2euler(q: Quaternion) -> Vector3:
  euler : Vector3 = Vector3()

  # roll (x-axis rotation)
  sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
  cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
  euler.x = atan2(sinr_cosp, cosr_cosp)

  # pitch (y-axis rotation)
  sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
  cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
  euler.y = 2 * atan2(sinp, cosp) - pi / 2

  # yaw (z-axis rotation)
  siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  euler.z = atan2(siny_cosp, cosy_cosp)

  return euler


class RobotPose():
  def __init__(self) -> None:
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'robot_pose' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_pose', anonymous=True)

    self.__amcl_pose: Vector3
    self.__odom_pose: Vector3

    # Subscriber
    self.__amcl_pose_sub: Subscriber = Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.__amcl_pose_subscriber)
    self.__odom_pose_sub: Subscriber = Subscriber("/mobile_base_controller/odom", Odometry, self.__odom_pose_subscriber)

    # Publisher
    self.__rate: Rate = Rate(10) # 10hz
    self.__robot_pose_pub: Publisher = Publisher("/homodeus/perceptions/robot_pose", Vector3, queue_size=1)

    rospy.loginfo("Robot Pose initialized")
    self.__robot_pose_publisher()
    rospy.on_shutdown(self.__node_shutdown)


  def __amcl_pose_subscriber(self, pose_wcs: PoseWithCovarianceStamped) -> None:
    pose:       Pose      = pose_wcs.pose.pose
    position:   Point     = pose.position
    covar_mat:  float[36] = pose_wcs.pose.covariance
    # mat:        float[36] =  around(covar_mat, 4)

    # mean_mat:   float[36] = mean(covar_mat)                 # > 0.0025 -> lost, < 0.0005 -> A+
    msqrt:      float[36] = mean(square(covar_mat))   # > 0.00005 -> lost, < 0.000001 -> A+
    # max_value:    float     = abs(max(covar_mat, key=abs))
    # mat_sum:      float     = sum([i for i in covar_mat])
    
    yaw   = quarternion2euler(pose.orientation).z
    self.__amcl_pose = Vector3(position.x, position.y, yaw)


    # rospy.loginfo(rospy.get_caller_id() + " AMCL - x: %.4f, y: %.4f, yaw: %.4f; mean: %.4f", position.x, position.y, yaw, msqrt)


  def __odom_pose_subscriber(self, odom: Odometry) -> None:
    pose:       Pose      = odom.pose.pose
    position:   Point     = pose.position
    covar_mat:  float[36] = odom.pose.covariance
    # mat:        float[36] =  around(covar_mat, 4)

    # mean_mat:   float[36] = mean(covar_mat)                 # > 0.0025 -> lost, < 0.0005 -> A+
    msqrt:      float[36] = mean(square(covar_mat))         # > 0.00005 -> lost, < 0.000001 -> A+
    # max_value:    float     = abs(max(covar_mat, key=abs))
    # mat_sum:      float     = sum([i for i in covar_mat])
    
    yaw   = quarternion2euler(pose.orientation).z
    self.__odom_pose = Vector3(position.x, position.y, yaw)
    # self.__robot_pose_pub.publish(self.__odom_pose)

    rospy.loginfo(rospy.get_caller_id() + " ODOM - x: %.4f, y: %.4f, yaw: %.4f; mean: %.4f", position.x, position.y, yaw, msqrt)


  def __robot_pose_publisher(self) -> None:
    self.__odom_pose = Vector3(0,0,0)

    while not rospy.is_shutdown():
      self.__robot_pose_pub.publish(self.__odom_pose)
      self.__rate.sleep()


  def __node_shutdown(self) -> None:
    self.__amcl_pose_sub.unregister()
    self.__odom_pose_sub.unregister()

    rospy.loginfo("Perception Robot Pose - Shutdown")


def main() -> None:
  robot_pose = RobotPose()
  rospy.spin()
 

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException as ROSie:
    rospy.loginfo("Perception Robot Pose", ROSie)

