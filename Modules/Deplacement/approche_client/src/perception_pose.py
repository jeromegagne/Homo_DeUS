#!/usr/bin/env python

# https://wiki.ros.org/ROS/Tutorials/CreatingPackage
# https://docs.ros.org/en/melodic/api/catkin/html/howto/format2/installing_python.html

import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance , PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np

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
# Motivations (Se localiser, Survivre, Servir client)
#   Entrees (perception) [topic]
#     - sa confiance () [/amcl_pose/pose/covariance]
#     - diff repere de la carte et odom (tr.transform("/map", "/odom")) []
#     - de se le faire dire (vocal, texte)
#
#     - niveau de la batterie () []
#
#     - selon l'etat machine rendu
#   Sorties (desir set) []
#     - activer (se localiser) ou le poids grandit en fct d'etre perdu

def quarternion2euler(q: Quaternion) -> Point:
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

def amcl_pose_subscriber(pose_wcs: PoseWithCovarianceStamped):
  pose: Pose = pose_wcs.pose.pose
  point: Point = pose.position
  covar_mat : float[36] = pose_wcs.pose.covariance
  mat   =  np.around(covar_mat, 4)
  #
  mean  = np.mean(covar_mat)               # > 0.0025 -> lost, < 0.0005 -> A+
  msqrt = np.mean(np.square(covar_mat))   # > 0.00005 -> lost, < 0.000001 -> A+
  # max_value = abs(max(covar_mat, key=abs))
  # mat_sum = sum([i for i in covar_mat])
  yaw   = quarternion2euler(pose.orientation).z
  print(f"AMCL - x: {point.x}, y: {point.y}, yaw: {yaw}; \nMean {msqrt}")
  # print(f"{np.reshape(mat, (6,6))}")
  # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_wcs.pose)

def odom_pose_subscriber(odom: Odometry):
  pose: Pose = odom.pose.pose
  point: Point = pose.position
  covar_mat : float[36] = odom.pose.covariance
  mat =  np.around(covar_mat, 4)
  #
  mean  = np.mean(covar_mat)               # > 0.0025 -> lost, < 0.0005 -> A+
  msqrt = np.mean(np.square(covar_mat))   # > 0.00005 -> lost, < 0.000001 -> A+
  # max_value = abs(max(covar_mat, key=abs))
  # mat_sum = sum([i for i in covar_mat])
  yaw   = quarternion2euler(pose.orientation).z
  print(f"ODOM - x: {point.x}, y: {point.y}, yaw: {yaw}; \nMean {msqrt}")
  # print(f"{np.reshape(mat, (6,6))}")
  # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_wcs.pose)

def listener():
  # In ROS, nodes are uniquely named. If two nodes with the same
  # name are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('listener', anonymous=True)
  amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_subscriber)
  odom_pose_sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_pose_subscriber)
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
  listener()

