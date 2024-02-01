#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import numpy as np

class ObstacleDetectorSonar:
  def __init__(self) -> None:
    self.__limite_distance = 0.3048  # m
    self.__last_range_sonars = {}
    self.__move_linear = 0.0
    self.__move_angular = 0.0
    
    self.__phone_vel_pub = rospy.Publisher("phone_vel", Twist, queue_size=1)
    self.__key_vel_sub = rospy.Subscriber('key_vel', Twist, self.__key_vel_callback, queue_size=1)
    self.__sonar_sub = rospy.Subscriber('sonar_base', Range, self.__sonar_callback, queue_size=1)
    rospy.on_shutdown(self.__node_shutdown)

    rospy.loginfo("Obstacle detector sonar initialized")

  def __key_vel_callback(self, twist: Twist) -> None:
    self.__move_linear = twist.linear.x / (abs(twist.linear.x)+1)
    self.__move_angular = twist.angular.z / (abs(twist.angular.z)+1)

  def __sonar_callback(self, sonar_msg: Range) -> None:
    # each sonar will generate a separate call

    sonar_name = sonar_msg.header.frame_id
    if not sonar_name in self.__last_range_sonars:
      self.__last_range_sonars[sonar_name] = sonar_msg.min_range

    obsDetected = False
    range = sonar_msg.range
    if range > sonar_msg.min_range and range < sonar_msg.max_range:
      if self.__move_linear < 0 and \
         range < self.__limite_distance:
        obsDetected = True
      self.__last_range_sonars[sonar_name] = range

    if obsDetected:
      self.__phone_vel_pub.publish(Twist())
      rospy.loginfo("Obstacle detected with sonar! Stop!")

  def __node_shutdown(self) -> None:
    self.__key_vel_sub.unregister()
    self.__sonar_sub.unregister()
    rospy.loginfo("Obstacle detector sonar shutdown")

def main():
  rospy.init_node("obstacle_detector_sonar")
  osbDetectorSonar = ObstacleDetectorSonar()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
