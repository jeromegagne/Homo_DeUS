#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class ObstacleDetectorSonar:
  def __init__(self) -> None:
    self.__limite_distance: float     = 0.3048  # m
    self.__last_range_sonars: dict    = {}
    
    self.__range_alpha: float         = 0.625   # %
    self.__range_filted: float        = 0.0     # m

    self.__move_alpha: float          = 0.75    # %
    self.__move_linear_filted: float  = 0.0     # m/s
    self.__move_angular_filted: float = 0.0     # rad/s
    
    self.__phone_vel_pub  = rospy.Publisher("phone_vel", Twist, queue_size=1)
    self.__key_vel_sub    = rospy.Subscriber('/key_vel', Twist, self.__key_vel_callback, queue_size=1)
    self.__sonar_sub      = rospy.Subscriber('sonar_base', Range, self.__sonar_callback, queue_size=1)

    rospy.loginfo("Obstacle detector sonar initialized")
    rospy.on_shutdown(self.__node_shutdown)


  def __key_vel_callback(self, twist: Twist) -> None:
    self.__move_linear_filted: float  = twist.linear.x * self.__move_alpha + self.__move_linear_filted * (1-self.__move_alpha)
    self.__move_angular_filted: float = twist.angular.z * self.__move_alpha + self.__move_angular_filted * (1-self.__move_alpha)


  def __sonar_callback(self, sonar_msg: Range) -> None:
    # each sonar will generate a separate call

    sonar_name: str = sonar_msg.header.frame_id
    if not sonar_name in self.__last_range_sonars:
      self.__last_range_sonars[sonar_name] = sonar_msg.min_range

    obsDetected: bool = True
    range: float      = sonar_msg.range

    if range > sonar_msg.min_range - 0.01 and range < sonar_msg.max_range + 0.01:
      obsDetected = False
      self.__range_filted = range * self.__range_alpha + self.__range_filted * (1-self.__range_alpha)

      if self.__move_linear_filted  < 0.10 and \
         self.__range_filted < self.__limite_distance:
        obsDetected = True
      self.__last_range_sonars[sonar_name] = range

    if obsDetected:
      self.__phone_vel_pub.publish(Twist())
      rospy.loginfo("Obstacle detected with sonar! Stop!")


  def __node_shutdown(self) -> None:
    self.__key_vel_sub.unregister()
    self.__sonar_sub.unregister()

    rospy.loginfo("Obstacle detector sonar shutdown")


def main() -> None:
  rospy.init_node("obstacle_detector_sonar")
  osbDetectorSonar = ObstacleDetectorSonar()
  rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
