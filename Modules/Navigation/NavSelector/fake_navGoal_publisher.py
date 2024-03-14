#!/usr/bin/env python3
from homodeus_precomp import *
import std_msgs.msg._String

def callbackResponse(val : std_msgs.msg.Int8) -> None :
  print(f"We received via the topic the following response : {val} which indicate that : {responseInterpretation(val)} ")

def responseInterpretation(val : std_msgs.msg.Int8) -> String :
  if val.data & 0b01 :
    return "Goal received, stop sending"
  return "No you should not be there :("

def main() -> None:
  initRosNode("fake_hbba")
  pub = rospy.Publisher("goto/Request", Pose, queue_size = 1, latch=True)
  sub = rospy.Subscriber("goto/Response", std_msgs.msg.Int8, callback = callbackResponse, queue_size = 10)
  pose : Pose = Pose(Point(0.326, 2.723, 0), Quaternion(0, 0, sin(1.662) * 0.5, cos(1.662) * 0.5))
  publish_time = time()
  pub.publish(pose)

  print("Waiting for the response of the nav selector")
  rospy.wait_for_message("goto/Response", std_msgs.msg.Int8)
  response_time = time() - publish_time
  print("Goto/0 said that we received the message")
  print(f"Respon se time for a two way communication : {response_time}")

  print("Returning the robot to (0,0)")
  pub.publish(Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0)))

  rospy.wait_for_message("goto/Response", std_msgs.msg.Int8)
  print("Goto/Response said that we received the message")

  pub.unregister()
  sub.unregister()
  exit()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass