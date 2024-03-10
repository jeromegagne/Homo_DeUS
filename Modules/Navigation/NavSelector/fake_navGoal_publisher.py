#!/usr/bin/env python3
from homodeus_precomp import *
from time import sleep
import std_msgs.msg._String
filename : str = "/home/pal/tiago_public_ws/src/hbba_lite-main/scripts/NavSelector/predefNavGoal.json"
responseReturned : bool = False
received : bool = False

def callbackResponse(val : std_msgs.msg.Int8) -> None :
  print(f"We received via the topic the following response : {val} which indicate that : {responseInterpretation(val)} ")

def responseInterpretation(val : std_msgs.msg.Int8) -> String :
  global responseReturned, received
  if val & 0b01 :
    received = True
    return "Goal received, stop sending"
  elif val & 0b11 : 
    responseReturned = True
    return "Goal completed"
  return "How TF"

def main() -> None:
  global responseReturned, received
  initRosNode("fake_hbba")
  pub = rospy.Publisher("/gotoRequest", Pose, queue_size = 10, tcp_nodelay=True)
  sub = rospy.Subscriber("/gotoResponse", std_msgs.msg.Int8, callback=callbackResponse, queue_size = 1)
  pose : Pose = Pose(Point(0.326, 2.723, 0), Quaternion(0, 0, sin(1.662) * 0.5, cos(1.662) * 0.5))
  pub.publish(pose)

  print("Value vas returned")
  pub.unregister()
  sub.unregister()
  exit()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass