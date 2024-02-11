#!/usr/bin/env python3
# $HOME/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/main_navSelector.py
from homodeus_precomp import *
from approche_client import ApproachClient
from NavSelector import NavSelector
from time import sleep

sendSecondGoal = False

def f(a) -> None: #As we see, we can get events from the NavSelector in the controller
  print(f"Voici ce qu'on recoit comme résultat du goal : {convGoalStatus(a)}")
  global sendSecondGoal
  sendSecondGoal = True

def wait_for_goal_end() :
    global sendSecondGoal
    while(sendSecondGoal == False) :
        pass

def main() -> None:
  global sendSecondGoal
  initRosNode("nav_selector")
  hdInfo("Waiting for system init and arm to tuck, waiting 5 seconds")
  sleep(5)
  hdInfo("Starting test with 2 goals, navigating to the area required")
  nav = NavSelector()
  nav.AddGoal(-1.53, 0, 0, 0, "test1")
  nav.AddGoal(2, -3, 0, 1.57 * 3, "test2") 
  nav.ConnectCallBack(f)
  nav.SendGoal()
  wait_for_goal_end()
  sendSecondGoal = False
  nav.SendGoal()
  wait_for_goal_end()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass