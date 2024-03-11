#!/usr/bin/env python3
# $HOME/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/main_navSelector.py
from homodeus_precomp import *
# from approche_client import ApproachClient
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
  sendSecondGoal = False

filename : str = "/home/urobot/tiago_public_ws/src/zhomodeus/base_navigation/scripts/predefNavGoal.json"

def __node_shutdown():
  nav = NavSelector()
  nav.CancelAllGoals()

def main() -> None:
  
  global sendSecondGoal
  initRosNode("nav_selector")
  rospy.on_shutdown(__node_shutdown)
  hdInfo("Waiting for system init and arm to tuck, waiting 5 seconds")
  sleep(5)

  nav = NavSelector()
  nav.SetFilename(filename=filename)
#   nav.AddGoal(-1.53, 0, 0, 0, "test1")
#   nav.AddGoal(2, -3, 0, 1.57 * 3, "test2") 
  nav.ConnectCallBack(f)

  str_input : str = input("start recover (o/n) | 2D pose estimate: ")
  # if str_input != 'o':
  #   print('exit recover')
  #   return
  # nav.RelocateItselfInMap()
  
  str_input : str = input("start clear map (o/n) : ")
  if str_input != 'o':
    print('exit recover')
    return
  nav.ClearMap()
  
  for iter_circuit in range(5):
    str_input : str = input("start circuit " + str(iter_circuit) + " (o/n) : ")
    if str_input != 'o':
      print('exit laps circuit')
      break

    nav.LoadPreDefNavGoal()
    nav.SetIndexCurrentGoal(0)
    nav.UnblockAllGoals()

    for index_goal in range(5):
      str_input : str = input("send goal " + str(index_goal) + " (o/n) : ")
      if str_input != 'o':
        print('exit goal')
        break

      nav.SendGoal()
      wait_for_goal_end()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass