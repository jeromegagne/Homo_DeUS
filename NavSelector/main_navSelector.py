#!/usr/bin/env python3
# $HOME/tiago_public_ws/src/pmb2_simulation/pmb2_2dnav_gazebo/script/main_navSelector.py
from homodeus_precomp import *
from NavSelector import NavSelector

def f(a) -> None: #As we see, we can get events from the NavSelector in the controller
  print(f"Voici ce qu'on recoit : {convGoalStatus(a)}")

def main() -> None:
  initRosNode()
  nav = NavSelector()
  nav.ConnectCallBack(f)
  nav.run()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass