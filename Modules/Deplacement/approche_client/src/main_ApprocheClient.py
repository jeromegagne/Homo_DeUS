#!/usr/bin/env python3

from Navigation import homodeus_precomp
from approche_client import ApproachClient

def main() -> None:
  approcheClient = ApproachClient()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass