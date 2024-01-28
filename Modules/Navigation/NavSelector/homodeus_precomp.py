#Mon idée de mettre les imports les plus communs dans un fichier pour éviter de devoir reimport à chaque fois
#Rospy imports
import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Twist, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Normal imports
from math import cos,pi,sin
from os import system
from tokenize import String
from typing import List, Tuple
from time import time

#Aliasing
AL = actionlib 
rp = rospy

#Global consts (treat it as const pls)
moveBaseActionFeedback : MoveBaseActionFeedback = MoveBaseActionFeedback()
NAVGOALSUCCESS = 0x1
NAVGOALFAILED = 0x2
NONAVGOALREMAINING = 0x98
NOUNBLOCKEDNAVGOAL = 0x99

#Act as if this variable is Static
goalStatusList : List[GoalStatus] = {
        GoalStatus.PENDING : 'Pending',
        GoalStatus.ACTIVE : 'Active',
        GoalStatus.PREEMPTED : 'Preempted',
        GoalStatus.SUCCEEDED : 'Succeeded',
        GoalStatus.ABORTED : 'Aborted',
        GoalStatus.REJECTED : 'Rejected',
        GoalStatus.PREEMPTING : 'Preempting',
        GoalStatus.RECALLING : 'Recalling',
        GoalStatus.RECALLED : 'Recalled',
        GoalStatus.LOST : 'Lost'
    }

debug : bool = True # If in local, did not find how to show the rospy.logxxx, so i just added a debut
navGoalCount : int = 0

#Global functions
def getPose() -> Tuple[Point, Quaternion] :
    return moveBaseActionFeedback.feedback.base_position.pose

def initRosNode() -> None:
    rospy.init_node("nav_selector")

def convGoalStatus(goalStatus) -> String :
    return goalStatusList[goalStatus]

#Returns the current value and increments afterwards (similar to i++ from c++)
def getAndIncrementNavGoalCount() -> int :
    global navGoalCount
    returnVal = navGoalCount
    navGoalCount += 1
    return returnVal 

def hdWarn(message : String) -> None :
    if debug :
        print(message)
    else:
        rp.logwarn(message)

def hdInfo(message : String) -> None :
    if debug :
        print(message)
    else:
        rp.loginfo(message)

def hdErr(message : String) -> None :
    if debug :
        print(message)
    else:
        rp.logerr(message)