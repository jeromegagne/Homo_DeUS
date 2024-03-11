# Mon idée de mettre les imports les plus communs dans un fichier pour éviter de devoir reimport à chaque fois
# Rospy imports
import rospy
import actionlib

from actionlib import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, Vector3, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Header, String
from std_srvs.srv import Empty, SetBool, Trigger
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Normal imports
from datetime import datetime
from math import atan2, cos, pi, sin, sqrt
from os import system
from typing import List, Tuple
from time import time

# Aliasing
AL = actionlib 
rp = rospy

# Global consts (treat it as const pls)
moveBaseActionFeedback : MoveBaseActionFeedback = MoveBaseActionFeedback()
NAVGOALSUCCESS = 0x1
NAVGOALFAILED = 0x2
NONAVGOALREMAINING = 0x98
NOUNBLOCKEDNAVGOAL = 0x99

# Act as if this variable is Static
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

# Global functions
def getPose() -> Tuple[Point, Quaternion] :
    return moveBaseActionFeedback.feedback.base_position.pose

# https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
def initRosNode(name : str, anonymous: bool = False) -> None:
    rospy.init_node(name, anonymous=anonymous)

def convGoalStatus(goalStatus) -> str :
    return goalStatusList[goalStatus]

# Returns the current value and increments afterwards (similar to i++ from c++)
def getAndIncrementNavGoalCount() -> int :
    global navGoalCount
    returnVal = navGoalCount
    navGoalCount += 1
    return returnVal 

# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2
def quarternion2euler(w: float, x: float, y:float, z:float) -> Vector3:
    q: Quaternion = Quaternion(x,y,z,w)
    return quarternion2euler(q) 

def quarternion2euler(q: Quaternion) -> Vector3:
    euler : Vector3 = Vector3()

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    euler.x = atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    euler.y = 2 * atan2(sinp, cosp) - pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    euler.z = atan2(siny_cosp, cosy_cosp)

    return euler

def hdWarn(message : str) -> None :
    if debug :
        print(message)
    else:
        rp.logwarn(message)

def hdInfo(message : str) -> None :
    if debug :
        print(message)
    else:
        rp.loginfo(message)

def hdErr(message : str) -> None :
    if debug :
        print(message)
    else:
        rp.logerr(message)