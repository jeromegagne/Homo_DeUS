from homodeus_precomp import *
from NavGoalDeserializer import NavGoalDeserializer
from NavGoal import NavGoal

class NavSelector :
    staticTopic = "navigationTopic"
    staticInterruptTopic = "navigationInterrupt"
    __goalList : List[NavGoal]
    __currentGoal : NavGoal
    __callBack = None
    __currentLocation : Tuple[Point,Quaternion]

    def __init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, topic : String = None) -> None:
        self.__currentGoal = currentGoal
        self.__goalList = goalList
        self.__topic = topic
        self.__hz = rospy.get_param('~hz', 10)
        self.__currentLocation = getPose() #remove if not working
        NavGoalDeserializer().Read(self.ExtendGoals)

    def ConnectCallBack(self,callBackFunction) -> None :
        self.__callBack = callBackFunction

    def GetGoalList(self) -> List[NavGoal] :
        return self.__goalList

    def GetCurrentGoal(self) -> NavGoal :
        return self.__currentGoal

    def SetCurrentGoal(self, goal : NavGoal) -> None :
        self.__currentGoal = goal

    def AddGoal(self, goal : NavGoal) -> None :
        self.__goalList.append(goal)

    def AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : String) -> None :
        self.__goalList.append(NavGoal(goalX, goalY, goalZ, goalOri, name))

    def ExtendGoals(self, goals : List[NavGoal]) -> None :
        self.__goalList.extend(goals)

    def RemoveGoal(self, index : int) -> None :
        del self.__goalList[index]

    def RemoveCurrentGoal(self) -> None : #prob rename
        for goal in self.GetGoalList() :
            if (not goal.IsBlocked()) :
                self.SetCurrentGoal(goal)
                self.__goalList.remove(goal) # prone to cause error, maybe just wait after the loop then delete
                self.__currentLocation = getPose()
                return
        self.SetCurrentGoal(None)
        hdWarn("Navigation Selector - No NavGoal capable to be run at the current time, waiting for more")

    def BlockGoal(self, index : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index < nbElem :
            self.__goalList[index].BlockNavGoal()
            return
        hdWarn("Navigation Selector - No goal matches this index, no goal will be blocked")
        
    def BlockAllGoals(self) -> None :
        for goal in self.GetGoalList() :
            goal.BlockNavGoal()

    def UnblockGoal(self, index : int) -> None :
        nbElem = len(self.GetGoalList()) 
        if nbElem != 0 and index < nbElem :
            self.GetGoalList()[index].UnblockNavGoal()
            return
        hdWarn("Navigation Selector - No goal matches this index, no goal will be unblocked")

    def UnblockAllGoals(self) -> None :
        for goal in self.GetGoalList() :
            goal.UnblockNavGoal()

    def NbUnblockedTask(self) -> int :
        cpt = 0
        for goal in self.GetGoalList() :
            if not goal.IsBlocked() :
                cpt += 1
        return cpt

    def __OnNavGoalFail(self, errorDesc : String, goalStatus : GoalStatus) -> None :
        hdErr(f"Navigation Selector - Failed to fulfill the current goal (NavGoalID = {self.GetCurrentGoal().GetNavGoalID()}), removed from the list and notified the main controller \
                \nGoal created at {self.GetCurrentGoal().GetCreationTime()}, \
                \nTime taken for this resolution = {self.GetCurrentGoal().GetNavGoalExistenceTime()} \
                \nGoalStatus = {convGoalStatus(goalStatus)} \
                \nGoalStatus Value = {goalStatus} \
                \nError associated : {errorDesc}")
        self.__OnEvent(goalStatus)

    def __OnNavGoalSuccess(self) -> None :
        hdInfo(f"Navigation Selector - Current goal (NavGoalID = {self.GetCurrentGoal().GetNavGoalID()}), succeeded, beginning next NavGoal when the controller is ready \
                    \nGoal created at {self.GetCurrentGoal().GetCreationTime()}, \
                    \nTime taken for the success = {self.GetCurrentGoal().GetNavGoalExistenceTime()}")
        self.__OnEvent(NAVGOALSUCCESS)

    def __OnEvent(self, eventContent) -> None :
        if eventContent is not None and self.GetCurrentGoal() is not None:
            hdInfo(f"Navigation Selector - Event triggered by the current Navigation Goal (NavGoalID = {self.GetCurrentGoal().GetNavGoalID()})")
            self.__callBack(eventContent)

    def Sort(self) -> None:
        #self.__goalList.sort() #Need to sort with a key, which key, I don't know
        pass

    def __sendGoal(self) -> None:
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if self.GetCurrentGoal() is not None :
            posPoint, oriQuat = self.GetCurrentGoal().GetPose()
            goal.target_pose.pose.position = posPoint
            goal.target_pose.pose.orientation = oriQuat
            res = self.client.send_goal_and_wait(goal)#add correct timeout
            print(f"Goal complete : {self.GetCurrentGoal().GetNavGoalID()}")
            if res < GoalStatus.ABORTED :
                self.__OnNavGoalSuccess()
            else :
                self.__OnNavGoalFail(NAVGOALFAILED,res)
            self.RemoveCurrentGoal()
        elif self.GetCurrentGoal() is None and self.NbUnblockedTask() == 0 :
            hdInfo("Navigation Selector - No unblocked task left to select, throwing event to the controller")
            self.__OnEvent(NOUNBLOCKEDNAVGOAL)
        elif self.GetCurrentGoal() is None and len(self.GetGoalList()) == 0 :
            hdInfo("Navigation Selector - No tasks left to select, throwing event to the controller")
            self.__OnEvent(NONAVGOALREMAINING)
        elif self.GetCurrentGoal() is None and len(self.GetGoalList()) == 0 :
            self.RemoveCurrentGoal()

    def __display_menu(self) -> None:
        #system("clear")
        print('(0) Just continue')
        print('(1) Block next task')
        print('(2) Unblock next task')
        print('(3) Block all tasks')
        print('(4) Unblock all tasks')
        print('(5) Reload the predef goals')
        print('(9) Quit')
        if debug :
            print(f'Nombre de Navigation goal ici présent dans le pays du québec \n Nb = {len(self.GetGoalList())}')
            if (self.GetCurrentGoal() is not None):
                print(f'Current goal id : {self.GetCurrentGoal().GetNavGoalID()}')

    def run(self) -> None: #Revoir au complet la func de run, pas un fan de comment on appel le __send_goal

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.__rate = rospy.Rate(self.__hz)

        if self.GetCurrentGoal() is None and len(self.GetGoalList()) != 0 :
            self.SetCurrentGoal(self.GetGoalList()[0])
            del self.__goalList[0] #Scuffed

        running = True
        while running:
            self.__display_menu()
            choice = int(input("Please enter your selection number: "))
            if choice == 0: #Deg
                self.__sendGoal()
            elif choice == 1:
                self.BlockGoal(0)
            elif choice == 2:
                self.UnblockGoal(0)
            elif choice == 3:
                self.BlockAllGoals()
            elif choice == 4:
                self.UnblockAllGoals()
            elif choice == 5:
                NavGoalDeserializer().Read(self.ExtendGoals)
            elif choice == 6:
                system("clear")
            elif choice == 9:
                print("Bye")
                running = False
            else:
                print("Choice is not valid")
            self.__rate.sleep()