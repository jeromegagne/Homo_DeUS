from homodeus_precomp import *
from NavGoalDeserializer import NavGoalDeserializer
from NavGoal import NavGoal

class NavSelector :

    __instance = None

    def __new__(cls):
        if cls.__instance is None :
            cls.__instance = super(NavSelector, cls).__new__(cls)
        return cls.__instance

    def __init__(self, goalList : List[NavGoal] = [], currentGoal : NavGoal = None, 
                 topic : str = None, filename : str = None) -> None:
        self.__currentGoal : NavGoal = currentGoal
        self.__goalList : List[NavGoal] = goalList
        self.__topic : str = topic
        self.__hz = rospy.get_param('~hz', 10)
        self.__currentLocation : Tuple[Point,Quaternion] = getPose() #remove if not working
        self.__callBack = None
        self.__isActive : bool = False
        self.__navGoalSerializer = NavGoalDeserializer(filename)
        self.__navGoalSerializer.Read(self.ExtendGoals)
        self.initConnectionToNode()

    def ConnectCallBack(self,callBackFunction) -> None :
        self.__callBack = callBackFunction

    def GetGoalList(self) -> List[NavGoal] :
        return self.__goalList

    def GetCurrentGoal(self) -> NavGoal :
        return self.__currentGoal

    def SetCurrentGoal(self, goal : NavGoal) -> None :
        self.__currentGoal = goal

    def AddGoal(self, goal : NavGoal) -> None :
        if self.__currentGoal is None :
            self.__currentGoal = goal
        else :
            self.__goalList.append(goal)

    def AddGoal(self, goalX : float, goalY : float, goalZ : float, goalOri : float, name : str) -> None :
        print(goalY)
        if self.__currentGoal is None :
            self.__currentGoal = (NavGoal(goalX, goalY, goalZ, goalOri, name))
        else :
            self.__goalList.append(NavGoal(goalX, goalY, goalZ, goalOri, name))

    def ExtendGoals(self, goals : List[NavGoal]) -> None :
        self.__goalList.extend(goals)

    def RemoveGoal(self, index : int) -> None :
        del self.__goalList[index]

    def GetState(self) -> GoalStatus :
        return self.client.get_state()

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
            if (self.GetCurrentGoal() is None):
                self.RemoveCurrentGoal()
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

    def __OnNavGoalFail(self, errorDesc : str, goalStatus : GoalStatus) -> None :
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

    def HandleNodeTaskEnd(self, endState, _) -> None:
        if endState == 0:
            self.__OnNavGoalFail(NAVGOALFAILED,endState)
        elif endState == GoalStatus.SUCCEEDED :
            self.__OnNavGoalSuccess()
        else :
            self.__OnNavGoalFail(NAVGOALFAILED,endState)
        self.RemoveCurrentGoal()

    def SendGoal(self) -> None:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if self.client.get_state() not in [GoalStatus.PENDING]:
            if self.GetCurrentGoal() is not None and not self.GetCurrentGoal().IsBlocked():
                posPoint, oriQuat = self.GetCurrentGoal().GetPose()
                goal.target_pose.pose.position = posPoint
                goal.target_pose.pose.orientation = oriQuat
                self.client.send_goal(goal=goal,done_cb=self.HandleNodeTaskEnd)
            elif self.GetCurrentGoal() is None and self.NbUnblockedTask() == 0 :
                hdInfo("Navigation Selector - No unblocked task left to select, throwing event to the controller")
                self.__OnEvent(NOUNBLOCKEDNAVGOAL)
            elif self.GetCurrentGoal() is None and len(self.GetGoalList()) > 0 :
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
        print('(6) Clear Terminal')
        print('(7) Get status of the current goal')
        print('(8) Cancel/Abort current goal')
        print('(9) Quit')
        print('(10) Set new goal')
        print('(11) Dump the goals list in json')
        print('(12) Print the nav goal list')
        if debug :
            print(f'Nombre de Navigation goal ici présent dans le pays du québec \n Nb = {len(self.GetGoalList())}')
            print(f'Nombre de unblocked task = {self.NbUnblockedTask()}')
            if (self.GetCurrentGoal() is not None):
                print(f'Current goal id : {self.GetCurrentGoal().GetNavGoalID()}')

    def initConnectionToNode(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.__rate = rospy.Rate(self.__hz)

    def run(self) -> None: #Will only manually control the class for now, once the HBBA controller works, will be automatic

        running = True
        while running:
            self.__display_menu()
            strInput : str = input("Please enter your selection number: ")
            if not strInput.isnumeric():
                choice = None
            else:
                choice = int(strInput)
 
            if choice == 0:
                self.SendGoal()
            elif choice == 1:
                self.BlockGoal(0)
            elif choice == 2:
                self.UnblockGoal(0)
            elif choice == 3:
                self.BlockAllGoals()
            elif choice == 4:
                self.UnblockAllGoals()
            elif choice == 5:
                self.__goalList = []
                self.__navGoalSerializer.Read(self.ExtendGoals)
            elif choice == 6:
                system("clear")
            elif choice == 7:
                print(convGoalStatus(self.client.get_state()))
            elif choice == 8:
                self.client.cancel_goal()
                pass
            elif choice == 10:
                # with topic /amcl_pose it possible to obtain the robot's position but position's name is still missing!
                # coming 

                goalName : str = input("Please enter your goal name: ")
                strGoal : str = input("Please enter your coor (x y w) without (): ")
                coords = [float(x) for x in strGoal.split()]
                if len(coords) == 3:
                    self.AddGoal(coords[0], coords[1], 0.0, coords[2], goalName)
            elif choice == 11:
                self.__navGoalSerializer.Write(self.__goalList)
            elif choice == 12:
                for goal in self.GetGoalList():
                    print(goal)
            elif choice == 9:
                print("Bye")
                running = False
            else:
                print("Choice is not valid")

            self.__rate.sleep()