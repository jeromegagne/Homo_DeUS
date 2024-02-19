from DesireSet import *
from Desire import *
from Motivation import *

class Promenade(Motivation) :
    def __init__(self) -> None:
        super().__init__()
        self.DesireSet.AddDesire(MoveDesire("Goto", 1))
    
    def Observe(self) -> None :
        pass

    
    def StopObserving(self) -> None :
        pass

    
    def CanceledCB(self) -> None :
        pass

    def CancelDesireCourrant(self) -> None :
        self.DesireSet.GetCurrentDesire().CancelDesire()

    
    def OnStrategieFail(self):
        pass

    def StrategieSuccess(self, code : int) -> bool :
        return True if code < 3 else False

    def ResetMotivation(self) -> None:
        self.DesireSet.AddDesire(MoveDesire("Goto", 1))
