from DesireSet import *
from abc import abstractmethod

class Motivation() :

    def __init__(self) -> None :
        self.DesireSet : DesireSet = DesireSet()

    def GetDesireSet(self) -> DesireSet :
        return self.DesireSet

    @abstractmethod #À réimplémenter au besoin
    def StrategyCallback(self, valeurReponse : int) -> None :
        if self.StrategieSuccess(valeurReponse) :
            self.DesireSet.RemoveFirstDesire()
    
    @abstractmethod
    def Observe(self) -> None :
        pass #À implémenter pour les perceptions

    @abstractmethod
    def StopObserving(self) -> None :
        pass #À implémenter pour les perceptions

    @abstractmethod
    def CanceledCB(self) -> None :
        pass #À implémenter pour les motivations

    def CancelDesireCourrant(self) -> None :
        self.DesireSet.GetCurrentDesire().CancelDesire()

    @abstractmethod
    def OnStrategieFail(self):
        pass

    @abstractmethod
    def StrategieSuccess(self, code : int) -> bool :
        return True if code < 3 else False
    
    @abstractmethod
    def ResetMotivation(self) -> None : 
        pass


