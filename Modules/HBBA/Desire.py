from abc import ABC, abstractmethod

class Desire(ABC) :

    def __repr__(self) :
        return f'DesireName : {self.name}, Intensity : {self.intensity}, Type : {self.GetType()}'
    
    def __init__(self, name : str, intensity : int) :
        self.intensity : int = intensity
        self.name : str = name
        self.strategies = []
    
    def GetName(self) -> str :
        return self.name

    def GetIntensity(self) -> int :
        return self.intensity
    
    def GetStrategies(self):
        return self.strategies

    def AddStrategies(self, strategy):
        self.strategies.append(strategy)

    def RemoveStrategies(self, strategyToRemove) -> None :
        self.strategies.remove(strategyToRemove)

    @abstractmethod
    def CancelDesire(self) -> None :
        pass #À implémenter pour chacun des désirs 
    #Pour exemple, un but de navigation, on peut juste faire Nav.CancelCurrentGoal() par exemple

    @abstractmethod
    def GetType(self) -> str :
        pass

class MoveDesire(Desire) :
    def CancelDesire(self) -> None :
        pass

    def GetType(self) -> str :
        return "Move"

class DialogueDesire(Desire) :
    def CancelDesire(self) -> None :
        pass
    
    def GetType(self) -> str :
        return "Dialogue" 
    
class VisionDesire(Desire) :
    def CancelDesire(self) -> None :
        pass

    def GetType(self) -> str :
        return "Vision"

class PrehensionDesire(Desire) :
    def CancelDesire(self) -> None :
        pass
    
    def GetType(self) -> str :
        return "Prehension" 