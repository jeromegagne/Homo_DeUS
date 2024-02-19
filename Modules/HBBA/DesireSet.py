from Desire import Desire

class DesireSet():
    
    def __init__(self) -> None :
        self.Set : list[Desire] = []

    def GetSet(self) -> list[Desire] :
        return self.Set
    
    def GetCurrentDesire(self) -> Desire :
        return self.Set[0]

    def AddDesire(self, desire : Desire) -> None :
        self.Set.append(desire)
    
    def RemoveDesire(self, desire : Desire) -> None :
        self.Set.remove(desire)

    def RemoveFirstDesire(self) -> None :
        self.Set.pop(0)
    
    def ClearSet(self) -> None :
        self.Set.clear()

    def GetAllDesireOfType(self, type : str) -> list[Desire] :
        typeSet : list[Desire] = []
        for desire in self.Set:
            if(desire.GetType() == type):
                typeSet.append(desire)
        return typeSet