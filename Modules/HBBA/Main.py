from Desire import *
from MotivationList.GuiderClientTable import *
from MotivationList.Promenade import *

#Main qui permet de tester les motivations

VALEUR_TEMP = 1
guiderClientTableMotivation : GuiderClientTable = GuiderClientTable()

print(guiderClientTableMotivation.GetDesireSet().GetSet())
guiderClientTableMotivation.StrategyCallback(VALEUR_TEMP)
print(guiderClientTableMotivation.GetDesireSet().GetSet())
print(guiderClientTableMotivation.GetDesireSet().GetAllDesireOfType("Move"))
guiderClientTableMotivation.GetDesireSet().ClearSet()
print(guiderClientTableMotivation.GetDesireSet().GetSet())