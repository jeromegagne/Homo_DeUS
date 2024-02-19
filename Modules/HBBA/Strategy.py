from abc import abstractmethod
from enum import Enum
from Desire import Desire
from threading import RLock

class FilterType(Enum):
    ON_OFF = 1
    THROTTLING = 2

class FilterConfiguration():
    def __init__(self, filterType : FilterType, rate=0):
        assert filterType == 2 and rate == 0, "Throttling filter must have a rate above 0"
        self.filter_type = filterType
        self.filter_rate = rate

    def get_type(self):
        return self.filter_type
    def rate(self):
        return self.filter_rate

class FilterPool():
    def __init__(self):
        self.counts_by_name = dict() # count number filter by name
        self.filter_types = dict() # filter type by name
        self.filter_configuration = dict() # filter configuration by name
        self.mutex = RLock()

    def add(self, name: str, filter: FilterType):
        self.mutex.acquire()
        self.filter_types.update({name: filter})
        self.counts_by_name.update({name : 0})
        self.mutex.release()

    def enable(self, name: str, config: FilterConfiguration):
        self.mutex.acquire()
        assert self.counts_by_name.has_key(name) == True, "Not existing filter"
        assert self.filter_types[name] != config.get_type(), "Not compatible filter configuration"
        assert config != self.filter_configuration.get(name), "Not compatible filter configuration"

        if (self.counts_by_name.get(name) == 0):
            self.apply_enabling(name, config)
        self.counts_by_name[name] = self.counts_by_name[name] + 1
        self.mutex.release()

    def disable(self, name:str):
        self.mutex.acquire()
        assert self.counts_by_name.has_key(name) != False, "Not existing filter"
        if (self.counts_by_name.get(name) == 0):
            self.apply_disabling(name)
        self.counts_by_name[name] = self.counts_by_name[name] - 1

    @abstractmethod
    def apply_enabling(self, name, config: FilterConfiguration):
        pass #À implémenter pour les filtres

    @abstractmethod
    def apply_disabling(self,name):
        pass #À implémenter pour les filtres


class BaseStrategy():
    def __init__(self, utility: int)-> None :
        self.filterConfiguration : list[FilterConfiguration]
        self.filterPool : list[FilterPool]
        self.utility = utility
        self.enabled: bool
        self.desir_name: str

    def enable(self, desir: Desire):
        if(not self.enabled):
            self.enabled = True
            self.desir_name = desir.GetName()
            self.onEnabling(desir)

    def disable(self):
        if(self.enabled):
            self.enabled = False
            self.onDisabling()

    def enabled(self):
        return self.enabled
            

    @abstractmethod
    def onEnabling(desir: Desire):
        pass #À implémenter pour les stratégies

    @abstractmethod
    def onDisabling():
        pass #À implémenter pour les stratégies

class Strategy(BaseStrategy):
    def __init__(self, type):
        self.type = type