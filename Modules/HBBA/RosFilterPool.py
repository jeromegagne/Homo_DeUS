from Strategy import *
from abc import abstractmethod
from Strategy import FilterConfiguration
import rospy
from hbba_lite.srv import SetOnOffFilterState, SetThrottlingFilterState

class RosFilterPool(FilterPool):
    def __init__(self, nodeHandle, wait_for_service: bool):
        self.nodeHandle = rospy.init_node(nodeHandle)
        self.wait_for_service = wait_for_service
        self. service_clients_by_name = dict() # ros::ServiceClient by name

    def add_filter_type(self, name, type:FilterType):
        self.mutex.acquire()
        self.add(name, type)

        if(type == FilterType.ON_OFF):
            self.service_clients_by_name[name] = self.nodeHandle.service_clients
        elif(type == FilterType.THROTTLING):
            self.service_clients_by_name[name] = self.nodeHandle.service_clients
            #m_serviceClientsByName[name] = m_nodeHandle.serviceClient<hbba_lite::SetThrottlingFilterState>(name, true);
        else:
            print("Not supported filter type")
        self.mutex.release()
    
    def apply_enabling(self, name, config: FilterConfiguration):
        item = self.filter_types[name]

        if(item == FilterType.ON_OFF):
            srv: SetOnOffFilterState
            srv.request.is_filtering_all_messages = False
            srv.request.rate = config.rate()
            self.call(name, srv)

        elif(item == FilterType.THROTTLING):
            srv: SetThrottlingFilterState
            srv.request.is_filtering_all_messages = False
            srv.request.rate = config.rate()
            self.call(name, srv)

    def apply_disabling(self, name):
        item = self.filter_types[name]

        if(item == FilterType.ON_OFF):
            srv: SetOnOffFilterState
            srv.request.is_filtering_all_messages = True
            self.call(name, srv)

        elif(item == FilterType.THROTTLING):
            srv: SetThrottlingFilterState
            srv.request.is_filtering_all_messages = True
            srv.request.rate = 1
            self.call(name, srv)

    def call(self, name, srv):
        # ros::ServiceClient& service = m_serviceClientsByName[name];

        if(self.wait_for_service):
            # ros::service::waitForService(name);
        