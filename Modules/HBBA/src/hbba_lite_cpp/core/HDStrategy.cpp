#include <hbba_lite/core/HDStrategy.h>

#include <std_msgs/String.h>
#include <memory.h>
#include <HDStrategy.h>



template<class T>
HDStrategy<T>::HDStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string, bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet , std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName) :
//HDStrategy<T>::HDStrategy(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<string, bool>& publisherTopicList, const std::map<string, bool>& subscriberTopicList, DesireSet& desireSet, std::unordered_map<string, FilterConfiguration> filterConfigurationByName) : 
Strategy<T>(0, {}, {filterConfigurationByName}, move(filterPool))
{
    for (const auto& [publisherTopic, latch] : publisherTopicList)
    {
        m_PublisherList.push_back(nodeHandle.advertise<std_msgs::String>(publisherTopic), 10, latch);
    }
    
    for (const auto& [subscriberTopic, latch] : subscriberTopicList)
    {
        m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopic, 10, SubscriberCallBack));
    }
    m_DesireSet = desireSet;
}