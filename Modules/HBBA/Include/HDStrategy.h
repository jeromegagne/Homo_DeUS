#pragma once

#include <ros/ros.h>
#include <hbba_lite/core/Strategy.h>
#include <hbba_lite/core/DesireSet.h>

#include <iostream>
#include <vector>
#include <map>

template<class T>
class HDStrategy : public Strategy<T>
{
public:
    HDStrategy<T>(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string, bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet , std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName);

protected:
    virtual void SubscriberCallBack() {};

private:
    std::vector<ros::Publisher>  m_PublisherList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
    std::shared_ptr<DesireSet> m_DesireSet = nullptr;

};