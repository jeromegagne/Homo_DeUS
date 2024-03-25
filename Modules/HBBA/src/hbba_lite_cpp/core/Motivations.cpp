#include <include/hbba_lite/Motivations.h>

AcceuillirClient::AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList) :
Motivation(std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList) 
{
    m_SubscriberList.push_back(nodeHandle.subscribe(subscriberTopicList.first, 10, VisionSubscriberCallBack));
    m_PerceptionList = PerceptionList;
}

void AcceuillirClient::VisionSubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.find("Person"))
    {
        if(msg->data.find("Entrée"))
        {
            m_PerceptionList[0] = true;
            VerifyCondition();
        }
    }
}

void AcceuillirClient::VerifyCondition()
{
    bool valid = true;
    for(auto& Perception : m_PerceptionList)
    {
        if(!Perception){
            valid = false;
        } 
    }
    if(valid){
        StateMachine();
    }
}

void AcceuillirClient::StateMachine()
{
    //To Do
}