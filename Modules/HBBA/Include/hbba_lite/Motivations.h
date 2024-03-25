#include<include/hbba_lite/Motivation.h>

class AcceuillirClient : public Motivation
{
protected:
    std::vector<bool> m_PerceptionList{};
    std::vector<ros::Subscriber> m_SubscriberList{};
public:
    AcceuillirClient(const std::map<std::string, bool>& subscriberTopicList, ros::NodeHandle& nodeHandle, std::vector<bool> PerceptionList);
    void VisionSubscriberCallBack(const std_msgs::String::ConstPtr& msg) {};
    void VerifyCondition();
    void StateMachine();
};