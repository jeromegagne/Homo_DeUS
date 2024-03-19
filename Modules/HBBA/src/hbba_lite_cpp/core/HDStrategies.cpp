#include<HDStrategies.h>

GotoStrategy::GotoStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void GotoStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == GotoStrategy::m_desireID)
    {
        Stragy<GotoDesire>::onDisabling();
        /*GotoStrategy::onDisabling();*/
    }
}

void GotoStrategy::Publish(GotoDesire desire)
{
    GotoStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<GotoDesire>::onEnabling(msg);
    /*GotoStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : GotoStrategy::m_PublisherList){
        pub.publish(msg);
    }
}


TalkStrategy::TalkStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void TalkStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == TalkStrategy::m_desireID)
    {
        Stragy<TalkDesire>::onDisabling();
        /*TalkStrategy::onDisabling();*/
    }
}

void TalkStrategy::Publish(TalkDesire desire)
{
    
    TalkStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<TalkDesire>::onEnabling(msg);
    /*TalkStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : TalkStrategy::m_PublisherList){
        pub.publish(msg);
    }
}

DiscussStrategy::DiscussStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void DiscussStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == DiscussStrategy::m_desireID)
    {
        Stragy<DiscussDesire>::onDisabling();
        /*DiscussStrategy::onDisabling();*/
    }
}

void DiscussStrategy::Publish(DiscussDesire desire)
{
    DiscussStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<DiscussDesire>::onEnabling(msg);
    /*DiscussStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : DiscussStrategy::m_PublisherList){
        pub.publish(msg);
    }
}


TakeStrategy::TakeStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void TakeStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == TakeStrategy::m_desireID)
    {
        Stragy<TakeDesire>::onDisabling();
        /*TakeStrategy::onDisabling();*/
    }
}

void TakeStrategy::Publish(TakeDesire desire)
{
    
    TakeStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<TakeDesire>::onEnabling(msg);
    /*TakeStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : TakeStrategy::m_PublisherList){
        pub.publish(msg);
    }
}

DropStrategy::DropStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void DropStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == DropStrategy::m_desireID)
    {
        Stragy<DropDesire>::onDisabling();
        /*DropStrategy::onDisabling();*/
    }
}

void DropStrategy::Publish(DropDesire desire)
{
    DropStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<DropDesire>::onEnabling(msg);
    /*DropStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : DropStrategy::m_PublisherList){
        pub.publish(msg);
    }
}


ExploreStrategy::ExploreStrategy(std::shared_ptr<FilterPool> filterPool) 
: HDStrategy<nullptr>
(std::shared_ptr<FilterPool> filterPool, ros::NodeHandle& nodeHandle, const std::map<std::string,
 bool>& publisherTopicList, const std::map<std::string, bool>& subscriberTopicList, DesireSet& desireSet,
 std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName)
{

}

void ExploreStrategy::SubscriberCallBack(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == ExploreStrategy::m_desireID)
    {
        Stragy<ExploreDesire>::onDisabling();
        /*ExploreStrategy::onDisabling();*/
    }
}

void ExploreStrategy::Publish(ExploreDesire desire)
{
    
    ExploreStrategy::m_desireID = desire.id();
    std_msgs::String msg;
    msg.data = desire.name();
    Stragy<ExploreDesire>::onEnabling(msg);
    /*ExploreStrategy::onEnabling(msg);*/
    for(ros::Publisher pub : ExploreStrategy::m_PublisherList){
        pub.publish(msg);
    }
}