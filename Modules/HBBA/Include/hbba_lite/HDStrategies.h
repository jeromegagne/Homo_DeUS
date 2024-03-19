#include<HDStrategy.h>

class GotoStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(GotoDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class TalkStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(TalkDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class DiscussStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(DiscussDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class TakeStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(TakeDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class DropStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(DropDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};

class ExploreStrategy : public HDStrategy<nullptr>
{
    std::string m_desireID
    public:
        void Publish(ExploreDesire desire);
        void SubscriberCallBack(const std_msgs::String::ConstPtr& msg);
};
