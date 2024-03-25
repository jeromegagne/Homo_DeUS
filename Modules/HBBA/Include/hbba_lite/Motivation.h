#ifndef HBBA_LITE_CORE_MOTIVATION_H
#define HBBA_LITE_CORE_MOTIVATION_H

#include <hbba_lite/core/DesireSet.h>

#include <memory>
#include <atomic>
#include <thread>

class Motivation
{
protected:
    std::shared_ptr<DesireSet> m_desireSet;
    std::vector<Desire> m_desireList{};
public:
    Motivation(std::shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList);
    virtual ~Motivation() = default;
};

#endif