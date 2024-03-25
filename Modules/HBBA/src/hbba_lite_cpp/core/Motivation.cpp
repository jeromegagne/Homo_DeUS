#include <include/hbba_lite/Motivation.h>

using namespace std;

Motivation::Motivation(shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList)
{
    m_desireSet(move(desireSet));
    m_desireList = desireList;
}