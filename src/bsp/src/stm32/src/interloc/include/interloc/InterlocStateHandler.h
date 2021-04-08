#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include "TwoWayRanging.h"
#include <array>
#include <states/AbstractInterlocState.h>
#include <states/InterlocStateContainer.h>

class InterlocStateHandler {
  public:
    InterlocStateHandler();

    void setState(InterlocStates state);
    void process();

    TwoWayRanging& getTWR();

  private:
    AbstractInterlocState* m_currentState;
    TwoWayRanging m_twr{};
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
