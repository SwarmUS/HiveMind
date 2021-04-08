#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include "TwoWayRanging.h"
#include <states/IInterlocState.h>

class InterlocStateHandler {
  public:
    InterlocStateHandler();

    void setState(IInterlocState& state);
    void process();

    TwoWayRanging& getTWR();

  private:
    IInterlocState* m_state;
    TwoWayRanging m_twr;
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
