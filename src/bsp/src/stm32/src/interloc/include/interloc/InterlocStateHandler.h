#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include <states/IInterlocState.h>

class InterlocStateHandler {
  public:
    InterlocStateHandler();

    void setState(IInterlocState& state);
    void process();

  private:
    IInterlocState& m_state;
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
