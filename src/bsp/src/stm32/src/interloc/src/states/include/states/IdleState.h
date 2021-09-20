#ifndef HIVE_MIND_IDDLESTATE_H
#define HIVE_MIND_IDDLESTATE_H

#include "AbstractInterlocState.h"

class IdleState : public AbstractInterlocState {
  public:
    IdleState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private:
    static void processOperatingMode(InterlocStateHandler& context);
};

#endif // HIVE_MIND_IDDLESTATE_H
