#ifndef HIVE_MIND_SETDISTANCESTATE_H
#define HIVE_MIND_SETDISTANCESTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SetDistanceState : public AbstractInterlocState {
  public:
    SetDistanceState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    //    UWBMessages::TWRResponse m_respMsg{};
};

#endif // HIVE_MIND_SETDISTANCESTATE_H
