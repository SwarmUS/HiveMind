#ifndef HIVE_MIND_SENDFINALSTATE_H
#define HIVE_MIND_SENDFINALSTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendFinalState : public AbstractInterlocState {
  public:
    SendFinalState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRFinal m_finalMsg{};
};

#endif // HIVE_MIND_SENDFINALSTATE_H
