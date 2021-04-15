#ifndef HIVE_MIND_SENDRESPONSESTATE_H
#define HIVE_MIND_SENDRESPONSESTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendResponseState : public AbstractInterlocState {
  public:
    SendResponseState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRResponse m_respMsg{};
};

#endif // HIVE_MIND_WAITFINALSTATE_H
