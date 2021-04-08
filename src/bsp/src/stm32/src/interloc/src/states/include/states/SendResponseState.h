#ifndef HIVE_MIND_WAITFINALSTATE_H
#define HIVE_MIND_WAITFINALSTATE_H

#include "IInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendResponseState : public IInterlocState {
  public:
    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRResponse m_respMsg{};
    UWBRxFrame m_rxFrame;
};

#endif // HIVE_MIND_WAITFINALSTATE_H
