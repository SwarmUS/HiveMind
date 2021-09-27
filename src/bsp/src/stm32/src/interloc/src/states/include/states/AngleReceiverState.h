#ifndef __ANGLERECEIVERSTATE_H__
#define __ANGLERECEIVERSTATE_H__

#include "AbstractInterlocState.h"
#include "interloc/UWBRxFrame.h"
#include <array>

class AngleReceiverState : public AbstractInterlocState {
  public:
    AngleReceiverState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    std::array<UWBRxFrame, DecawaveArray::angleAntennaArraySize> m_rxFrames;

    void saveAngleData(BspInterlocRawAngleData& data, uint32_t frameIndex);
    void readAngleFrame();
};

#endif //__ANGLERECEIVERSTATE_H__
