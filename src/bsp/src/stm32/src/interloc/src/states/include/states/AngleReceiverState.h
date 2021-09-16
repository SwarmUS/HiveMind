#ifndef __ANGLERECEIVERSTATE_H__
#define __ANGLERECEIVERSTATE_H__

#include "AbstractInterlocState.h"

class AngleReceiverState : public AbstractInterlocState {
  public:
    constexpr static uint8_t numDecas = 2; // MAXIMUM 3

    AngleReceiverState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;
    static void saveAngleData(BspInterlocRawAngleData& data,
                              std::array<UWBRxFrame, numDecas>& rxFrames,
                              uint32_t frameIndex);
    void readAngleFrame(std::array<UWBRxFrame, numDecas>& rxFrames);
};

#endif //__ANGLERECEIVERSTATE_H__
