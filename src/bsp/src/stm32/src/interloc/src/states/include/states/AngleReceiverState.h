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
    uint16_t m_timeoutHundredMicros = 0;
    volatile bool m_aborted = false;

    void saveAngleData(BspInterlocRawAngleData& data, uint32_t frameIndex);
    bool readAngleFrameContinuousMode();
    bool readAngleFrameNormalMode(const InterlocStateHandler& context,
                                  uint32_t angleId,
                                  uint16_t timeoutUs);
    bool waitReceptionOrTimeout();
    bool verifyDataValid();

    void abortRx();

    void timerCallback();
    static void staticTimerCallback(void* context);
};

#endif //__ANGLERECEIVERSTATE_H__
