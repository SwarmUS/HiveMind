#include "states/SendFinalState.h"
#include "interloc/InterlocStateHandler.h"
#include <Task.h>
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>

SendFinalState::SendFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendFinalState::process(InterlocStateHandler& context) {
    if (auto deca = m_decawaves.getMasterAntenna()) {
        uint64_t finalTxTime =
            context.getTimeManager().getFinalTxTs(context.getPreviousFrameStartTs());

        // Get the timestamp at which it will really be sent
        uint64_t finalTxTs = deca->get().getTxTimestampFromDelayedTime(finalTxTime);

        // TODO: ADD DESTINATION ID
        context.constructUWBHeader(0x01, UWBMessages::DATA, UWBMessages::TWR_FINAL,
                                   (uint8_t*)(&m_finalMsg), sizeof(m_finalMsg));

        context.getTWR().constructFinal(&m_finalMsg, finalTxTs);

        deca->get().transmitDelayed((uint8_t*)(&m_finalMsg), sizeof(UWBMessages::TWRFinal),
                                    finalTxTime);
        context.setState(InterlocStates::ANGLE_SENDER, InterlocEvent::NO_EVENT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}
