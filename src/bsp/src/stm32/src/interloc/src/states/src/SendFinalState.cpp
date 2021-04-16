#include "states/SendFinalState.h"
#include "interloc/InterlocStateHandler.h"
#include <Task.h>
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>

SendFinalState::SendFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendFinalState::process(InterlocStateHandler& context) {
    uint64_t finalTxTime = context.getTimeManager().getFinalTxTs(context.getTWR().m_pollTxTs);
    // volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();

    // Get the timestamp at which it will really be sent
    uint64_t finalTxTs = m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(finalTxTime);

    // TODO: ADD DESTINATION ID
    context.constructUWBHeader(0x01, UWBMessages::DATA, UWBMessages::TWR_FINAL,
                               (uint8_t*)(&m_finalMsg), sizeof(m_finalMsg));

    context.getTWR().constructFinal(&m_finalMsg, finalTxTs);
    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)(&m_finalMsg),
                                                 sizeof(UWBMessages::TWRFinal), finalTxTime);

    // TODO: Replace by a blocking transmit instead
    Task::delay(1);
    context.setState(InterlocStates::WAIT_POLL, InterlocEvent::NO_EVENT);
}
