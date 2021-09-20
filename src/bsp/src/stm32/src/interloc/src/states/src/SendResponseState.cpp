#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocTimeManager.h>
#include <states/InterlocStateContainer.h>
#include <states/SendResponseState.h>

SendResponseState::SendResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendResponseState::process(InterlocStateHandler& context) {
    if (auto deca = m_decawaves.getMasterAntenna()) {
        // TODO: Set destination ID
        context.constructUWBHeader(0x01, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                                   (uint8_t*)&m_respMsg, sizeof(m_respMsg));
        m_respMsg.m_subFrameId = context.getSlotId();

        volatile uint64_t respTxTime =
            context.getTimeManager().getResponseTxTs(context.getPreviousFrameStartTs());

        deca->get().transmitDelayed((uint8_t*)&m_respMsg, sizeof(m_respMsg), respTxTime);

        // Get the real time at which the response was sent
        context.getTWR().m_responseTxTs = deca->get().getTxTimestampFromDelayedTime(respTxTime);

        context.setState(InterlocStates::WAIT_FINAL, InterlocEvent::NO_EVENT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}
