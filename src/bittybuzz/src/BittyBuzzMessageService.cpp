#include "BittyBuzzMessageService.h"
#include "hivemind-host/MessageDTO.h"
#include <bbzringbuf.h>

BittyBuzzMessageService::BittyBuzzMessageService(ICircularQueue<MessageDTO>& hostQueue,
                                                 ICircularQueue<MessageDTO>& remoteQueue,
                                                 IBSP& bsp,
                                                 ILogger& logger) :
    m_hostQueue(hostQueue), m_remoteQueue(remoteQueue), m_bsp(bsp), m_logger(logger) {}

bool BittyBuzzMessageService::callHostFunction(uint16_t hostId,
                                               const char* functionName,
                                               const FunctionCallArgumentDTO* args,
                                               uint16_t argsLength) {
    FunctionCallRequestDTO fReq(functionName, args, argsLength);
    // Target of a usercall from buzz is always to a host
    UserCallRequestDTO uReq(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fReq);

    uint16_t uuid = m_bsp.getUUId();
    RequestDTO req(m_bsp.generateRandomNumber(), uReq);
    MessageDTO message(uuid, hostId, req);
    // Broadcast
    if (hostId == 0) {
        return m_hostQueue.push(message) && m_remoteQueue.push(message);
    }

    // Host
    if (hostId == uuid) {
        return m_hostQueue.push(message);
    }

    // Remote
    return m_remoteQueue.push(message);
}

bool BittyBuzzMessageService::sendBuzzMessage(const uint8_t* payload, uint16_t payloadLength) {
    if (payloadLength > BuzzMessageDTO::PAYLOAD_MAX_SIZE) {
        return false;
    }

    BuzzMessageDTO buzzMessage(payload, payloadLength);
    // destination = 0 for broadcast
    MessageDTO message(m_bsp.getUUId(), 0, buzzMessage);
    return m_remoteQueue.push(message);
}
