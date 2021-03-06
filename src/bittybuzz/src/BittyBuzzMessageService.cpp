#include "BittyBuzzMessageService.h"

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
