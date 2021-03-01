#include "BittyBuzzMessageService.h"

BittyBuzzMessageService::BittyBuzzMessageService(ICircularQueue<MessageDTO>& hostQueue,
                                                 ICircularQueue<MessageDTO>& remoteQueue,
                                                 uint16_t bspuuid,
                                                 ILogger& logger) :
    m_hostQueue(hostQueue), m_remoteQueue(remoteQueue), m_uuid(bspuuid), m_logger(logger) {}

bool BittyBuzzMessageService::callFunction(uint16_t id,
                                           const char* functionName,
                                           const FunctionCallArgumentDTO* args,
                                           uint16_t argsLength) {
    FunctionCallRequestDTO fReq(functionName, args, argsLength);
    // Target of a usercall from buzz is always to a host
    UserCallRequestDTO uReq(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, fReq);
    RequestDTO req(1, uReq);
    MessageDTO message(m_uuid, id, req);
    // Broadcast
    if (id == 0) {
        return m_hostQueue.push(message) && m_remoteQueue.push(message);
    }

    // Host
    if (id == m_uuid) {
        return m_hostQueue.push(message);
    }

    // Remote
    return m_remoteQueue.push(message);
}
