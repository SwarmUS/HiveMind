#include "BittyBuzzMessageService.h"
#include "pheromones/MessageDTO.h"
#include <bbzoutmsg.h>
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

bool BittyBuzzMessageService::queueBuzzMessages() {

    uint16_t queueSize = bbzoutmsg_queue_size();
    for (uint16_t i = 0; i < queueSize; i++) {

        bbzmsg_payload_t outPayload;
        std::optional<std::reference_wrapper<BuzzMessageDTO>> buzzMessageOpt = getNextMessage();
        if (!buzzMessageOpt) {
            return false;
        }
        BuzzMessageDTO& buzzMessage = buzzMessageOpt->get();

        bbzringbuf_clear(&outPayload);
        bbzringbuf_construct(&outPayload, buzzMessage.getRawPayload().data(), 1,
                             BuzzMessageDTO::PAYLOAD_MAX_SIZE);
        bbzoutmsg_queue_first(&outPayload);

        buzzMessage.setRawPayloadLength(bbzringbuf_size(&outPayload));

        bbzoutmsg_queue_next();
    }

    return flush();
}

std::optional<std::reference_wrapper<BuzzMessageDTO>> BittyBuzzMessageService::getNextMessage() {
    auto& messagesArray = m_messages.getRawMessages();

    if (m_messages.getMessagesLength() >= messagesArray.max_size()) {
        if (!flush()) {
            return {};
        }
    }

    uint16_t msgIdx = m_messages.getMessagesLength();

    m_messages.setRawMessagesLength(m_messages.getMessagesLength() + 1);
    return messagesArray[msgIdx];
}
bool BittyBuzzMessageService::flush() {
    if (m_messages.getMessagesLength() > 0) {
        VmMessageDTO vmMsgDto(m_messages);
        MessageDTO msgDto(m_bsp.getUUId(), 0, vmMsgDto);
        m_messages.setMessages((BuzzMessageDTO*)NULL, 0);
        return m_remoteQueue.push(msgDto);
    }
    return true;
}
