#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include "logger/ILogger.h"
#include <bbzmsg.h>
#include <bbzoutmsg.h>
#include <bbzringbuf.h>
#include <bbzvm.h>

UserFunctionRegister::UserFunctionRegister(uint8_t strId, bbzvm_funp functionPtr) :
    m_strId(strId), m_functionPtr(functionPtr) {}

bool BittyBuzzVm::step() {

    if (vm->state != BBZVM_STATE_ERROR) {
        // Handle incomming messages from remote
        m_neighborsManager.updateNeighbors();
        uint16_t messagesLength = m_messageHandler.messageQueueLength();
        for (uint16_t i = 0; i < messagesLength; i++) {
            if (!m_messageHandler.processMessage()) {
                m_logger.log(LogLevel::Warn,
                             "BBVM: Could not process message or the queue output is full");
            }
        }

        // Buzz
        bbzvm_process_inmsgs();
        BittyBuzzSystem::functionCall(__BBZSTRID_step);
        bbzvm_process_outmsgs();

        uint16_t queueSize = bbzoutmsg_queue_size();
        for (uint16_t i = 0; i < queueSize; i++) {

            BuzzMessageDTO msg(NULL, 0);
            bbzmsg_payload_t outPayload;

            bbzringbuf_clear(&outPayload);
            bbzringbuf_construct(&outPayload, msg.getRawPayload().data(), 1,
                                 BuzzMessageDTO::PAYLOAD_MAX_SIZE);
            bbzoutmsg_queue_first(&outPayload);

            msg.setRawPayloadLength(bbzringbuf_size(&outPayload));

            if (!m_messageService.sendBuzzMessage(msg)) {
                m_logger.log(LogLevel::Warn, "BBVM: Could not push buzz message");
                return false;
            }

            bbzoutmsg_queue_next();
        }

        return true;
    }

    return false;
}

bbzvm_state BittyBuzzVm::getSate() const { return vm->state; }
bbzvm_error BittyBuzzVm::getError() const { return vm->error; }
