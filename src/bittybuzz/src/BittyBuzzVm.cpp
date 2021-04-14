#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/BittyBuzzSystem.h"
#include "logger/ILogger.h"
#include <bbzmsg.h>
#include <bbzoutmsg.h>
#include <bbzringbuf.h>
#include <bbzvm.h>

BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBittyBuzzStringResolver& stringResolver,
                         IBittyBuzzMessageHandler& messageHandler,
                         IBittyBuzzClosureRegister& closureRegister,
                         IBittyBuzzMessageService& messageService,
                         IBittyBuzzNeighborsManager& neighborsManager,
                         IBSP& bsp,
                         ILogger& logger,
                         IUserInterface& ui) :
    m_bytecode(bytecode),
    m_bsp(bsp),
    m_messageHandler(messageHandler),
    m_messageService(messageService),
    m_neighborsManager(neighborsManager),
    m_logger(logger),
    m_ui(ui) {
    // Init global variable
    vm = &m_bbzVm;
    BittyBuzzSystem::g_logger = &logger;
    BittyBuzzSystem::g_ui = &ui;
    BittyBuzzSystem::g_stringResolver = &stringResolver;
    BittyBuzzSystem::g_closureRegister = &closureRegister;
    BittyBuzzSystem::g_messageService = &messageService;
    BittyBuzzSystem::g_bsp = &bsp;
}

bool BittyBuzzVm::init(const BittyBuzzUserFunctionRegister* functions,
                       uint32_t functionsLength,
                       IBittyBuzzLib& bbzLibs) {

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());

    bbzLibs.registerLibs();

    // Function registration
    for (uint32_t i = 0; i < functionsLength; i++) {
        bbzvm_function_register(functions[i].getStringId(), functions[i].getFunctionPointer());
    }

    // Execute the global part of the script
    while (vm->state == BBZVM_STATE_READY) {
        bbzvm_step();
    }

    // Verify that the registration and startup was successfull
    if (vm->state == BBZVM_STATE_ERROR) {
        return false;
    }

    // Start init
    vm->state = BBZVM_STATE_READY;
    BittyBuzzSystem::functionCall(__BBZSTRID_init);

    // Verify that the initialization was successfull
    return vm->state == BBZVM_STATE_READY;
}

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
