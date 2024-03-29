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
                         IUserUI& userUI,
                         IBSP& bsp,
                         ILogger& logger,
                         IUserInterface& ui) :
    m_bytecode(bytecode),
    m_bsp(bsp),
    m_messageHandler(messageHandler),
    m_messageService(messageService),
    m_neighborsManager(neighborsManager),
    m_closureRegister(closureRegister),
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
    BittyBuzzSystem::g_userUI = &userUI;
}

bool BittyBuzzVm::init(const std::reference_wrapper<IBittyBuzzLib>* bbzLibs,
                       uint32_t bbzLibsLength) {

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(BittyBuzzSystem::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());

    // Register lib
    for (uint32_t i = 0; i < bbzLibsLength; i++) {
        if (!bbzLibs[i].get().registerLib()) {
            return false;
        }
    }

    // Execute the global part of the script
    while (vm->state == BBZVM_STATE_READY) {
        bbzvm_step();
    }

    // Verify that the registration and startup was successfull
    bbzvm_assert_state(false);

    return vm->state == BBZVM_STATE_DONE;
}

bool BittyBuzzVm::start() {
    // Start init
    vm->state = BBZVM_STATE_READY;
    BittyBuzzSystem::functionCall(__BBZSTRID_init);

    // Verify that the initialization was successfull
    return vm->state == BBZVM_STATE_READY;
}

void BittyBuzzVm::stop() {
    // Start init
    vm->state = BBZVM_STATE_STOPPED;
}

void BittyBuzzVm::logDump(LogLevel logLevel) { BittyBuzzSystem::logVmDump(logLevel); }

void BittyBuzzVm::terminate() {
    // Destroy the vm
    m_bbzVm.state = BBZVM_STATE_STOPPED;
    bbzvm_destruct();
    // Clear registered closures
    m_closureRegister.clearClosures();
    // Clear all the pending messages since they will be out of date anyway
    m_messageHandler.clearMessages();
}

BBVMRet BittyBuzzVm::step() {

    if (vm->state == BBZVM_STATE_READY) {
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

        if (!m_messageService.queueBuzzMessages()) {
            return BBVMRet::OutMsgErr;
        }

        return vm->state == BBZVM_STATE_READY ? BBVMRet::Ok : BBVMRet::VmErr;
    }

    if (vm->state == BBZVM_STATE_STOPPED) {
        return BBVMRet::Stopped;
    }

    return BBVMRet::VmErr;
}

bbzvm_state BittyBuzzVm::getState() const { return vm->state; }

bbzvm_error BittyBuzzVm::getError() const { return vm->error; }

bbzvm_instr BittyBuzzVm::getInstruction() const {
    bbzvm_instr instr =
        (bbzvm_instr) *
        (*vm->bcode_fetch_fun)(vm->pc - 1,
                               1); // -1 since the PC was incremented before the error occured
    return instr;
}
