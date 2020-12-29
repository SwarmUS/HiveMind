#include "bittybuzz/BittyBuzzVm.h"
#include "BittyBuzzSystem.h"
#include "BittyBuzzUserFunctions.h"
#include <FreeRTOS.h>
#include <bbzvm.h>
#include <task.h>
#include <util/bbzstring.h>

extern "C" {
#include <main_bytecode.h>
}

bbzvm_t bbz_vm_obj;
uint8_t bbzmsg_buf[11];
bbzmsg_payload_t bbz_payload_buf;

BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode,
                         const IBSP& bsp,
                         const ILogger& logger) :
    m_bytecode(bytecode), m_bsp(bsp), m_logger(logger) {
    // Init global variable
    vm = &bbz_vm_obj;
    bbz_system::logger = &logger;

    // Init vm
    bbzvm_construct(m_bsp.getUUId());
    bbzvm_set_error_receiver(bbz_system::errorReceiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());
    bbzringbuf_construct(&bbz_payload_buf, bbzmsg_buf, 1, 11);

    // Function registration
    bbzvm_function_register(BBZSTRING_ID(logInt), bbz_user_functions::logInt);

    vm->state = BBZVM_STATE_READY;
    bbz_system::functionCall(__BBZSTRID_init);
}

bool BittyBuzzVm::step() {

    if (vm->state != BBZVM_STATE_ERROR) {
        bbzvm_process_inmsgs();
        bbz_system::functionCall(__BBZSTRID_step);
        bbzvm_process_outmsgs();
        return true;
    }

    return false;
}

bbzvm_state BittyBuzzVm::getSate() const { return vm->state; }
bbzvm_error BittyBuzzVm::getError() const { return vm->error; }
