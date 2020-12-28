#include "bittybuzz/BittyBuzzVm.h"
#include <FreeRTOS.h>
#include <bsp/bsp_info.h>
#include <task.h>
#include <util/bbzstring.h>

extern "C" {
#include <test_bytecode.h>
}

bbzvm_t bbz_vm_obj;
uint8_t bbzmsg_buf[11];
bbzmsg_payload_t bbz_payload_buf;

void bbz_func_call(uint16_t strid) {
    bbzvm_pushs(strid);
    bbzheap_idx_t l = bbzvm_stack_at(0);
    bbzvm_pop();
    if (bbztable_get(bbz_vm_obj.gsyms, l, &l)) {
        bbzvm_pushnil(); // Push self table
        bbzvm_push(l);
        bbzvm_closure_call(0);
    }
}

void bbz_err_receiver(bbzvm_error errcode) { printf("ERROR %d \n", errcode); }

void bbz_test_print() {
    bbzvm_assert_lnum(1);
    bbzobj_t* int_val = bbzheap_obj_at(bbzvm_locals_at(1));

    printf("Hello, test value: %d \n", int_val->i.value);
    bbzvm_ret0();
}

BittyBuzzVm::BittyBuzzVm(const IBittyBuzzBytecode& bytecode) : m_bytecode(bytecode) {

    vm = &bbz_vm_obj;
    bbzringbuf_construct(&bbz_payload_buf, bbzmsg_buf, 1, 11);

    // INIT
    bbzvm_construct(2);
    bbzvm_set_error_receiver(bbz_err_receiver);
    bbzvm_set_bcode(m_bytecode.getBytecodeFetchFunction(), m_bytecode.getBytecodeLength());

    // Function registration
    bbzvm_function_register(BBZSTRING_ID(print), bbz_test_print);

    vm->state = BBZVM_STATE_READY;
    bbz_func_call(__BBZSTRID_init);
}

bool BittyBuzzVm::step() {

    if (vm->state != BBZVM_STATE_ERROR) {
        bbzvm_process_inmsgs();
        bbz_func_call(__BBZSTRID_step);
        bbzvm_process_outmsgs();
        return true;
    }

    return false;
}

bbzvm_state BittyBuzzVm::getSate() const { return vm->state; }
bbzvm_error BittyBuzzVm::getError() const { return vm->error; }
