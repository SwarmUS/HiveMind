#include "bittybuzz/BittyBuzz.h"
#include "bittybuzzsymbols.h"
#include <FreeRTOS.h>
#include <bbzvm.h>
#include <bsp/bsp_info.h>
#include <task.h>

bbzvm_t bbz_vm_obj;
uint8_t bbzmsg_buf[11];
bbzmsg_payload_t bbz_payload_buf;

uint8_t buf[4];
const uint8_t* bbzkilo_bcodeFetcher(bbzpc_t offset, uint8_t size) {
    for (bbzpc_t i = 0; i < size; i++) {
        buf[i] = bcode[i + offset];
    }
    return buf;
}

void bbzkilo_func_call(uint16_t strid) {
    bbzvm_pushs(strid);
    bbzheap_idx_t l = bbzvm_stack_at(0);
    bbzvm_pop();
    if (bbztable_get(bbz_vm_obj.gsyms, l, &l)) {
        bbzvm_pushnil(); // Push self table
        bbzvm_push(l);
        bbzvm_closure_call(0);
    }
}

void bbzkilo_err_receiver(bbzvm_error errcode) { printf("ERROR %d \n", errcode); }

BittyBuzz::BittyBuzz() {}

void BittyBuzz::init() {

    bbzvm_construct(2);
    bbzvm_set_error_receiver(bbzkilo_err_receiver);

    bbzvm_set_bcode(bbzkilo_bcodeFetcher, bcode_size);

    vm->state = BBZVM_STATE_READY;
    bbzkilo_func_call(__BBZSTRID_init);
}

void BittyBuzz::run() {

    while (vm->state != BBZVM_STATE_ERROR) {
        if (vm->state != BBZVM_STATE_ERROR) {
            bbzvm_process_inmsgs();
            bbzkilo_func_call(__BBZSTRID_step);
            bbzvm_process_outmsgs();
        }

        vTaskDelay(100);
    }
}
