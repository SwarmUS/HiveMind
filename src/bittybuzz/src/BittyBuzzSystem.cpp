#include "bittybuzz/BittyBuzzSystem.h"

ILogger* BittyBuzzSystem::g_logger = NULL;
IUserInterface* BittyBuzzSystem::g_ui = NULL;
const IBittyBuzzStringResolver* BittyBuzzSystem::g_stringResolver = NULL;
IBittyBuzzClosureRegister* BittyBuzzSystem::g_closureRegister = NULL;
IBittyBuzzMessageService* BittyBuzzSystem::g_messageService = NULL;
IBSP* BittyBuzzSystem::g_bsp = NULL;

void BittyBuzzSystem::functionCall(uint16_t stringId) {
    bbzvm_pushs(stringId);
    bbzheap_idx_t l = bbzvm_stack_at(0);
    bbzvm_pop();
    if (bbztable_get(vm->gsyms, l, &l) == 1) {
        bbzvm_pushnil(); // Push self table
        bbzvm_push(l);
        bbzvm_closure_call(0);
        bbzvm_pop();
    }
}

void BittyBuzzSystem::errorReceiver(bbzvm_error errcode) {
    if (g_logger != NULL) {
        bbzvm_instr instr = (bbzvm_instr) * (*vm->bcode_fetch_fun)(vm->pc - 1, 1);
        g_logger->log(LogLevel::Error,
                                       "BittyBuzz virtual machine error, pc: %d, stackptr: %d, "
                                       "state: %s,  error code: %s, instruction: %s \n",
                                       vm->pc, vm->stackptr, getStateString(vm->state),
                                       getErrorString(errcode), getInstructionString(instr));
    }
}

const char* BittyBuzzSystem::getStateString(bbzvm_state state) {
    switch (state) {
    case BBZVM_STATE_NOCODE:
        return "BBZVM_STATE_NOCODE";
    case BBZVM_STATE_READY:
        return "BBZVM_STATE_READY";
    case BBZVM_STATE_STOPPED:
        return "BBZVM_STATE_STOPPED";
    case BBZVM_STATE_DONE:
        return "BBZVM_STATE_DONE";
    case BBZVM_STATE_ERROR:
        return "BBZVM_STATE_ERROR";
    default:
        return "BBZVM_STATE_UNKOWN";
    }
}

const char* BittyBuzzSystem::getErrorString(bbzvm_error error) {
    switch (error) {
    case BBZVM_ERROR_NONE:
        return "BBZVM_ERROR_NONE";
    case BBZVM_ERROR_INSTR:
        return "BBZVM_ERROR_INSTR";
    case BBZVM_ERROR_STACK:
        return "BBZVM_ERROR_STACK";
    case BBZVM_ERROR_LNUM:
        return "BBZVM_ERROR_LNUM";
    case BBZVM_ERROR_PC:
        return "BBZVM_ERROR_PC";
    case BBZVM_ERROR_FLIST:
        return "BBZVM_ERROR_FLIST";
    case BBZVM_ERROR_TYPE:
        return "BBZVM_ERROR_TYPE";
    case BBZVM_ERROR_OUTOFRANGE:
        return "BBZVM_ERROR_OUTOFRANGE";
    case BBZVM_ERROR_NOTIMPL:
        return "BBZVM_ERROR_NOTIMPL";
    case BBZVM_ERROR_RET:
        return "BBZVM_ERROR_RET";
    case BBZVM_ERROR_STRING:
        return "BBZVM_ERROR_STRING";
    case BBZVM_ERROR_SWARM:
        return "BBZVM_ERROR_SWARM";
    case BBZVM_ERROR_VSTIG:
        return "BBZVM_ERROR_VSTIG";
    case BBZVM_ERROR_MEM:
        return "BBZVM_ERROR_MEM";
    case BBZVM_ERROR_MATH:
        return "BBZVM_ERROR_MATH";
    default:
        return "BBZVM_ERROR_UNKOWN";
    }
}

const char* BittyBuzzSystem::getInstructionString(bbzvm_instr instruction) {
    switch (instruction) {
    case BBZVM_INSTR_NOP:
        return "BBZVM_INSTR_NOP";
    case BBZVM_INSTR_DONE:
        return "BBZVM_INSTR_DONE";
    case BBZVM_INSTR_PUSHNIL:
        return "BBZVM_INSTR_PUSHNIL";
    case BBZVM_INSTR_DUP:
        return "BBZVM_INSTR_DUP";
    case BBZVM_INSTR_POP:
        return "BBZVM_INSTR_POP";
    case BBZVM_INSTR_RET0:
        return "BBZVM_INSTR_RET0";
    case BBZVM_INSTR_RET1:
        return "BBZVM_INSTR_RET1";
    case BBZVM_INSTR_ADD:
        return "BBZVM_INSTR_ADD";
    case BBZVM_INSTR_SUB:
        return "BBZVM_INSTR_SUB";
    case BBZVM_INSTR_MUL:
        return "BBZVM_INSTR_MUL";
    case BBZVM_INSTR_DIV:
        return "BBZVM_INSTR_DIV";
    case BBZVM_INSTR_MOD:
        return "BBZVM_INSTR_MOD";
    case BBZVM_INSTR_POW:
        return "BBZVM_INSTR_POW";
    case BBZVM_INSTR_UNM:
        return "BBZVM_INSTR_UNM";
    case BBZVM_INSTR_LAND:
        return "BBZVM_INSTR_LAND";
    case BBZVM_INSTR_LOR:
        return "BBZVM_INSTR_LOR";
    case BBZVM_INSTR_LNOT:
        return "BBZVM_INSTR_LNOT";
    case BBZVM_INSTR_BAND:
        return "BBZVM_INSTR_BAND";
    case BBZVM_INSTR_BOR:
        return "BBZVM_INSTR_BOR";
    case BBZVM_INSTR_BNOT:
        return "BBZVM_INSTR_BNOT";
    case BBZVM_INSTR_LSHIFT:
        return "BBZVM_INSTR_LSHIFT";
    case BBZVM_INSTR_RSHIFT:
        return "BBZVM_INSTR_RSHIFT";
    case BBZVM_INSTR_EQ:
        return "BBZVM_INSTR_EQ";
    case BBZVM_INSTR_NEQ:
        return "BBZVM_INSTR_NEQ";
    case BBZVM_INSTR_GT:
        return "BBZVM_INSTR_GT";
    case BBZVM_INSTR_GTE:
        return "BBZVM_INSTR_GTE";
    case BBZVM_INSTR_LT:
        return "BBZVM_INSTR_LT";
    case BBZVM_INSTR_LTE:
        return "BBZVM_INSTR_LTE";
    case BBZVM_INSTR_GLOAD:
        return "BBZVM_INSTR_GLOAD";
    case BBZVM_INSTR_GSTORE:
        return "BBZVM_INSTR_GSTORE";
    case BBZVM_INSTR_PUSHT:
        return "BBZVM_INSTR_PUSHT";
    case BBZVM_INSTR_TPUT:
        return "BBZVM_INSTR_TPUT";
    case BBZVM_INSTR_TGET:
        return "BBZVM_INSTR_TGET";
    case BBZVM_INSTR_CALLC:
        return "BBZVM_INSTR_CALLC";
    case BBZVM_INSTR_CALLS:
        return "BBZVM_INSTR_CALLS";
    case BBZVM_INSTR_PUSHF:
        return "BBZVM_INSTR_PUSHF";
    case BBZVM_INSTR_PUSHI:
        return "BBZVM_INSTR_PUSHI";
    case BBZVM_INSTR_PUSHS:
        return "BBZVM_INSTR_PUSHS";
    case BBZVM_INSTR_PUSHCN:
        return "BBZVM_INSTR_PUSHCN";
    case BBZVM_INSTR_PUSHCC:
        return "BBZVM_INSTR_PUSHCC";
    case BBZVM_INSTR_PUSHL:
        return "BBZVM_INSTR_PUSHL";
    case BBZVM_INSTR_LLOAD:
        return "BBZVM_INSTR_LLOAD";
    case BBZVM_INSTR_LSTORE:
        return "BBZVM_INSTR_LSTORE";
    case BBZVM_INSTR_LREMOVE:
        return "BBZVM_INSTR_LREMOVE";
    case BBZVM_INSTR_JUMP:
        return "BBZVM_INSTR_JUMP";
    case BBZVM_INSTR_JUMPZ:
        return "BBZVM_INSTR_JUMPZ";
    case BBZVM_INSTR_JUMPNZ:
        return "BBZVM_INSTR_JUMPNZ";
    default:
        return "BBZVM_INSTR_UNKOWN";
    }
}
