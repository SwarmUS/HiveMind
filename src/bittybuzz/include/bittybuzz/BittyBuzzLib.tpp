
#ifndef __BITTYBUZZSTDLIB_TPP_
#define __BITTYBUZZSTDLIB_TPP_

#include "BittyBuzzLib.h"
#include "BittyBuzzLibMemberRegister.h"
#include "BittyBuzzMathFunctions.h"
#include <bbzvm.h>

template <typename Container>
BittyBuzzLib<Container>::BittyBuzzLib(uint16_t libTableId, const Container& container) :
    m_libTableId(libTableId), m_container(container) {}

template <typename Container>
BittyBuzzLib<Container>::BittyBuzzLib(const Container& container) :
    m_libTableId(std::nullopt), m_container(container) {}

template <typename Container>
bool BittyBuzzLib<Container>::registerLib() {
    return m_libTableId ? registerLibTable() : registerLibGlobal();
}

template <typename Container>
bool BittyBuzzLib<Container>::registerLibGlobal() {

    // Register constants in the lib
    for (const BittyBuzzLibMemberRegister& libRegister : m_container) {
        std::variant<bbzvm_funp, float, int16_t> value = libRegister.getValue();
        uint16_t strId = libRegister.getStringId();
        if (const bbzvm_funp* functionPointer = std::get_if<bbzvm_funp>(&value)) {
            bbzvm_function_register((int16_t)strId, *functionPointer);
        } else if (const float* floatVal = std::get_if<float>(&value)) {
            bbzvm_pushf(bbzfloat_fromfloat(*floatVal));
            bbzvm_gsym_register(strId, bbzvm_stack_at(0));
            bbzvm_pop();
        } else if (const int16_t* intVal = std::get_if<int16_t>(&value)) {
            bbzvm_pushi(*intVal);
            bbzvm_gsym_register(strId, bbzvm_stack_at(0));
            bbzvm_pop();
        }
        bbzvm_assert_state(false);
    }
    return true;
}

template <typename Container>
bool BittyBuzzLib<Container>::registerLibTable() {

    if (!m_libTableId) {
        return false;
    }

    bbzvm_pusht(); // table for lib

    // Register constants in the lib
    for (const BittyBuzzLibMemberRegister& libRegister : m_container) {
        std::variant<bbzvm_funp, float, int16_t> value = libRegister.getValue();
        uint16_t strId = libRegister.getStringId();
        if (const bbzvm_funp* functionPointer = std::get_if<bbzvm_funp>(&value)) {
            bbztable_add_function(strId, *functionPointer);
        } else if (const float* floatVal = std::get_if<float>(&value)) {
            bbzheap_idx_t floatHeapIdx = bbzfloat_new(bbzfloat_fromfloat(*floatVal));
            bbztable_add_data(strId, floatHeapIdx);
        } else if (const int16_t* intVal = std::get_if<int16_t>(&value)) {
            bbzheap_idx_t intHeapIdx = bbzint_new(*intVal);
            bbztable_add_data(strId, intHeapIdx);
        }
        bbzvm_assert_state(false);
    }

    // Register global symbol with the table
    bbzvm_gsym_register(m_libTableId.value(), bbzvm_stack_at(0));
    bbzvm_pop(); // Pop table from stack
    bbzvm_assert_state(false);
    return true;
}

#endif // __BITTYBUZZSTDLIB_TPP_
