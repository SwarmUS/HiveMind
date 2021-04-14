#include "BittyBuzzLib.h"
#include "BittyBuzzLibMemberRegister.h"
#include "BittyBuzzMathFunctions.h"
#include <bbzvm.h>

template <typename Container>
BittyBuzzLib<Container>::BittyBuzzLib(int16_t libTableId, const Container& container) :
    m_libTableId(libTableId), m_container(container) {}

template <typename Container>
void BittyBuzzLib<Container>::registerLibs() {
    bbzvm_pusht(); // "table for lib"

    // Register constants in the lib
    for (BittyBuzzLibMemberRegister& libRegister : m_container) {
        std::variant<bbzvm_funp, float, int16_t> value = m_container.getValue();
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
    }

    // Register global symbol with the table
    bbzvm_gsym_register(m_libTableId, bbzvm_stack_at(0));
    bbzvm_pop(); // Pop table from stack
}
