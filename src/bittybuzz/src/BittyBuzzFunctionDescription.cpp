#include "BittyBuzzFunctionDescription.h"

BittyBuzzFunctionDescription::BittyBuzzFunctionDescription(const char* functionName) :
    m_functionName(functionName) {}

const std::array<std::tuple<const char*, FunctionDescriptionArgumentTypeDTO>,
                 FunctionDescriptionDTO::ARGUMENTS_MAX_SIZE>&
BittyBuzzFunctionDescription::getArguments() const {
    return m_argumentDescriptions;
}

uint16_t BittyBuzzFunctionDescription::getArgumentsLength() const {
    return m_argumentDescriptionsLength;
}

bool BittyBuzzFunctionDescription::addArgument(const char* argumentName,
                                               FunctionDescriptionArgumentTypeDTO argumentType) {
    if (m_argumentDescriptionsLength >= m_argumentDescriptions.size()) {
        return false;
    }

    m_argumentDescriptions[m_argumentDescriptionsLength++] =
        std::make_tuple(argumentName, argumentType);
    return true;
}
