#include "bittybuzz/BittyBuzzStringResolver.h"

BittyBuzzStringResolver::BittyBuzzStringResolver(
    const std::pair<const uint16_t, const char*>* stringArray,
    uint16_t arrayLength,
    uint16_t stringIdOffset,
    const ILogger& logger) :
    m_logger(logger),
    m_stringArray(stringArray),
    m_arrayLength(arrayLength),
    m_stringIdOffset(stringIdOffset) {}

std::optional<const char*> BittyBuzzStringResolver::getString(uint16_t stringId) {
    int32_t stringIdIndex = stringId - m_stringIdOffset;

    if (stringIdIndex >= 0 && stringIdIndex < m_arrayLength) {

        // TODO: Is it really necessary?
        if (stringId != m_stringArray[stringIdIndex].first) {
            m_logger.log(LogLevel::Warn, "String resolver array was corrupted");
            return {};
        }
        return m_stringArray[stringIdIndex].second;
    }

    m_logger.log(LogLevel::Warn, "Obtained stringid out of bound");
    return {};
}
