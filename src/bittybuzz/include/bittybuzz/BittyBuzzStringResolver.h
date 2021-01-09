
#ifndef __BITTYBUZZSTRINGRESOLVER_H_
#define __BITTYBUZZSTRINGRESOLVER_H_

#include "IBittyBuzzStringResolver.h"
#include "logger/ILogger.h"
#include <cstdint>
#include <utility>

class BittyBuzzStringResolver : public IBittyBuzzStringResolver {
  public:
    BittyBuzzStringResolver(const std::pair<const uint16_t, const char*>* stringArray,
                            uint16_t arrayLength,
                            uint16_t offset,
                            const ILogger& logger);

    ~BittyBuzzStringResolver() override = default;

    std::optional<const char*> getString(uint16_t stringId) override;

  private:
    const ILogger& m_logger;
    const std::pair<const uint16_t, const char*>* m_stringArray;
    const uint16_t m_arrayLength;
    const uint16_t m_offset;
};

#endif // __BITTYBUZZSTRINGRESOLVER_H_
