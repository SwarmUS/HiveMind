#ifndef __IBITTYBUZZSTRINGRESOLVER_H_
#define __IBITTYBUZZSTRINGRESOLVER_H_

#include <cstdint>
#include <optional>

/**
 *@brief Class that resolves a string from the bittybuzz virtual machine
 **/
class IBittyBuzzStringResolver {
  public:
    virtual ~IBittyBuzzStringResolver() = default;

    /**
     *@brief Get the string from the stringId
     **/
    virtual std::optional<const char*> getString(uint16_t stringId) const = 0;
};

#endif // __IBITTYBUZZSTRINGRESOLVER_H_
