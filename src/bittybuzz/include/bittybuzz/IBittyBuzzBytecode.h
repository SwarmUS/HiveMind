#ifndef __IBITTYBUZZBYTECODE_H_
#define __IBITTYBUZZBYTECODE_H_

#include <bbzvm.h>
#include <cstdint>

/**
 *@brief  Manages the BittyBuzz byte code */
class IBittyBuzzBytecode {
  public:
    virtual ~IBittyBuzzBytecode() = default;

    /**
     *@brief Obtain the function that fetches the BittyBuzz byte code
     **/
    virtual bbzvm_bcode_fetch_fun getBytecodeFetchFunction() const = 0;

    /**
     *@brief Get the size of the BittyBuzz byte code
     *
     *@return the size of the byte code */
    virtual uint16_t getBytecodeLength() const = 0;
};

#endif // __IBITTYBUZZBYTECODE_H_
