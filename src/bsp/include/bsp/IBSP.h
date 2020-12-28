#ifndef __IBSP_H_
#define __IBSP_H_

#include <cstdint>

class IBSP {
  public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise the chip for usage. Needs to be called early in the program.
     */
    virtual void initChip() = 0;

    /**
     * @brief Returns the unique id associated with the board
     * */
    virtual uint16_t getUUId() const = 0;
};

#endif // __IBSP_H_
