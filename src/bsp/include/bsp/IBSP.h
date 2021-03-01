#ifndef __IBSP_H_
#define __IBSP_H_

#include <cstdint>

class IBSP {
  public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise the chip for usage. Needs to be called early in the program.
     */
    virtual void initChip(void* args) = 0;

    /**
     * @brief Returns the unique id associated with the board
     * */
    virtual uint16_t getUUId() const = 0;

    /**
     * @brief Returns a positive random number, use a hardware RNG if available
     * */
    virtual uint64_t generateRandomNumber() = 0;
};

/**
 *@brief A structure to wrap command line arguments used for certain BSPs
 **/
typedef struct {
    int m_argc;
    char** m_argv;
} CmdLineArgs;

#endif // __IBSP_H_
