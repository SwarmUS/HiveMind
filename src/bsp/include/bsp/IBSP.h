#ifndef __IBSP_H_
#define __IBSP_H_

class IBSP {
  public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise the chip for usage. Needs to be called early in the program.
     */
    virtual void initChip() = 0;
};

#endif // __IBSP_H_
