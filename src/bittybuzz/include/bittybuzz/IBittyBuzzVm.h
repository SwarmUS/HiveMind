#ifndef __IBITTYBUZZVM_H_
#define __IBITTYBUZZVM_H_

class IBittyBuzzVm {
  public:
    virtual ~BittyBuzz() = default;

    /**
     * @brief Initialize the bittybuzz virtual machine and execute the init function in the buzz code
     * */
    virtual void init() = 0;

    /**
     * @brief Does one execution step in the virtual machine.  Thus execute the buzz code in the step function
     *
     * @return true if the operation was successfull, false if not.
     * */
    virtual bool step() = 0;


    /**
     * @brief Does one execution step in the virtual machine.  Thus execute the buzz code in the step function
     *
     * */
    virtual vmState getSate() = 0;
};


#endif // __IBITTYBUZZVM_H_
