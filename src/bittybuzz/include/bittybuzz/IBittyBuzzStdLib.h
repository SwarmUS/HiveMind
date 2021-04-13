#ifndef __IBITTYBUZZSTDLIB_H_
#define __IBITTYBUZZSTDLIB_H_

/**@brief register the std lib functions in the bbvm via tables.
 * such as the maths */
class IBittyBuzzStdLib {
  public:
    virtual ~IBittyBuzzStdLib() = 0;

    /**@brief register the libs in the vm, each lib will have it's own table*/
    virtual void registerLibs() = 0;
};

#endif // __IBITTYBUZZSTDLIB_H_
