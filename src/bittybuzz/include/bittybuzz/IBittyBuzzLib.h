#ifndef __IBITTYBUZZLIB_H_
#define __IBITTYBUZZLIB_H_

/**@brief register the std lib functions in the bbvm via tables.
 * such as the maths */
class IBittyBuzzLib {
  public:
    virtual ~IBittyBuzzLib() = default;

    /**@brief register the libs in the vm, each lib will have it's own table*/
    virtual void registerLibs() = 0;
};

#endif // __IBITTYBUZZLIB_H_
