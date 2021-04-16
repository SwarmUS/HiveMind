#ifndef __IBITTYBUZZLIB_H_
#define __IBITTYBUZZLIB_H_

/**@brief register the std lib functions in the bbvm via tables.
 * such as the maths */
class IBittyBuzzLib {
  public:
    virtual ~IBittyBuzzLib() = default;

    /**@brief register the library in the vm, each lib will have it's own table, if the lib table id
     * is 0, the data will be registered on the global scope */
    virtual bool registerLib() = 0;
};

#endif // __IBITTYBUZZLIB_H_
