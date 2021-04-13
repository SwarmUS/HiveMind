#ifndef __IBITTYBUZZMATHFUNCTIONS_H_
#define __IBITTYBUZZMATHFUNCTIONS_H_

class IBittyBuzzMathFunctions {
  public:
    virtual ~IBittyBuzzMathFunctions() = default;

    /**@brief register the math table in the bbvm
     *@return true on succes, false if not */
    virtual bool registerMathTable() = 0;
};

#endif // __IBITTYBUZZMATHFUNCTIONS_H_
