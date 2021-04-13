#ifndef __BITTYBUZZSTDLIB_H_
#define __BITTYBUZZSTDLIB_H_

#include "IBittyBuzzStdLib.h"

class BittyBuzzStdLib : public IBittyBuzzStdLib {
  public:
    ~BittyBuzzStdLib() override = default;

    void registerLibs() override;

  private:
    void registerMathLib();
};

#endif // __BITTYBUZZSTDLIB_H_
