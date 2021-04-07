#ifndef __IINTERLOCSTATE_H__
#define __IINTERLOCSTATE_H__

#include <interloc/InterlocManager.h>

class IInterlocState {
  public:
    virtual void enter(InterlocManager& context) = 0;
    virtual void process(InterlocManager& context) = 0;
    virtual void exit(InterlocManager& context) = 0;
};

#endif //__IINTERLOCSTATE_H__
