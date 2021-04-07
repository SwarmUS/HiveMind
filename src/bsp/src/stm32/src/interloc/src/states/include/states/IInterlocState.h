#ifndef __IINTERLOCSTATE_H__
#define __IINTERLOCSTATE_H__

class InterlocStateHandler;

class IInterlocState {
  public:
    virtual void process(InterlocStateHandler& context) = 0;
};

#endif //__IINTERLOCSTATE_H__
