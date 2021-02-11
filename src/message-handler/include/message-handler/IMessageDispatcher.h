#ifndef __IMESSAGEDISPATCHER_H_
#define __IMESSAGEDISPATCHER_H_

class IMessageDispatcher {

  public:
    virtual ~IMessageDispatcher() = default;

    virtual bool deserializeAndDispatch() = 0;
};
#endif // __IMESSAGEDISPATCHER_H_
