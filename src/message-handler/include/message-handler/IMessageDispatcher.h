#ifndef __IMESSAGEDISPATCHER_H_
#define __IMESSAGEDISPATCHER_H_

/**
 *@brief A class to manage message dispatching to queues, sending messages at the right place*/
class IMessageDispatcher {

  public:
    virtual ~IMessageDispatcher() = default;

    /**
     *@brief deserialize a message and dispatches it in the appropriate queue
     *
     *@return true if the serialization and the dispatch was successful, false if any of those were
     *not*/
    virtual bool deserializeAndDispatch() = 0;
};
#endif // __IMESSAGEDISPATCHER_H_
