#ifndef __MESSAGEHANDLERCONTAINER_H_
#define __MESSAGEHANDLERCONTAINER_H_

#include "ThreadSafeQueue.h"
#include <hivemind-host/MessageDTO.h>

namespace MessageHandlerContainer {

    /**
     *@brief get the buzz message queue
     *
     *@return A reference to the buzz message queue
     **/
    ThreadSafeQueue<MessageDTO>& getBuzzMsgQueue();

    /**
     *@brief get the host message queue
     *
     *@return A reference to the host message queue
     **/
    ThreadSafeQueue<MessageDTO>& getHostMsgQueue();

    /**
     *@brief get the remote message queue
     *
     *@return A reference to the remote message queue
     **/
    ThreadSafeQueue<MessageDTO>& getRemoteMsgQueue();
} // namespace MessageHandlerContainer

#endif // __MESSAGEHANDLERCONTAINER_H_
