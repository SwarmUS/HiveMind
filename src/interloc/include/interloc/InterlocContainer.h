#ifndef __INTERLOCCONTAINER_H__
#define __INTERLOCCONTAINER_H__

#include "IInterloc.h"
#include "IInterlocMessageHandler.h"
#include <ThreadSafeQueue.h>

namespace InterlocContainer {
    IInterloc& getInterloc();

    IInterlocMessageHandler& getInterlocMessageHandler();

    /**
     *@brief get interloc position update msg queu
     *@return A reference to a queue with the id of robots with new positions */
    ThreadSafeQueue<uint16_t>& getInterlocPosUpdateQueue();
} // namespace InterlocContainer

#endif //__INTERLOCCONTAINER_H__
