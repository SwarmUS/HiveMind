#ifndef __MESSAGEHANDLERCONTAINER_H_
#define __MESSAGEHANDLERCONTAINER_H_

#include "HiveMindHostApiRequestHandler.h"
#include "MessageDispatcher.h"
#include "MessageSender.h"
#include <ThreadSafeQueue.h>
#include <pheromones/IHiveMindHostDeserializer.h>
#include <pheromones/IHiveMindHostSerializer.h>
#include <pheromones/MessageDTO.h>

namespace MessageHandlerContainer {

    /**
     *@brief create a message handler
     *@return A new message handler */
    HiveMindHostApiRequestHandler createHiveMindHostApiRequestHandler();

    /**
     *@brief create a message dispatcher
     *@return A new message dispatcher */
    MessageDispatcher createMessageDispatcher(IHiveMindHostDeserializer& deserializer,
                                              IHiveMindHostApiRequestHandler& hivemindApiReqHandler,
                                              IGreetSender& greetSender);

    /**
     *@brief get the buzz message queue
     *@return A reference to the buzz message queue */
    ThreadSafeQueue<MessageDTO>& getBuzzMsgQueue();

    /**
     *@brief get the host message queue
     *@return A reference to the host message queue */
    ThreadSafeQueue<MessageDTO>& getHostMsgQueue();

    /**
     *@brief get the remote message queue
     *@return A reference to the remote message queue */
    ThreadSafeQueue<MessageDTO>& getRemoteMsgQueue();

    /**
     *@brief get the interloc message queue
     *@return A reference to the interloc message queue */
    ThreadSafeQueue<MessageDTO>& getInterlocMsgQueue();

    /**
     *@brief get interloc position update msg queu
     *@return A reference to a queue with the id of robots with new positions */
    ThreadSafeQueue<uint16_t>& getInterlocPosUpdateQueue();

} // namespace MessageHandlerContainer

#endif // __MESSAGEHANDLERCONTAINER_H_
