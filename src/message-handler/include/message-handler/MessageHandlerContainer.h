#ifndef __MESSAGEHANDLERCONTAINER_H_
#define __MESSAGEHANDLERCONTAINER_H_

#include "HiveConnectHiveMindApiMessageHandler.h"
#include "HiveMindHostApiRequestHandler.h"
#include "MessageDispatcher.h"
#include "MessageSender.h"
#include <NotificationQueue.h>
#include <ThreadSafeQueue.h>
#include <pheromones/IHiveMindHostDeserializer.h>
#include <pheromones/IHiveMindHostSerializer.h>
#include <pheromones/MessageDTO.h>

namespace MessageHandlerContainer {

    /**
     *@brief create a message handler
     *@return A new message handler */
    HiveConnectHiveMindApiMessageHandler createHiveConnectHiveMindApiMessageHandler();

    /**
     *@brief create a request handler
     *@return A new request handler */
    HiveMindHostApiRequestHandler createHiveMindHostApiRequestHandler();

    /**
     *@brief create a message dispatcher
     *@return A new message dispatcher */
    MessageDispatcher createMessageDispatcher(
        IHiveMindHostDeserializer& deserializer,
        IHiveMindHostApiRequestHandler& hivemindApiReqHandler,
        IHiveConnectHiveMindApiMessageHandler& hiveconnectApiMessageHandler,
        IGreetSender& greetSender);

    MessageDispatcher& createHostDispatcher();
    MessageDispatcher& createRemoteDispatcher();
    IHiveMindHostSerializer& createHostSerializer();
    IHiveMindHostSerializer& createRemoteSerializer();

    /**
     *@brief get the buzz message queue
     *@return A reference to the buzz message queue */
    ThreadSafeQueue<MessageDTO>& getBuzzMsgQueue();

    /**
     *@brief get the host message queue
     *@return A reference to the host message queue */
    NotificationQueue<MessageDTO>& getHostMsgQueue();

    /**
     *@brief get the remote message queue
     *@return A reference to the remote message queue */
    NotificationQueue<MessageDTO>& getRemoteMsgQueue();

    /**
     *@brief get the interloc message queue
     *@return A reference to the interloc message queue */
    NotificationQueue<MessageDTO>& getInterlocMsgQueue();

} // namespace MessageHandlerContainer

#endif // __MESSAGEHANDLERCONTAINER_H_
