#ifndef __HIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_
#define __HIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_

#include "IHiveConnectHiveMindApiMessageHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>
#include <pheromones/MessageDTO.h>

class HiveConnectHiveMindApiMessageHandler : public IHiveConnectHiveMindApiMessageHandler {
  public:
    HiveConnectHiveMindApiMessageHandler(ICircularQueue<MessageDTO>& hostQueue,
                                         ICircularQueue<MessageDTO>& remoteQueue,
                                         ILogger& logger);

    ~HiveConnectHiveMindApiMessageHandler() override = default;

    bool handleMessage(uint16_t sourceId,
                       uint16_t destId,
                       const HiveConnectHiveMindApiDTO& message) override;

  private:
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
    ILogger& m_logger;
};

#endif // __HIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_
