#ifndef __BITTYBUZZMESSAGESERVICE_H_
#define __BITTYBUZZMESSAGESERVICE_H_

#include "IBittyBuzzMessageService.h"
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class BittyBuzzMessageService : public IBittyBuzzMessageService {
  public:
    BittyBuzzMessageService(ICircularQueue<MessageDTO>& hostQueue,
                            ICircularQueue<MessageDTO>& remoteQueue,
                            uint16_t bspuuid,
                            ILogger& logger);

    ~BittyBuzzMessageService() override = default;

    bool callFunction(uint16_t id,
                      const char* functionName,
                      const FunctionCallArgumentDTO* args,
                      uint16_t argsLength) override;

  private:
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
    const uint16_t m_uuid;
    ILogger& m_logger;
};

#endif // __BITTYBUZZMESSAGESERVICE_H_
