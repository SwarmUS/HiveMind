#ifndef __BITTYBUZZMESSAGESERVICE_H_
#define __BITTYBUZZMESSAGESERVICE_H_

#include "IBittyBuzzMessageService.h"
#include <bbzmsg.h>
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class BittyBuzzMessageService : public IBittyBuzzMessageService {
  public:
    BittyBuzzMessageService(ICircularQueue<MessageDTO>& hostQueue,
                            ICircularQueue<MessageDTO>& remoteQueue,
                            IBSP& bsp,
                            ILogger& logger);

    ~BittyBuzzMessageService() override = default;

    bool callHostFunction(uint16_t hostId,
                          const char* functionName,
                          const FunctionCallArgumentDTO* args,
                          uint16_t argsLength) override;

    bool queueBuzzMessages() override;

  private:
    std::optional<std::reference_wrapper<BuzzMessageDTO>> getNextMessage();
    bool flush();

    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
    BuzzMessagesDTO m_messages;
    IBSP& m_bsp;
    ILogger& m_logger;
};

#endif // __BITTYBUZZMESSAGESERVICE_H_
