#ifndef __HIVEMINDHOSTAPIREQUESTHANDLER_H_
#define __HIVEMINDHOSTAPIREQUESTHANDLER_H_

#include "IHiveMindHostApiRequestHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class HiveMindHostApiRequestHandler : public IHiveMindHostApiRequestHandler {
  public:
    HiveMindHostApiRequestHandler(const IBSP& bsp,
                                  ICircularQueue<MessageDTO>& hostQueue,
                                  ILogger& logger);

    ~HiveMindHostApiRequestHandler() override = default;

    bool handleRequest(const MessageDTO& message) override;

  private:
    bool handleHiveMindHostApiRequest(const MessageDTO& message,
                                      const HiveMindHostApiRequestDTO& request);

    const IBSP& m_bsp;
    ICircularQueue<MessageDTO>& m_hostQueue;
    ILogger& m_logger;
};

#endif // __HIVEMINDHOSTAPIREQUESTHANDLER_H_
