#ifndef __HIVEMINDHOSTAPIREQUESTHANDLER_H_
#define __HIVEMINDHOSTAPIREQUESTHANDLER_H_

#include "IHiveMindHostApiRequestHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <interloc/IInterloc.h>
#include <logger/ILogger.h>

class HiveMindHostApiRequestHandler : public IHiveMindHostApiRequestHandler {
  public:
    HiveMindHostApiRequestHandler(const IBSP& bsp,
                                  ICircularQueue<MessageDTO>& hostQueue,
                                  const IInterloc& interloc,
                                  ILogger& logger);

    ~HiveMindHostApiRequestHandler() override = default;

    bool handleRequest(const MessageDTO& message) override;

  private:
    bool handleHiveMindHostApiRequest(uint16_t requestId,
                                      const MessageDTO& message,
                                      const HiveMindHostApiRequestDTO& request);

    const IBSP& m_bsp;
    ICircularQueue<MessageDTO>& m_hostQueue;
    const IInterloc& m_interloc;
    ILogger& m_logger;
};

#endif // __HIVEMINDHOSTAPIREQUESTHANDLER_H_
