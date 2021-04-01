#ifndef __HIVEMINDHOSTAPIREQUESTHANDLER_H_
#define __HIVEMINDHOSTAPIREQUESTHANDLER_H_

#include "IHiveMindHostApiRequestHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class HiveMindHostApiRequestHandler : public IHiveMindHostApiRequestHandler {
  public:
    HiveMindHostApiRequestHandler(const IBSP& bsp, ILogger& logger);

    ~HiveMindHostApiRequestHandler() override = default;

    HiveMindHostApiResponseDTO handleRequest(const HiveMindHostApiRequestDTO& request) override;

  private:
    const IBSP& m_bsp;
    ILogger& m_logger;
};

#endif // __HIVEMINDHOSTAPIREQUESTHANDLER_H_
