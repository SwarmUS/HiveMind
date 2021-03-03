#ifndef __HIVEMINDAPIREQUESTHANDLER_H_
#define __HIVEMINDAPIREQUESTHANDLER_H_

#include "IHiveMindApiRequestHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <logger/ILogger.h>

class HiveMindApiRequestHandler : public IHiveMindApiRequestHandler {
  public:
    HiveMindApiRequestHandler(const IBSP& bsp, ILogger& logger);

    ~HiveMindApiRequestHandler() override = default;

    HiveMindApiResponseDTO handleRequest(const HiveMindApiRequestDTO& request) override;

  private:
    const IBSP& m_bsp;
    ILogger& m_logger;
};

#endif // __HIVEMINDAPIREQUESTHANDLER_H_
