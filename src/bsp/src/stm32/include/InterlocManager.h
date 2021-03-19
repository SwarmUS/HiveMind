#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include "Decawave.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

class InterlocManager : public IInterlocManager {
  public:
    explicit InterlocManager(ILogger& logger);
    ~InterlocManager() override = default;

    void startInterloc() override;

  private:
    ILogger& m_logger;
    Decawave m_decaA;
    Decawave m_decaB;
};

#endif //__INTERLOCMANAGER_H__
