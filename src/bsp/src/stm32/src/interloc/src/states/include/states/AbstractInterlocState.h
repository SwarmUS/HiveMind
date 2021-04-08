#ifndef __ABSTRACTINTERLOCSTATE_H__
#define __ABSTRACTINTERLOCSTATE_H__

#include <interloc/Decawave.h>
#include <interloc/DecawaveArray.h>
#include <logger/ILogger.h>

class InterlocManager;
class InterlocStateHandler;

class AbstractInterlocState {
  public:
    AbstractInterlocState(ILogger& logger,
                          InterlocManager& interlocManager,
                          DecawaveArray& decawaves);

    virtual void process(InterlocStateHandler& context) = 0;

  protected:
    ILogger& m_logger;
    InterlocManager& m_interlocManager;
    DecawaveArray& m_decawaves;
};

#endif //__ABSTRACTINTERLOCSTATE_H__
