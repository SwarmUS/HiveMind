#ifndef __ABSTRACTINTERLOCSTATE_H__
#define __ABSTRACTINTERLOCSTATE_H__

#include <interloc/Decawave.h>
#include <interloc/DecawaveArray.h>
#include <logger/ILogger.h>

class InterlocStateHandler;

class AbstractInterlocState {
  public:
    AbstractInterlocState(ILogger& logger, DecawaveArray& decawaves);

    virtual void process(InterlocStateHandler& context) = 0;

  protected:
    ILogger& m_logger;
    DecawaveArray& m_decawaves;
};

#endif //__ABSTRACTINTERLOCSTATE_H__
