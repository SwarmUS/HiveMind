#ifndef __GREETER_H_
#define __GREETER_H_

#include "IGreeter.h"
#include <bsp/IBSP.h>
#include <common/IProtobufStream.h>

class Greeter : public IGreeter {
  public:
    Greeter(IProtobufStream& stream, IBSP& bsp);

    bool greet() override;

  private:
    IProtobufStream& m_stream;
    IBSP& m_bsp;
};

#endif // __GREETER_H_
