#ifndef __GREETSENDER_H_
#define __GREETSENDER_H_

#include "IGreetSender.h"
#include "hivemind-host/MessageDTO.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>

class GreetSender : public IGreetSender {
  public:
    GreetSender(ICircularQueue<MessageDTO>& outputQueue, IBSP& bsp);

    ~GreetSender() override = default;

    bool sendGreet() override;

  private:
    ICircularQueue<MessageDTO>& m_outputQueue;
    IBSP& m_bsp;
};

#endif // __GREETSENDER_H_
