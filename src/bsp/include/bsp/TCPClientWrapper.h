#ifndef __TCPClientWrapperWRAPPER_H_
#define __TCPClientWrapperWRAPPER_H_

#include "bsp/ITCPClient.h"
#include <logger/ILogger.h>
#include <netinet/in.h>

class TCPClientWrapper : public ITCPClient {
  public:
    TCPClientWrapper(ITCPClient& client);

    ~TCPClientWrapper() override;

    int32_t receive(uint8_t* data, uint16_t length) override;

    int32_t send(const uint8_t* data, uint16_t length) override;

    bool close() override;


  private:
    const ITCPClient& m_client

};

#endif // __TCPClientWrapperWRAPPER_H_
