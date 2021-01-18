#ifndef __TCPClientWrapperWRAPPER_H_
#define __TCPClientWrapperWRAPPER_H_

#include "bsp/ITCPClient.h"
#include <logger/ILogger.h>
#include <netinet/in.h>

/**
 *@brief A TCP client wrapper that is as a common socket between platforms.
 **/
class TCPClientWrapper : public ITCPClient {
  public:
    TCPClientWrapper(ITCPClient& client) : m_client(client){};

    ~TCPClientWrapper() override { TCPClientWrapper::close(); };

    int32_t receive(uint8_t* data, uint16_t length) override {
        return m_client.receive(data, length);
    }

    int32_t send(const uint8_t* data, uint16_t length) override {
        return m_client.send(data, length);
    }

    bool close() override { return m_client.close(); }

  private:
    ITCPClient& m_client;
};

#endif // __TCPClientWrapperWRAPPER_H_
