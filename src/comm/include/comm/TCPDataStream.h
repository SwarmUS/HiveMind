#ifndef __TCPDATASTREAM_H_
#define __TCPDATASTREAM_H_

#include "IDataStream.h"
#include <bsp/ITCPClient.h>
#include <logger/ILogger.h>

class TCPDataStream : public IDataStream {
  public:
    TCPDataStream(ITCPClient& client);
    ~TCPDataStream() override;

    int32_t receive(uint8_t* data, uint16_t length) override;

    int32_t send(const uint8_t* data, uint16_t length) override;

  private:
    ITCPClient& m_client;
};

#endif // __TCPDATASTREAM_H_
