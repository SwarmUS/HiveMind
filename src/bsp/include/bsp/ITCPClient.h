#ifndef __ITCPCLIENT_H_
#define __ITCPCLIENT_H_

#include "ICommInterface.h"
#include <common/IProtobufStream.h>
#include <cstdint>

/**
 *@brief class to manage a tcp client socket */
class ITCPClient : public ICommInterface {
  public:
    virtual ~ITCPClient() = default;

    /**
     *@brief Receives data from the remote server
     *@param [out] data buffer for the reception of the data
     *@param [in] length maximum size of the data buffer
     *@return true if the operation was successful, false if not */
    virtual bool receive(uint8_t* data, uint16_t length) = 0;

    /**
     *@brief Sends data to the remote server
     *@param [in] data buffer  to send to the remote server
     *@param [in] length data size of the data buffer
     *@return true if the operation was successful, false if not */
    virtual bool send(const uint8_t* data, uint16_t length) = 0;

    /**
     * @brief Tells if the socket is connected and functionning
     * @return true if connected, false otherwise */
    virtual bool isConnected() const = 0;

    /**
     *@brief Closes the socket
     *@return true if the operation was successful, false if not (i.e. the socket was already
     *closed) */
    virtual bool close() = 0;
};

#endif // __ITCPCLIENT_H_
