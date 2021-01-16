#ifndef __ITCPCLIENT_H_
#define __ITCPCLIENT_H_

#include <cstdint>

// TODO: should we specify that it's a tcp socket? is it necessary to know that it's a client?
/**
 *@brief class to manage a tcp client socket */
class ITCPClient {
  public:
    virtual ~ITCPClient() = default;

    /**
     *@brief Receives data from the remote server
     *
     *@param [out] data buffer for the reception of the data
     *
     *@param [in] length maximum size of the data buffer
     **/
    virtual void receive(uint8_t* data, uint16_t length) = 0;

    /**
     *@brief Sends data to the remote server
     *
     *@param [in] data buffer  to send to the remote server
     *
     *@param [in] length data size of the data buffer
     **/
    virtual void send(const uint8_t data, uint16_t length) = 0;

    /**
     *@brief Closes the socket
     *
     *@return true if the operation was successful, false if not (i.e. the socket was already
     *closed)
     **/
    virtual bool close() = 0;
};

#endif // __ITCPCLIENT_H_
