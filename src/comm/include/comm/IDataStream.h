#ifndef __IDATASTREAM_H_
#define __IDATASTREAM_H_

#include <cstdint>

/**
 *@brief Class to represent a biderectional stream of data
 *
 **/
class IDataStream {

  public:
    virtual ~IDataStream() = default;

    /**
     *@brief Receives data from the stream
     *
     *@param [out] data buffer for the reception of the data
     *
     *@param [in] length maximum size of the data buffer
     *
     *@return the number of bytes received or -1 on error
     **/
    virtual int32_t receive(uint8_t* data, uint16_t length) = 0;

    /**
     *@brief Sends data to the stream
     *
     *@param [in] data buffer  to send to the remote server
     *
     *@param [in] length data size of the data buffer
     *
     *@return the number of bytes sent or -1 on error
     **/
    virtual int32_t send(const uint8_t* data, uint16_t length) = 0;
};

#endif // __IDATASTREAM_H_
