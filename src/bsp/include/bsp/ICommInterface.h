#ifndef __ICOMMINTERFACE_H_
#define __ICOMMINTERFACE_H_

#include <pheromones/IProtobufStream.h>

enum class ConnectionType { Ethernet, USB, SPI };

class ICommInterface : public IProtobufStream {
  public:
    virtual ~ICommInterface() = default;

    /**
     * @brief Tells if interface is connected and functionning
     * @return true if connected, false otherwise */
    virtual bool isConnected() const = 0;

    /**@brief Tells the type of the connection interface
     *@return the type of connection */
    virtual ConnectionType getType() const = 0;
};

#endif // __ICOMMINTERFACE_H_
