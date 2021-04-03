#ifndef __ICOMMINTERFACE_H_
#define __ICOMMINTERFACE_H_

#include <pheromones/IProtobufStream.h>

class ICommInterface : public IProtobufStream {
  public:
    virtual ~ICommInterface() = default;

    /**
     * @brief Tells if interface is connected and functionning
     * @return true if connected, false otherwise */
    virtual bool isConnected() const = 0;
};

#endif // __ICOMMINTERFACE_H_
