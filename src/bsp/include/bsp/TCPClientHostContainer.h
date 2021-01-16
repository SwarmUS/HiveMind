#ifndef __TCPCLIENTHOSTCONTAINER_H_
#define __TCPCLIENTHOSTCONTAINER_H_

#include "ITCPClient.h"

/**@brief Manages the tcp client with the host*/
namespace TCPClientHostContainer {
    /**
     * @brief Return an instance of the platform dependent BSP.
     */
    ITCPClient& getClient();

} // namespace TCPClientHostContainer

#endif // __TCPCLIENTHOSTCONTAINER_H_
