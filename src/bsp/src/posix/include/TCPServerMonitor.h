#ifndef __TCPSERVERMONITOR_H__
#define __TCPSERVERMONITOR_H__

#include <sys/select.h>

class TCPServerMonitor {
  public:
    TCPServerMonitor();
    ~TCPServerMonitor() = default;

    /**
     * Add the server's file descriptor to the watch list.
     * @param serverFd File descriptor of the server.
     */
    void addServer(int serverFd);

    /**
     * Add a connection to the watch list.
     * @param conn File descriptor of the connection.
     */
    void addConn(int conn);

    /**
     * Remove a connection from the watch list.
     * @param conn File descriptor of the connection.
     */
    void removeConn(int conn);

    /**
     * Wait for until there is incoming data on a connection.
     * @param conn File descriptor of the connection.
     * @return
     */
    bool waitIncoming(int conn);

  private:
    int m_serverFd;
    int m_maxFd;
    fd_set m_fileDescriptorSet;
};

#endif //__TCPSERVERMONITOR_H__
