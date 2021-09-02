#ifndef SYSTEMSTATES_H_
#define SYSTEMSTATES_H_

enum class ConnectionState {
    Unconnected = 0,
    Booting = 0,
    USBHost = 0,
    EthernetHost = 0,
    Error = 0,
};

enum class DeviceState { Ok = 0, Error = 1 };

struct SystemStates {
    bool m_espHandshaked;
    bool m_hostHandshaked;
    ConnectionState m_connection;
    DeviceState m_device;
};

#endif // SYSTEMSTATES_H_
