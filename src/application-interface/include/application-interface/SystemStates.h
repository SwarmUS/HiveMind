#ifndef SYSTEMSTATES_H_
#define SYSTEMSTATES_H_

enum class ConnectionState { Unconnected = 0, Booting, USBHost, EthernetHost, Error };

enum class DeviceState {
    Ok = 0,
    ErrorVMInstr,
    ErrorVMStack,
    ErrorVMLnum,
    ErrorVMPc,
    ErrorVMFlist,
    ErrorVMType,
    ErrorVMOutofrange,
    ErrorVMNotimpl,
    ErrorVMRet,
    ErrorVMString,
    ErrorVMSwarm,
    ErrorVMVstig,
    ErrorVMMem,
    ErrorVMMath
};

struct SystemStates {
    /**@brief if the handshake with the esp is successfull*/
    bool m_espHandshaked;

    /**@brief if the handshake with the host is successfull*/
    bool m_hostHandshaked;

    /**@brief the state of the connection with the host*/
    ConnectionState m_connection;

    /**@brief the overall state of the device*/
    DeviceState m_device;
};

#endif // SYSTEMSTATES_H_
