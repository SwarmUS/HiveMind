#ifndef SYSTEMSTATES_H_
#define SYSTEMSTATES_H_

enum class ConnectionState { Booting = 0, Unconnected, USB, Ethernet, Error };

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
    bool m_remoteHandshaked = false;

    /**@brief if the handshake with the host is successfull*/
    bool m_hostHandshaked = false;

    /**@brief the state of the connection with the host*/
    ConnectionState m_connection = ConnectionState::Unconnected;

    /**@brief the overall state of the device*/
    DeviceState m_device = DeviceState::Ok;
};

#endif // SYSTEMSTATES_H_
