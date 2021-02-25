#include "BittyBuzzContainer.h"
#include <bsp/SettingsContainer.h>
#include <logger/LoggerContainer.h>
#include <message-handler/MessageHandlerContainer.h>

BittyBuzzMessageHandler& BittyBuzzContainer::getBBZMessageHandler() {
    static BittyBuzzMessageHandler s_bbzMessageHandler(
        getBBZFunctionRegister(), MessageHandlerContainer::getBuzzMsgQueue(),
        MessageHandlerContainer::getHostMsgQueue(), MessageHandlerContainer::getRemoteMsgQueue(),
        SettingsContainer::getUUID(), LoggerContainer::getLogger());
    return s_bbzMessageHandler;
}

BittyBuzzFunctionRegister& BittyBuzzContainer::getBBZFunctionRegister() {
    static BittyBuzzFunctionRegister s_bbzFunctionRegister;
    return s_bbzFunctionRegister;
}
