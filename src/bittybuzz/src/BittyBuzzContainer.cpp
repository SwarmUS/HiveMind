#include "BittyBuzzContainer.h"
#include <bsp/BSPContainer.h>
#include <bsp/SettingsContainer.h>
#include <logger/LoggerContainer.h>
#include <message-handler/MessageHandlerContainer.h>

BittyBuzzMessageHandler& BittyBuzzContainer::getBBZMessageHandler() {
    static BittyBuzzMessageHandler s_bbzMessageHandler(
        getBBZClosureRegister(), MessageHandlerContainer::getBuzzMsgQueue(),
        MessageHandlerContainer::getHostMsgQueue(), MessageHandlerContainer::getRemoteMsgQueue(),
        BSPContainer::getBSP(), LoggerContainer::getLogger());
    return s_bbzMessageHandler;
}

BittyBuzzClosureRegister& BittyBuzzContainer::getBBZClosureRegister() {
    static BittyBuzzClosureRegister s_bbzClosureRegister;
    return s_bbzClosureRegister;
}

BittyBuzzMessageService& BittyBuzzContainer::getBBZMessageService() {
    static BittyBuzzMessageService s_bbzMessageService(
        MessageHandlerContainer::getHostMsgQueue(), MessageHandlerContainer::getRemoteMsgQueue(),
        BSPContainer::getBSP(), LoggerContainer::getLogger());
    return s_bbzMessageService;
}

BittyBuzzNeighborsManager& BittyBuzzContainer::getBBZNeighborsManager() { static BittyBuzzNei }
