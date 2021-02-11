#include "MessageHandlerContainer.h"
#include <cpp-common/CircularQueueStack.h>

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getBuzzMsgQueue() {
    static CircularQueueStack<MessageDTO, 8> s_buzzMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_buzzMsgThreadQueue(s_buzzMsgQueue);

    return s_buzzMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getHostMsgQueue() {
    static CircularQueueStack<MessageDTO, 8> s_hostMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_hostMsgThreadQueue(s_hostMsgQueue);

    return s_hostMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getRemoteMsgQueue() {
    static CircularQueueStack<MessageDTO, 8> s_remoteMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_remoteMsgThreadQueue(s_remoteMsgQueue);

    return s_remoteMsgThreadQueue;
}
