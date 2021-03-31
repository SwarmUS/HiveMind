#ifndef __INTERLOCCONTAINER_H__
#define __INTERLOCCONTAINER_H__

#include "IInterloc.h"
#include "IInterlocMessageHandler.h"
namespace InterlocContainer {
    IInterloc& getInterloc();

    IInterlocMessageHandler& getInterlocMessageHandler();
} // namespace InterlocContainer

#endif //__INTERLOCCONTAINER_H__
