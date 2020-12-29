#ifndef __BITTYBUZZSYSTEM_H_
#define __BITTYBUZZSYSTEM_H_

#include <bbzvm.h>
#include <logger/ILogger.h>

namespace bbz_system {

    extern const ILogger* logger;

    void functionCall(uint16_t strid);

    void errorReceiver(bbzvm_error errcode);

} // namespace bbz_system

#endif // __BITTYBUZZSYSTEM_H_
