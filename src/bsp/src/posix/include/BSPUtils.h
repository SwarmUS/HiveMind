#ifndef __BSPUTILS_H__
#define __BSPUTILS_H__

#include <FreeRTOS.h>
#include <csignal>
#include <functional>
#include <task.h>
#include <thread>

/**
 * @brief Namespace used to store utility functions used in the ROS BSP
 */
namespace BSPUtils {
    /**
     * @brief Wrapper that runs a given function inside a seperate thread so sytem calls can be used
     * without interference from FreeRTOS
     * @tparam retType Return type of the provided function
     * @tparam F Template of the function to call
     * @tparam Args Variadic args template
     * @param func Function to call (could be a lambda)
     * @param args Arguments to pass to function
     * @return The result of the given function
     */
    template <typename retType, typename F, typename... Args>
    retType sysCallWrapper(F func, Args... args) {
        retType retVal;

        bool isFinished = false;

        vPortEnterCritical();

        std::thread t = std::thread([&]() {
            sigset_t mask;
            sigfillset(&mask);
            pthread_sigmask(SIG_BLOCK, &mask, NULL);

            retVal = std::invoke(func, args...);
            isFinished = true;
        });
        t.detach();

        vPortExitCritical();

        while (!isFinished) {
            vTaskDelay(10);
        }

        return retVal;
    }

} // namespace BSPUtils

#endif //__BSPUTILS_H__
