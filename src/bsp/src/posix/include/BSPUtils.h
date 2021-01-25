#ifndef __BSPUTILS_H__
#define __BSPUTILS_H__

/**
 * @brief Namespace used to store utility functions used in the ROS BSP
 */
namespace BSPUtils {
    /**
     * @brief Blocks all system signals. Used around a syscall done inside of a FreeRTOS task
     */
    void blockSignals();

    /**
     * @brief Unblocks all system signals. Used around a syscall done inside of a FreeRTOS task
     */
    void unblockSignals();
} // namespace BSPUtils

#endif //__BSPUTILS_H__
