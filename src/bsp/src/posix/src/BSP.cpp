#include "BSP.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <sstream>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() = default;

/**
 * @brief Task that kills FreeRTOS when ROS node is stopped
 */
void rosWatcher(void* param) {
    (void)param;
    const int loopRate = 100;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        vTaskDelay(loopRate);
    }

    vTaskEndScheduler();
}

void BSP::initChip(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind");

    m_rosNodeHandle = new ros::NodeHandle("~");

    xTaskCreate(rosWatcher, "ros_watch", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,
                NULL);
}
