#include "BSP.h"
#include "ros/ros.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <bsp/BSPContainer.h>
#include <hive_mind/ExampleMessage.h>
#include <sstream>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() {
    if (m_rosNodeHandle != NULL) {
        delete m_rosNodeHandle;
    }
}

/**
 * @brief Task that kills FreeRTOS when ROS node is stopped
 */
void rosWatcher(void* param) {
    (void)param;
    const int loopRate = 100;

    while (ros::ok()) {
        ros::spinOnce();
        vTaskDelay(loopRate);
    }

    vTaskEndScheduler();
}

void exampleTopicPublish(void* param) {
    (void)param;
    const int loopRate = 4000;
    hive_mind::ExampleMessage msg = hive_mind::ExampleMessage();
    msg.number = 0;
    msg.text = "Hello World";

    BSP* bsp = (BSP*)(BSPContainer::getBSP());
    ros::Publisher publisher =
        bsp->getRosNodeHandle()->advertise<hive_mind::ExampleMessage>("exampleTopic", 1000);

    while (true) {
        publisher.publish(msg);
        msg.number++;

        vTaskDelay(loopRate);
    }
}

void BSP::initChip(void* args) {
    CmdLineArgs* cmdLineArgs = (CmdLineArgs*)args;
    ros::init(cmdLineArgs->m_argc, cmdLineArgs->m_argv, "hive_mind");

    m_rosNodeHandle = new ros::NodeHandle();

    xTaskCreate(rosWatcher, "ros_watch", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(exampleTopicPublish, "example_topic_publish", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, NULL);
}

ros::NodeHandle* BSP::getRosNodeHandle() { return m_rosNodeHandle; }
