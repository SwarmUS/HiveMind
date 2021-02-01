#include "BSP.h"
#include "ros/ros.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <TCPUartMock.h>
#include <bsp/BSPContainer.h>
#include <hive_mind/ExampleMessage.h>
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

    while (ros::ok()) {
        ros::spinOnce();
        vTaskDelay(loopRate);
    }

    vTaskEndScheduler();
}

void exampleTopicPublish(void* param) {
    const int loopRate = 4000;
    hive_mind::ExampleMessage msg = hive_mind::ExampleMessage();
    msg.number = 0;
    msg.text = "Hello World";

    std::shared_ptr<ros::NodeHandle>* nodeHandle = (std::shared_ptr<ros::NodeHandle>*)param;
    const uint32_t queueSize = 1000;
    ros::Publisher publisher =
        nodeHandle->get()->advertise<hive_mind::ExampleMessage>("exampleTopic", queueSize);

    while (true) {
        publisher.publish(msg);
        msg.number++;

        vTaskDelay(loopRate);
    }
}

void BSP::initChip(void* args) {
    CmdLineArgs* cmdLineArgs = (CmdLineArgs*)args;
    ros::init(cmdLineArgs->m_argc, cmdLineArgs->m_argv, "hive_mind");

    m_rosNodeHandle = std::make_shared<ros::NodeHandle>("~");

    TCPUartMock& tcpUart = static_cast<TCPUartMock&>(BSPContainer::getHostUart());
    int port = m_rosNodeHandle->param("uart_mock_port", 0);
    tcpUart.openSocket(port);

    xTaskCreate(rosWatcher, "ros_watch", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(exampleTopicPublish, "example_topic_publish", configMINIMAL_STACK_SIZE,
                &m_rosNodeHandle, tskIDLE_PRIORITY + 1, NULL);
}

std::shared_ptr<ros::NodeHandle> BSP::getRosNodeHandle() { return m_rosNodeHandle; }

uint16_t BSP::getUUId() const {
    // TODO: Change do the ID is obtained from persistent memory or ROS command line argument
    return 1;
}