#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include "ros/ros.h"
#include <TCPUartMock.h>
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <hive_mind/ExampleMessage.h>

/**
 * @brief Task that kills FreeRTOS when ROS node is stopped
 */
void rosWatcher(void* param) {
    (void)param;
    const int loopRate = 100;

    while (ros::ok()) {
        ros::spinOnce();
        Task::delay(loopRate);
    }
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

        Task::delay(loopRate);
    }
}

BSP::BSP() :
    m_rosWatchTask("ros_watch", tskIDLE_PRIORITY + 1, rosWatcher, NULL),
    m_exampleTopicPublishTask(
        "ros_watch", tskIDLE_PRIORITY + 1, exampleTopicPublish, &m_rosNodeHandle) {}

BSP::~BSP() = default;

void BSP::initChip(void* args) {
    CmdLineArgs* cmdLineArgs = (CmdLineArgs*)args;
    ros::init(cmdLineArgs->m_argc, cmdLineArgs->m_argv, "hive_mind");

    m_rosNodeHandle = std::make_shared<ros::NodeHandle>("~");

    TCPUartMock& tcpUart = static_cast<TCPUartMock&>(BSPContainer::getHostUart());
    int port = m_rosNodeHandle->param("uart_mock_port", 12345);
    tcpUart.openSocket(port);

    m_rosWatchTask.start();
    m_exampleTopicPublishTask.start();
}

std::shared_ptr<ros::NodeHandle> BSP::getRosNodeHandle() { return m_rosNodeHandle; }

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }
