#include "BSP.h"
#include "SpiEspMock.h"
#include "bsp/SettingsContainer.h"
#include "ros/ros.h"
#include <TCPUartMock.h>
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <random>

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

BSP::BSP() :
    m_rosWatchTask("ros_watch", tskIDLE_PRIORITY + 1, rosWatcher, NULL),
    m_rng(std::random_device()()),
    m_distribution(0, UINT32_MAX) {}

BSP::~BSP() = default;

void BSP::initChip(void* args) {
    CmdLineArgs* cmdLineArgs = (CmdLineArgs*)args;
    ros::init(cmdLineArgs->m_argc, cmdLineArgs->m_argv, "hive_mind");

    m_rosNodeHandle = std::make_shared<ros::NodeHandle>("~");

    TCPUartMock& tcpUart = static_cast<TCPUartMock&>(BSPContainer::getHostUart());
    int port = m_rosNodeHandle->param("uart_mock_port", 0);
    tcpUart.openSocket(port);

    SpiEspMock& espMock = static_cast<SpiEspMock&>(BSPContainer::getSpiEsp());
    port = m_rosNodeHandle->param("spi_mock_port", 9001);
    espMock.openSocket(port);

    m_rosWatchTask.start();
}

std::shared_ptr<ros::NodeHandle> BSP::getRosNodeHandle() { return m_rosNodeHandle; }

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return m_distribution(m_rng); }
