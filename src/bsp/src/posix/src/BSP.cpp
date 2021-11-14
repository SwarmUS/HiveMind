#include "BSP.h"
#include "TCPServer.h"
#include "bsp/SettingsContainer.h"

#include <Task.h>
#include <bsp/BSPContainer.h>
#include <random>
#include <ros/ros.h>

/**
 * @brief Task that kills OS when ROS node is stopped
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

    m_rosWatchTask.start();
}

std::shared_ptr<ros::NodeHandle> BSP::getRosNodeHandle() { return m_rosNodeHandle; }

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return m_distribution(m_rng); }
