#include "InterlocManager.h"
#include "BSP.h"
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <regex>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

InterlocManager::InterlocManager(ILogger& logger) :
    m_logger(logger), m_positionUpdateCallback(nullptr), m_positionUpdateContext(nullptr) {}

tf::Transform subtractPoses(geometry_msgs::Pose currentAgentPose,
                            geometry_msgs::Pose distantAgentPose) {
    tf::Transform tfCurrentAgent;
    tf::Transform tfDistantAgent;

    tf::poseMsgToTF(currentAgentPose, tfCurrentAgent);
    tf::poseMsgToTF(distantAgentPose, tfDistantAgent);

    return tfDistantAgent.inverseTimes(tfCurrentAgent);
}

double getDistance(const tf::Transform& transform) {
    return sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) +
                pow(transform.getOrigin().z(), 2));
}

double getOrientation(const tf::Transform& transform) {
    double roll;
    double pitch;
    double yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    return yaw;
}

void InterlocManager::gazeboUpdateCallback(const gazebo_msgs::ModelStates& msg) {
    std::optional<uint16_t> currentAgentIdx;
    std::map<uint16_t, uint16_t> idxLookup;
    const std::regex agentIdRegex("agent_([0-9]+)");
    std::smatch match;

    // Prepare map of id <-> index
    for (unsigned int i = 0; i < msg.name.size(); i++) {
        if (std::regex_match(msg.name[i], match, agentIdRegex)) {
            uint16_t agentId = stoi(match.str(1));

            if (agentId == BSPContainer::getBSP().getUUId()) {
                currentAgentIdx = i;
            } else {
                idxLookup[i] = agentId;
            }
        }
    }

    if (!currentAgentIdx) {
        return;
    }

    for (auto& x : idxLookup) {
        uint16_t index = x.first;
        uint16_t agentId = x.second;

        tf::Transform relTransform =
            subtractPoses(msg.pose[currentAgentIdx.value()], msg.pose[index]);
        double distance = getDistance(relTransform);
        double orientation = getOrientation(relTransform) * 180 / M_PI;

        if (m_positionUpdateCallback != nullptr) {
            InterlocUpdate update;
            update.m_robotId = agentId;
            update.m_distance = distance;
            update.m_relativeOrientation = orientation;
            update.m_isInLineOfSight = true; // TODO: maybe add a way to get LOS

            m_positionUpdateCallback(m_positionUpdateContext, update);
        }

        m_logger.log(LogLevel::Debug, "Updating position of agent %d. Dist: %f, Orientation %f",
                     agentId, distance, orientation);
    }

    Task::delay(5000);
}

void InterlocManager::startInterloc() {
    ros::NodeHandle node;
    m_sub = node.subscribe("/gazebo/model_states", 1, &InterlocManager::gazeboUpdateCallback, this);
}

void InterlocManager::setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                                void* context) {
    m_positionUpdateCallback = callback;
    m_positionUpdateContext = context;
}