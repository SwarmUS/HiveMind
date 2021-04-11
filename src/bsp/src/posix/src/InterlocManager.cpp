#include "InterlocManager.h"
#include "BSP.h"
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <gazebo_msgs/ModelStates.h>
#include <regex>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define ROBOT_CENTER_SUFFIX "/base_link"
#define HIVEBOARD_SUFFIX "/hiveboard"

InterlocManager::InterlocManager(ILogger& logger) :
    m_logger(logger),
    m_tfListener(m_tfBuffer),
    m_positionUpdateCallback(nullptr),
    m_positionUpdateContext(nullptr) {}

tf2::Transform InterlocManager::computeRelativeTransform(
    const geometry_msgs::Pose& currentAgentPose,
    const geometry_msgs::TransformStamped& currentAgentHBTransform,
    const geometry_msgs::Pose& distantAgentPose,
    const geometry_msgs::TransformStamped& distantAgentHBTransform) {

    // Convert the poses into the HB frame
    geometry_msgs::Pose currentAgentPoseHBFrame;
    geometry_msgs::Pose distantAgentPoseHBFrame;

    tf2::doTransform(currentAgentPose, currentAgentPoseHBFrame, currentAgentHBTransform);
    tf2::doTransform(distantAgentPose, distantAgentPoseHBFrame, distantAgentHBTransform);

    tf2::Transform currentAgentTf;
    tf2::Transform distantAgentTf;
    tf2::fromMsg(currentAgentPoseHBFrame, currentAgentTf);
    tf2::fromMsg(distantAgentPoseHBFrame, distantAgentTf);

    // Equivalent of currentAgentPose - distantAgentPose
    return distantAgentTf.inverseTimes(currentAgentTf);
}

double InterlocManager::getDistance(const tf2::Transform& transform) {
    return sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) +
                pow(transform.getOrigin().z(), 2));
}

double InterlocManager::getOrientation(const tf2::Transform& transform) {
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    return yaw;
}

std::optional<geometry_msgs::TransformStamped> InterlocManager::getHiveBoardTransform(
    const std::string& agentName) {
    geometry_msgs::TransformStamped transform;

    try {
        transform = m_tfBuffer.lookupTransform(agentName + HIVEBOARD_SUFFIX,
                                               agentName + ROBOT_CENTER_SUFFIX, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        m_logger.log(LogLevel::Warn, "Could not find transform to hiveboard for %s",
                     agentName.c_str());
        m_logger.log(LogLevel::Warn, "%s", ex.what());
    }

    return transform;
}

void InterlocManager::gazeboUpdateCallback(const gazebo_msgs::ModelStates& msg) {
    const uint16_t currentAgentId = BSPContainer::getBSP().getUUId();
    std::map<uint16_t, uint16_t> idxLookup;
    const std::regex agentIdRegex("agent_([0-9]+)");
    std::smatch match;

    // Prepare map of id <-> index
    for (unsigned int i = 0; i < msg.name.size(); i++) {
        if (std::regex_match(msg.name[i], match, agentIdRegex)) {
            uint16_t agentId = stoi(match.str(1));

            // Only try to get the transform once per robot as it will not change at runtime
            if (m_baseLinkToHiveBoardTransforms.count(agentId) == 0) {
                auto transform = getHiveBoardTransform(msg.name[i]);
                if (!transform) {
                    // Transform between robot and hiveboard could not be found. Do not add this ID
                    // to the map to prevent calculating interloc with it.
                    continue;
                }
                m_baseLinkToHiveBoardTransforms[agentId] = transform.value();
            }

            idxLookup[i] = agentId;
        }
    }

    if (idxLookup.count(currentAgentId) == 0) {
        return;
    }

    for (auto& x : idxLookup) {
        uint16_t index = x.first;
        uint16_t agentId = x.second;

        if (agentId == currentAgentId) {
            // Do not calculate interloc with ourself
            continue;
        }

        tf2::Transform relTransform = computeRelativeTransform(
            msg.pose[idxLookup[currentAgentId]], m_baseLinkToHiveBoardTransforms[currentAgentId],
            msg.pose[index], m_baseLinkToHiveBoardTransforms[agentId]);

        double distance = getDistance(relTransform);
        double orientation = getOrientation(relTransform) * 180 / M_PI;

        if (m_positionUpdateCallback != nullptr) {
            InterlocUpdate update;
            update.m_robotId = agentId;
            update.m_distance = distance;
            update.m_relativeOrientation = orientation;
            update.m_isInLineOfSight = true; // TODO: maybe add a way to get LOS

            m_positionUpdateCallback(m_positionUpdateContext, update);

            m_logger.log(LogLevel::Debug, "Updating position of agent %d. Dist: %f, Orientation %f",
                         agentId, distance, orientation);
        }
    }

    Task::delay(m_interlocRefreshDelayMs);
}

void InterlocManager::startInterloc() {
    ros::NodeHandle node("~");
    m_sub = node.subscribe("/gazebo/model_states", 1, &InterlocManager::gazeboUpdateCallback, this);
    double interlocRefreshRate = node.param("interloc_refresh_rate", 10.0);
    m_interlocRefreshDelayMs = (int)(1.0 / interlocRefreshRate * 1000);

    m_logger.log(LogLevel::Info, "Starting interloc with a refresh rate of %.2f Hz",
                 interlocRefreshRate);
}

void InterlocManager::setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                                void* context) {
    m_positionUpdateCallback = callback;
    m_positionUpdateContext = context;
}