#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include <bsp/IInterlocManager.h>
#include <gazebo_msgs/ModelStates.h>
#include <logger/ILogger.h>
#include <ros/subscriber.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

class InterlocManager : public IInterlocManager {
  public:
    InterlocManager(ILogger& logger);
    ~InterlocManager() override = default;

    void startInterloc() override;

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override;

    // Calib API is not needed in simulation. Can just ignore any calls to these functions
    void startCalibSingleInitiator() override;
    void startCalibSingleResponder(uint16_t initiatorId,
                                   calibrationEndedCallbackFunction_t callback,
                                   void* context) override;
    void stopCalibration() override;
    void configureTWRCalibration(uint16_t distanceCalibCm) override;

  private:
    ILogger& m_logger;
    ros::Subscriber m_sub;
    int m_interlocRefreshDelayMs;
    std::map<uint16_t, geometry_msgs::TransformStamped> m_baseLinkToHiveBoardTransforms;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateContext;

    static tf2::Stamped<tf2::Transform> getHiveboardTf(
        const geometry_msgs::Pose& poseWorldFrame,
        const geometry_msgs::TransformStamped& hiveboardToRobotTf);
    static double getDistance(const tf2::Transform& transform);
    static double getRelativeOrientation(const tf2::Transform& transform);
    static double getAngleOfArrival(const tf2::Transform& agentToAgentTransform);

    void gazeboUpdateCallback(const gazebo_msgs::ModelStates& msg);
    std::optional<geometry_msgs::TransformStamped> getHiveBoardTransform(
        const std::string& agentName);
};

#endif //__INTERLOCMANAGER_H__
