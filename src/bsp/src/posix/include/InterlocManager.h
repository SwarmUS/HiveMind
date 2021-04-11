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
    void startCalibSingleInitiator() override{};
    void startCalibSingleResponder(uint16_t initiatorId,
                                   calibrationEndedCallbackFunction_t callback,
                                   void* context) override {
        (void)initiatorId;
        (void)callback;
        (void)context;
    };
    void stopCalibration() override{};
    void setCalibDistance(uint16_t distanceCalibCm) override { (void)distanceCalibCm; };

  private:
    ILogger& m_logger;
    ros::Subscriber m_sub;
    int m_interlocRefreshDelayMs;
    std::map<uint16_t, geometry_msgs::TransformStamped> m_baseLinkToHiveBoardTransforms;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateContext;

    static tf2::Transform computeRelativeTransform(
        const geometry_msgs::Pose& currentAgentPose,
        const geometry_msgs::TransformStamped& currentAgentHBTransform,
        const geometry_msgs::Pose& distantAgentPose,
        const geometry_msgs::TransformStamped& distantAgentHBTransform);

    static double getDistance(const tf2::Transform& transform);
    static double getOrientation(const tf2::Transform& transform);

    void gazeboUpdateCallback(const gazebo_msgs::ModelStates& msg);
    std::optional<geometry_msgs::TransformStamped> getHiveBoardTransform(
        const std::string& agentName);
};

#endif //__INTERLOCMANAGER_H__
