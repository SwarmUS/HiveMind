#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include <NotificationQueue.h>
#include <bsp/IInterlocManager.h>
#include <gazebo_msgs/ModelStates.h>
#include <logger/ILogger.h>
#include <ros/subscriber.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

class InterlocManager : public IInterlocManager {
  public:
    InterlocManager(ILogger& logger, NotificationQueue<InterlocUpdate>& interlocUpdateQueue);
    ~InterlocManager() override = default;

    void startInterloc() override;

    // Calib API is not needed in simulation. Can just ignore any calls to these functions
    void setInterlocManagerState(InterlocStateDTO state) override;

    void configureTWRCalibration(uint16_t distanceCalibCm) override;

    void configureAngleCalibration(uint32_t numberOfFrames) override;

    void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) override;

    void setInterlocManagerRawAngleDataCallback(interlocRawAngleDataCallbackFunction_t callback,
                                                void* context) override;

  private:
    ILogger& m_logger;
    ros::Subscriber m_sub;
    uint m_interlocRefreshDelayMs;
    std::map<uint16_t, geometry_msgs::TransformStamped> m_baseLinkToHiveBoardTransforms;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    NotificationQueue<InterlocUpdate>& m_interlocUpdateQueue;

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
