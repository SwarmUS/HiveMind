#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

#include <bsp/IInterlocManager.h>
#include <gazebo_msgs/ModelStates.h>
#include <logger/ILogger.h>
#include <ros/subscriber.h>

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

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateContext;

    void gazeboUpdateCallback(const gazebo_msgs::ModelStates& msg);
};

#endif //__INTERLOCMANAGER_H__
