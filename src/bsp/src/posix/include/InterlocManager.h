#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

class InterlocManager : public IInterlocManager {
  public:
    ~InterlocManager() override = default;

    void startInterloc() override{};

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override {
        // TODO: implement
        (void)callback;
        (void)context;
    };

    void startCalibSingleInitiator() override{};
    void startCalibSingleResponder(uint16_t initiatorId,
                                   calibrationEndedCallbackFunction_t callback,
                                   void* context) override {
        (void)initiatorId;
        (void)callback;
        (void)context;

        // TODO: Implement
    };
    void stopCalibration() override{};
    void setCalibDistance(uint16_t distanceCalibCm) override { (void)distanceCalibCm; };

  private:
};

#endif //__INTERLOCMANAGER_H__
