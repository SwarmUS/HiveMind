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
    void startCalibSingleResponder() override{};
    void setCalibDistance(uint16_t distanceCalibCm) override { (void)distanceCalibCm; };
    void setCalibFinishedCallback(void (*fct)(void* context), void* context) override {
        (void)fct;
        (void)context;
    };

  private:
};

#endif //__INTERLOCMANAGER_H__
