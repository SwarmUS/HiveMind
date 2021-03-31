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

  private:
};

#endif //__INTERLOCMANAGER_H__
