#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

class InterlocManager : public IInterlocManager {
  public:
    ~InterlocManager() override = default;

    void startInterloc() override{};

    void setPositionUpdateCallback(PositionUpdateCallbackFunction callback) override {
        // TODO: implement
        (void)callback;
    };

  private:
};

#endif //__INTERLOCMANAGER_H__
