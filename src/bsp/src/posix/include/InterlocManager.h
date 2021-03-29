#ifndef __INTERLOCMANAGER_H__
#define __INTERLOCMANAGER_H__

class InterlocManager : public IInterlocManager {
  public:
    ~InterlocManager() override = default;

    void startInterloc() override{};

    void registerPositionUpdateCallback(std::function<void(InterlocUpdate)> callback) override{
        (void)callback};

  private:
};

#endif //__INTERLOCMANAGER_H__
