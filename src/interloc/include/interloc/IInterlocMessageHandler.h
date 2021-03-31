#ifndef __IINTERLOCMESSAGEHANDLER_H__
#define __IINTERLOCMESSAGEHANDLER_H__

class IInterlocMessageHandler {
  public:
    virtual bool processMessage() = 0;
    virtual bool notifyCalibrationEnded(uint16_t initiatorId) = 0;
};

#endif //__IINTERLOCMESSAGEHANDLER_H__
