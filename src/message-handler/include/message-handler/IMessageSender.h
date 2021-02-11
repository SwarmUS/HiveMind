#ifndef __IMESSAGESENDER_H_
#define __IMESSAGESENDER_H_

class IMessageSender {
  public:
    virtual ~IMessageSender() = default;

    virtual bool processAndSerialize() = 0;
};
#endif // __IMESSAGESENDER_H_
