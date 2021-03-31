#ifndef __IMESSAGESENDER_H_
#define __IMESSAGESENDER_H_

/**
 *@brief A class to pipe messages to a stream and serialize them */
class IMessageSender {
  public:
    virtual ~IMessageSender() = default;

    /**
     *@brief process an item in the queue, serializes it and sends it to the deserializer
     *@return true if the operation was successful, false if not*/
    virtual bool processAndSerialize() = 0;
};
#endif // __IMESSAGESENDER_H_
