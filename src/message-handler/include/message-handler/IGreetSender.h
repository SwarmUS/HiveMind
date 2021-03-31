#ifndef __IGREETSENDER_H_
#define __IGREETSENDER_H_

class IGreetSender {
  public:
    virtual ~IGreetSender() = default;

    /**
     * @brief send a greet to the output queue
     * @return true if the push was successfull, false if not
     * */
    virtual bool sendGreet() = 0;
};

#endif // __IGREETSENDER_H_
