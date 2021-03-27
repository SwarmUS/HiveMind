#ifndef __IGREETHANDLER_H_
#define __IGREETHANDLER_H_

class IGreetHandler {
  public:
    virtual ~IGreetHandler() = default;

    /**
     *@brief wait for a greet and then send the greet response. Serializes directly to the stream
     *and not the the queue. Use to initialize a connection.
     *@return true if serialization/deserialization successed, false otherwise*/
    virtual bool greet() = 0;
};

#endif // __IGREETHANDLER_H_
