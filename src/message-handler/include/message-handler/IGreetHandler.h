#ifndef __IGREETHANDLER_H_
#define __IGREETERHANDLER_H_

class IGreetHandler {
  public:
    virtual ~IGreetHandler() = default;

    /**
     *@brief wait for a greet and then send the greet response.
     *@return true if serialization/deserialization successed, false otherwise*/
    virtual bool greet() = 0;

    /**
     * @brief send a greet
     * @return trie if the serialization successed, false otherwise
     * */
    virtual bool sendGreet() = 0;
};

#endif // __IGREETERHANDLER_H_
