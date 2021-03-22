#ifndef __IGREETER_H_
#define __IGREETER_H_

class IGreeter {
  public:
    /**
     *@brief wait for a greet and send the greet response.
     *@return true if serialization/deserialization successed, false otherwise*/
    virtual bool greet() = 0;
};

#endif // __IGREETER_H_
