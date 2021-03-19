#ifndef __IINTERLOCMANAGER_H__
#define __IINTERLOCMANAGER_H__

class IInterlocManager {
  public:
    virtual ~IInterlocManager() = default;

    /**
     * @brief Dummy function to demonstrate working DW1000s
     */
    virtual void startInterloc() = 0;
};

#endif //__IINTERLOCMANAGER_H__
