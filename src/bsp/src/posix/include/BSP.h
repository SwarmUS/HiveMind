#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"
#include "ros/ros.h"

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip(void* args) override;
    uint16_t getUUId() const override;
    std::shared_ptr<ros::NodeHandle> getRosNodeHandle();

  private:
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
};

#endif // __BSP_H_
