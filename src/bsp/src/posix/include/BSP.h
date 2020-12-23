#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"
#include "ros/ros.h"

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip(int argc, char** argv) override;
    ros::NodeHandle* getRosNodeHandle();

  private:
    ros::NodeHandle* m_rosNodeHandle;
};

#endif // __BSP_H_
