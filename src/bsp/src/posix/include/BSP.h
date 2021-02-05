#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"
#include <freertos-utils/BaseTask.h>
#include <ros/ros.h>

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip(void* args) override;
    uint16_t getUUId() const override;
    std::shared_ptr<ros::NodeHandle> getRosNodeHandle();

  private:
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
    BaseTask<2 * configMINIMAL_STACK_SIZE> m_rosWatchTask;
    BaseTask<configMINIMAL_STACK_SIZE> m_exampleTopicPublishTask;
};

#endif // __BSP_H_
