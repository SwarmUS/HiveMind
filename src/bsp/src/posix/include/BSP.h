#ifndef __BSP_H_
#define __BSP_H_

#include "bsp/IBSP.h"
#include <BaseTask.h>
#include <random>
#include <ros/ros.h>

class BSP : public IBSP {
  public:
    BSP();
    ~BSP() override;

    void initChip(void* args) override;

    uint16_t getUUId() const override;

    uint32_t generateRandomNumber() override;

    std::shared_ptr<ros::NodeHandle> getRosNodeHandle();

  private:
    std::shared_ptr<ros::NodeHandle> m_rosNodeHandle;
    BaseTask<2 * configMINIMAL_STACK_SIZE> m_rosWatchTask;

    std::mt19937 m_rng;
    std::uniform_int_distribution<uint32_t> m_distribution;
};

#endif // __BSP_H_
