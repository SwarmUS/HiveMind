#ifndef HIVE_MIND_SETDISTANCESTATE_H
#define HIVE_MIND_SETDISTANCESTATE_H

#include "AbstractInterlocState.h"
#include <cstdint>

class UpdateInterloc : public AbstractInterlocState {
  public:
    UpdateInterloc(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    float calculateAngle(InterlocStateHandler& context);

    std::array<float, 30> m_tdoaDist;
    std::array<float, 30> m_tdoaAngle;

    float pdoa_M = 8.01;
    float pdoa_B[5] = {-1173.73, -1370.77, -1539.66, -1708.56, -1891.52};
    float pdoa_Origin[5];
};

#endif // HIVE_MIND_SETDISTANCESTATE_H
