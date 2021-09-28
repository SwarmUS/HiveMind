#ifndef __DECAWAVEARRAY_H__
#define __DECAWAVEARRAY_H__

#include "Decawave.h"
#include <array>
#include <iterator>

/**
 * @brief Wrapper class that allows accessing DW1000s through an enum
 */
class DecawaveArray {
  public:
    constexpr static uint8_t angleAntennaArraySize = (DWT_NUM_DW_DEV < 3) ? 0 : 3; // MAXIMUM 3

    DecawaveArray() = default;
    virtual ~DecawaveArray() = default;

    inline Decawave* begin() { return m_decawaves.begin(); }
    inline const Decawave* begin() const { return m_decawaves.begin(); }

    inline Decawave* end() { return m_decawaves.end(); }
    inline const Decawave* end() const { return m_decawaves.end(); }

    void initializeAll();
    bool canDoTWR() const;
    bool canCalculateAngles() const;

    std::optional<std::reference_wrapper<Decawave>> getMasterAntenna();
    std::optional<std::reference_wrapper<Decawave>> getLeftAntenna();
    std::optional<std::reference_wrapper<Decawave>> getRightAntenna();
    std::array<std::optional<std::reference_wrapper<Decawave>>, angleAntennaArraySize>&
    getAngleAntennaArray();

  private:
    uint8_t m_workingDecasLength = 0;

    void initializeAngleAntennaArray();

    // Because the Decawave class is not copyable or moveable (atomic variable inside of a task), we
    // have to assign the arrays here
#ifdef STM32F4
    std::array<Decawave, DWT_NUM_DW_DEV> m_decawaves = {Decawave(DW_A0), Decawave(DW_B0)};
    std::array<std::reference_wrapper<Decawave>, angleAntennaArraySize> m_angleAntennaArray;
#elif STM32H7
    std::array<Decawave, DWT_NUM_DW_DEV> m_decawaves = {Decawave(DW_A0), Decawave(DW_A1),
                                                        Decawave(DW_B0), Decawave(DW_B1),
                                                        Decawave(DW_C0), Decawave(DW_C1)};

    std::array<std::optional<std::reference_wrapper<Decawave>>, angleAntennaArraySize>
        m_angleAntennaArray;
#endif
};

#endif //__DECAWAVEARRAY_H__
