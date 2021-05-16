#ifndef __DECAWAVEARRAY_H__
#define __DECAWAVEARRAY_H__

#include "Decawave.h"
#include <array>
#include <iterator>

enum class DecawavePort { A = 0, B, NUM_PORTS };

/**
 * @brief Wrapper class that allows accessing DW1000s through an enum
 */
class DecawaveArray {
  public:
    DecawaveArray() = default;
    virtual ~DecawaveArray() = default;

    inline Decawave* begin() { return m_decawaves.begin(); }
    inline const Decawave* begin() const { return m_decawaves.begin(); }

    inline Decawave* end() { return m_decawaves.end(); }
    inline const Decawave* end() const { return m_decawaves.end(); }

    inline Decawave& operator[](DecawavePort port) {
        return m_decawaves[static_cast<unsigned int>(port)];
    }

  private:
    // TODO: Generate based on H7 or F4. (Maybe give names like master, slave_0..n, etc)
    std::array<Decawave, static_cast<unsigned int>(DecawavePort::NUM_PORTS)> m_decawaves = {
        Decawave(DW_A0, UWBChannel::CHANNEL_5, UWBSpeed::SPEED_6M8),
        Decawave(DW_B0, UWBChannel::CHANNEL_5, UWBSpeed::SPEED_6M8)};
};

#endif //__DECAWAVEARRAY_H__
