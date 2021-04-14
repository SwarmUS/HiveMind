#ifndef __BITTYBUZZSTDLIB_H_
#define __BITTYBUZZSTDLIB_H_

#include "IBittyBuzzLib.h"
#include <cstdint>

template <typename Container>
class BittyBuzzLib : public IBittyBuzzLib {
  public:
    BittyBuzzLib(uint16_t libTableId, const Container& container);

    ~BittyBuzzLib() override = default;

    void registerLibs() override;

  private:
    const uint16_t m_libTableId;
    const Container m_container;
};

#include "BittyBuzzLib.tpp"

#endif // __BITTYBUZZSTDLIB_H_
