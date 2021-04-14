#ifndef __BITTYBUZZSTDLIB_H_
#define __BITTYBUZZSTDLIB_H_

#include "IBittyBuzzLib.h"
#include <cstdint>

template <typename Container>
class BittyBuzzLib : public IBittyBuzzLib {
  public:
    /**@brief BittyBuzzLib constructor
     * @param libTableId the string id of the table, use 0 to register on the global scope
     * @param container a container of BittyBuzzLibMemberRegister*/
    BittyBuzzLib(uint16_t libTableId, const Container& container);

    ~BittyBuzzLib() override = default;

    bool registerLib() override;

  private:
    bool registerLibGlobal();
    bool registerLibTable();
    const uint16_t m_libTableId;
    const Container m_container;
};

#include "BittyBuzzLib.tpp"

#endif // __BITTYBUZZSTDLIB_H_
