#ifndef __BITTYBUZZSTDLIB_H_
#define __BITTYBUZZSTDLIB_H_

#include "IBittyBuzzLib.h"
#include <cstdint>
#include <optional>

template <typename Container>
class BittyBuzzLib : public IBittyBuzzLib {
  public:
    /**@brief BittyBuzzLib constructor, will register on a table/namespace
     * @param libTableId the string id of the table
     * @param container a container of BittyBuzzLibMemberRegister*/
    BittyBuzzLib(uint16_t libTableId, const Container& container);

    /**@brief BittyBuzzLib constructor, will register on the global scope
     * @param container a container of BittyBuzzLibMemberRegister*/
    BittyBuzzLib(const Container& container);

    ~BittyBuzzLib() override = default;

    bool registerLib() override;

  private:
    bool registerLibGlobal();
    bool registerLibTable();
    const std::optional<uint16_t> m_libTableId;
    const Container m_container;
};

#include "BittyBuzzLib.tpp"

#endif // __BITTYBUZZSTDLIB_H_
