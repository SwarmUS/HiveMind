
#ifndef __BSPINTERFACEMOCK_H_
#define __BSPINTERFACEMOCK_H_

#include <bsp/IBSP.h>
#include <gmock/gmock.h>

class BSPInterfaceMock final : public IBSP {
  public:
    const uint16_t& m_boardId;

    BSPInterfaceMock(const uint16_t& boardId) : m_boardId(boardId) {}

    void initChip(void* args) override {(void) args;}

    virtual uint16_t getUUId() const override { return m_boardId; }
};

#endif // __BSPINTERFACEMOCK_H_