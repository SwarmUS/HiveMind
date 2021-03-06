#include "bittybuzz/BittyBuzzClosureRegister.h"
#include <gtest/gtest.h>

class BittyBuzzClosureRegisterTestFixture : public testing::Test {
  protected:
    BittyBuzzClosureRegister* m_closureRegister;
    bbzvm_t m_bbzvm;
    BittyBuzzFunctionDescription m_description;

    void SetUp() override {

        vm = &m_bbzvm;
        m_closureRegister = new BittyBuzzClosureRegister();
    }

    void TearDown() override { delete m_closureRegister; }
};

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_empty) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  m_description);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_isPermanent) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  m_description);

    // Expect
    bbzobj_t* obj = bbzheap_obj_at(closureHeapIdx);
    EXPECT_TRUE(ret);
    EXPECT_TRUE(obj->mdata | BBZHEAP_MASK_PERMANENT);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_notClosure) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzint_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  m_description);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_full) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then

    for (uint16_t i = 0; i < BittyBuzzClosureRegister::m_maxSize; i++) {
        bool ret = m_closureRegister->registerClosure("Hello World", bbzclosure_new(i), m_bbzvm.nil,
                                                      m_description);
        EXPECT_TRUE(ret);
    }

    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  m_description);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_getRegisteredClosure_empty) {
    // Given

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Hello World");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_getRegisteredClosure_exist) {
    // Given
    bbzheap_idx_t heap = bbzclosure_new(42);
    m_closureRegister->registerClosure("Hello World", heap, m_bbzvm.nil, m_description);

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Hello World");

    // Expect
    EXPECT_TRUE(ret.operator bool());
    EXPECT_EQ(heap, ret.value().get().m_closureHeapIdx);
    EXPECT_EQ(m_bbzvm.nil, ret.value().get().m_selfHeapIdx);
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_doesNotExist) {
    // Given
    m_closureRegister->registerClosure("Hello World", bbzclosure_new(42), m_bbzvm.nil,
                                       m_description);

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Good bye");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_invalidSelf) {
    // Given
    m_closureRegister->registerClosure("Hello World", bbzclosure_new(42), bbzint_new(42),
                                       m_description);

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Hello World");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}
