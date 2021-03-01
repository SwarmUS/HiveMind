#include "bittybuzz/BittyBuzzClosureRegister.h"
#include <gtest/gtest.h>

class BittyBuzzClosureRegisterTestFixture : public testing::Test {
  protected:
    BittyBuzzClosureRegister* m_closureRegister;
    bbzvm_t m_bbzvm;

    void SetUp() override {

        vm = &m_bbzvm;
        m_closureRegister = new BittyBuzzClosureRegister();
    }

    void TearDown() override { delete m_closureRegister; }
};

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_empty) {
    // Given

    bbzheap_idx_t heapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", heapIdx);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_isPermanent) {
    // Given

    bbzheap_idx_t heapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", heapIdx);

    // Expect
    bbzobj_t* obj = bbzheap_obj_at(heapIdx);
    EXPECT_TRUE(ret);
    EXPECT_TRUE(obj->mdata | BBZHEAP_MASK_PERMANENT);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_notClosure) {
    // Given

    bbzheap_idx_t heapIdx = bbzint_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", heapIdx);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_full) {
    // Given

    bbzheap_idx_t heapIdx = bbzclosure_new(42);

    // Then

    for (uint16_t i = 0; i < BittyBuzzClosureRegister::m_maxSize; i++) {
        bool ret = m_closureRegister->registerClosure("Hello World", bbzclosure_new(i));
        EXPECT_TRUE(ret);
    }

    bool ret = m_closureRegister->registerClosure("Hello World", heapIdx);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_getClosureHeapIdx_empty) {
    // Given

    // Then

    std::optional<bbzheap_idx_t> ret = m_closureRegister->getClosureHeapIdx("Hello World");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_getClosureHeapIdx_exist) {
    // Given
    bbzheap_idx_t heap = bbzclosure_new(42);
    m_closureRegister->registerClosure("Hello World", heap);

    // Then

    std::optional<bbzheap_idx_t> ret = m_closureRegister->getClosureHeapIdx("Hello World");

    // Expect
    EXPECT_TRUE(ret.operator bool());
    EXPECT_EQ(heap, ret.value());
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getClosureHeapIdx_doesNotExist) {
    // Given
    m_closureRegister->registerClosure("Hello World", bbzclosure_new(42));

    // Then

    std::optional<bbzheap_idx_t> ret = m_closureRegister->getClosureHeapIdx("Good bye");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}
