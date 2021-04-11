#include "bittybuzz/BittyBuzzClosureRegister.h"
#include <gtest/gtest.h>

class BittyBuzzClosureRegisterTestFixture : public testing::Test {
  protected:
    BittyBuzzClosureRegister* m_closureRegister;
    bbzvm_t m_bbzvm;
    BittyBuzzFunctionDescription* m_description;
    const char* m_functionName = "functionName";

    void SetUp() override {
        vm = &m_bbzvm;
        m_description = new BittyBuzzFunctionDescription(m_functionName);
        m_closureRegister = new BittyBuzzClosureRegister();
    }

    void TearDown() override {
        delete m_description;
        delete m_closureRegister;
    }
};

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_empty) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  *m_description);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_isPermanent) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then
    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  *m_description);

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
                                                  *m_description);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture, BittyBuzzClosureRegister_registerClosure_full) {
    // Given

    bbzheap_idx_t closureHeapIdx = bbzclosure_new(42);

    // Then

    // We need to allocate the string and keep them alive for the whole function call since in the
    // real case we are using global const strings.
    std::vector<std::string> functionNames;
    for (uint16_t i = 0; i < BittyBuzzClosureRegister::m_maxSize; i++) {
        functionNames.push_back("Hello World" + std::to_string(i));
        bool ret = m_closureRegister->registerClosure(functionNames[i].c_str(), bbzclosure_new(i),
                                                      m_bbzvm.nil, *m_description);
        EXPECT_TRUE(ret);
    }

    bool ret = m_closureRegister->registerClosure("Hello World", closureHeapIdx, m_bbzvm.nil,
                                                  *m_description);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_name_empty) {
    // Given

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Hello World");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_name_exist) {
    // Given
    bbzheap_idx_t heap = bbzclosure_new(42);
    m_closureRegister->registerClosure("Hello World", heap, m_bbzvm.nil, *m_description);

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Hello World");

    // Expect
    EXPECT_TRUE(ret.operator bool());
    EXPECT_EQ(heap, ret.value().get().m_closureHeapIdx);
    EXPECT_EQ(m_bbzvm.nil, ret.value().get().m_selfHeapIdx);
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_name_doesNotExist) {
    // Given
    m_closureRegister->registerClosure("Hello World", bbzclosure_new(42), m_bbzvm.nil,
                                       *m_description);

    // Then

    auto ret = m_closureRegister->getRegisteredClosure("Good bye");

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_id_empty) {
    // Given

    // Then
    uint16_t idx = 0;
    auto ret = m_closureRegister->getRegisteredClosure(idx);

    // Expect
    EXPECT_FALSE(ret.operator bool());
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_id_exist) {
    // Given
    bbzheap_idx_t heap = bbzclosure_new(42);
    m_closureRegister->registerClosure("Hello World", heap, m_bbzvm.nil, *m_description);

    // Then
    uint16_t idx = 0;
    auto ret = m_closureRegister->getRegisteredClosure(idx);

    // Expect
    EXPECT_TRUE(ret.operator bool());
    EXPECT_EQ(heap, ret.value().get().m_closureHeapIdx);
    EXPECT_EQ(m_bbzvm.nil, ret.value().get().m_selfHeapIdx);
}

TEST_F(BittyBuzzClosureRegisterTestFixture,
       BittyBuzzClosureRegister_getRegisteredClosure_id_doesNotExist) {
    // Given
    m_closureRegister->registerClosure("Hello World", bbzclosure_new(42), m_bbzvm.nil,
                                       *m_description);

    // Then
    uint16_t idx = 1;
    auto ret = m_closureRegister->getRegisteredClosure(idx);

    // Expect
    EXPECT_FALSE(ret.operator bool());
}
