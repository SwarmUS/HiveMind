#ifndef __CIRCULARQUEUEINTERFACEMOCK_H_
#define __CIRCULARQUEUEINTERFACEMOCK_H_

#include <cpp-common/ICircularQueue.h>
#include <gmock/gmock.h>

template <typename T>
class CircularQueueInterfaceMock : public ICircularQueue<T> {
  public:
    ~CircularQueueInterfaceMock() = default;

    MOCK_METHOD(std::optional<const char*>, getString, (uint16_t stringId), (const override));

    MOCK_METHOD(bool, push, (const T& item), (override));

    MOCK_METHOD(const std::optional<std::reference_wrapper<const T>>, peek, (), (const override));

    MOCK_METHOD(void, pop, (), (override));

    MOCK_METHOD(void, clear, (), (override));

    MOCK_METHOD(bool, isFull, (), (const override));

    MOCK_METHOD(bool, isEmpty, (), (const override));

    MOCK_METHOD(uint32_t, getLength, (), (const override));

    MOCK_METHOD(uint32_t, getFreeSize, (), (const override));

    MOCK_METHOD(std::optional<std::reference_wrapper<T>>, getNextAllocation, (), (override));

    MOCK_METHOD(bool, advance, (), (override));
};

#endif // __CIRCULARQUEUEINTERFACEMOCK_H_
