#ifndef USERINTERFACEMOCK_H_
#define USERINTERFACEMOCK_H_

#include <bsp/IUserInterface.h>
#include <gmock/gmock.h>

class UserInterfaceMock : public IUserInterface {

    MOCK_METHOD(Mutex&, getPrintMutex, (), (override));
    MOCK_METHOD(void, flush, (), (override));
}

#endif // USERINTERFACEMOCK_H_
