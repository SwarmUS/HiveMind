#ifndef __BITTYBUZZSTRINGRESOLVERINTERFACEMOCK_H_
#define __BITTYBUZZSTRINGRESOLVERINTERFACEMOCK_H_

#include <bittybuzz/IBittyBuzzStringResolver.h>
#include <gmock/gmock.h>

class BittyBuzzStringResolverInterfaceMock : public IBittyBuzzStringResolver {
  public:
    ~BittyBuzzStringResolverInterfaceMock() = default;

    MOCK_METHOD(std::optional<const char*>, getString, (uint16_t stringId), (const override));
};

#endif // __BITTYBUZZSTRINGRESOLVERINTERFACEMOCK_H_
