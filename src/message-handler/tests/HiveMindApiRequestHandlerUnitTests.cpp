#include "mocks/BSPInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <message-handler/HiveMindApiRequestHandler.h>

class HiveMindApiRequestHandlerFixture : public testing::Test {
  protected:
    BSPInterfaceMock* m_bspMock;
    LoggerInterfaceMock* m_loggerInterfaceMock;
    HiveMindApiRequestHandler* m_hivemindApiReqHandler;

    const uint16_t m_boardId = 42;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_boardId);
        m_loggerInterfaceMock = new LoggerInterfaceMock();
        m_hivemindApiReqHandler = new HiveMindApiRequestHandler(*m_bspMock, *m_loggerInterfaceMock);
    }
    void TearDown() override {
        delete m_hivemindApiReqHandler;
        delete m_loggerInterfaceMock;
        delete m_bspMock;
    }
};

TEST_F(HiveMindApiRequestHandlerFixture, HiveMindApiRequestHandler_handleRequest_validIdRequest) {
    // Given
    IdRequestDTO idReq;
    HiveMindApiRequestDTO req(idReq);

    // Then
    HiveMindApiResponseDTO ret = m_hivemindApiReqHandler->handleRequest(req);

    // Expect
    auto response = std::get<IdResponseDTO>(ret.getResponse());
    EXPECT_EQ(response.getId(), m_boardId);
}

TEST_F(HiveMindApiRequestHandlerFixture, HiveMindApiRequestHandler_handleRequest_invalidRequest) {
    // Given
    IdRequestDTO idReq;
    HiveMindApiRequestDTO req(idReq);
    req.setRequest(std::monostate());

    // Then
    HiveMindApiResponseDTO ret = m_hivemindApiReqHandler->handleRequest(req);

    // Expect
    auto response = std::get<GenericResponseDTO>(ret.getResponse());
    EXPECT_EQ(response.getStatus(), GenericResponseStatusDTO::BadRequest);
}
