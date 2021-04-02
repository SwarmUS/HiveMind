#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>
#include <message-handler/HiveMindHostApiRequestHandler.h>

class HiveMindHostApiRequestHandlerFixture : public testing::Test {
  protected:
    BSPInterfaceMock* m_bspMock;
    LoggerInterfaceMock* m_loggerInterfaceMock;
    HiveMindHostApiRequestHandler* m_hivemindApiReqHandler;
    CircularQueueInterfaceMock<MessateDTO> m_hostQueueMock;

    RequestDTO* m_request;
    MessageDTO* m_message;

    const uint16_t m_boardId = 42;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_boardId);
        m_loggerInterfaceMock = new LoggerInterfaceMock();
        m_hivemindApiReqHandler =
            new HiveMindHostApiRequestHandler(*m_bspMock, m_hostQueueMock, *m_loggerInterfaceMock);

        m_request = new RequestDTO(Request{});
        m_message = new MessageDTO(m_boardId, m_boardId, *m_request);
    }
    void TearDown() override {
        delete m_hivemindApiReqHandler;
        delete m_loggerInterfaceMock;
        delete m_bspMock;
        delete m_request;
        delete m_message;
    }
};

TEST_F(HiveMindHostApiRequestHandlerFixture, HiveMindHostApiRequestHandler_handleRequest_bytes) {
    // Given
    HiveMindHostApiRequestDTO req(BytesDTO(0, 0, true, NULL, 0));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(req);

    // Expect
    auto response = std::get<GenericResponseDTO>(ret.getResponse());
    EXPECT_EQ(response.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(HiveMindHostApiRequestHandlerFixture,
       HiveMindHostApiRequestHandler_handleRequest_invalidRequest) {
    // Given
    HiveMindHostApiRequestDTO req(BytesDTO(0, 0, true, NULL, 0));
    req.setRequest(std::monostate());

    // Then
    HiveMindHostApiResponseDTO ret = m_hivemindApiReqHandler->handleRequest(req);

    // Expect
    auto response = std::get<GenericResponseDTO>(ret.getResponse());
    EXPECT_EQ(response.getStatus(), GenericResponseStatusDTO::BadRequest);
}
