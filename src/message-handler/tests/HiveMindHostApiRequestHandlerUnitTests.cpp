#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/InterlocInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "utils/DTOMatchers.h"
#include <gtest/gtest.h>
#include <message-handler/HiveMindHostApiRequestHandler.h>
#include <vector>

class HiveMindHostApiRequestHandlerFixture : public testing::Test {
  protected:
    BSPInterfaceMock* m_bspMock;
    LoggerInterfaceMock* m_loggerInterfaceMock;
    HiveMindHostApiRequestHandler* m_hivemindApiReqHandler;
    CircularQueueInterfaceMock<MessageDTO> m_hostQueueMock;
    InterlocInterfaceMock m_interlocMock;

    RequestDTO* m_request;
    MessageDTO* m_message;

    const uint16_t m_boardId = 42;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_boardId);
        m_loggerInterfaceMock = new LoggerInterfaceMock();
        m_hivemindApiReqHandler = new HiveMindHostApiRequestHandler(
            *m_bspMock, m_hostQueueMock, m_interlocMock, *m_loggerInterfaceMock);

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
    m_request->setRequest(req);
    m_message->setMessage(*m_request);
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(*m_message);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(HiveMindHostApiRequestHandlerFixture,
       HiveMindHostApiRequestHandler_handleRequest_neighbor_exist) {
    // Given
    RelativePosition pos;
    pos.m_distance = 1;
    pos.m_relativeOrientation = 1;
    pos.m_robotId = 42;
    pos.m_isInLineOfSight = true;

    HiveMindHostApiRequestDTO req(GetNeighborRequestDTO(42));
    m_request->setRequest(req);
    m_message->setMessage(*m_request);

    EXPECT_CALL(m_interlocMock, getRobotPosition(42)).WillOnce(testing::Return(pos));
    EXPECT_CALL(m_hostQueueMock, push(MessageGetNeighborResponseDTOMatcher(pos)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(*m_message);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(HiveMindHostApiRequestHandlerFixture,
       HiveMindHostApiRequestHandler_handleRequest_neighbor_doesnt_exist) {
    // Given
    HiveMindHostApiRequestDTO req(GetNeighborRequestDTO(42));
    m_request->setRequest(req);
    m_message->setMessage(*m_request);

    std::optional<RelativePosition> pos = {};
    EXPECT_CALL(m_interlocMock, getRobotPosition(42)).WillOnce(testing::Return(pos));
    EXPECT_CALL(m_hostQueueMock, push(MessageGetEmtpyNeighborResponseDTOMatcher(NULL)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(*m_message);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(HiveMindHostApiRequestHandlerFixture,
       HiveMindHostApiRequestHandler_handleRequest_neighbors_list) {
    // Given
    GetNeighborsListRequestDTO nreq;
    HiveMindHostApiRequestDTO req(nreq);
    m_request->setRequest(req);
    m_message->setMessage(*m_request);

    PositionsTable posTable;
    posTable.m_positionsLength = 1;
    posTable.m_positions[0].m_distance = 42;
    posTable.m_positions[0].m_relativeOrientation = 24;
    posTable.m_positions[0].m_robotId = 8;
    posTable.m_positions[0].m_isInLineOfSight = true;

    std::vector<uint16_t> robotsId;
    robotsId.push_back(8);

    EXPECT_CALL(m_interlocMock, getPositionsTable).WillOnce(testing::ReturnRef(posTable));
    EXPECT_CALL(m_hostQueueMock, push(MessageGetNeighborsListResponseDTOMatcher(robotsId)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(*m_message);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(HiveMindHostApiRequestHandlerFixture,
       HiveMindHostApiRequestHandler_handleRequest_invalidRequest) {
    // Given
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).Times(0);
    m_message->setMessage(std::monostate());

    // Then
    bool ret = m_hivemindApiReqHandler->handleRequest(*m_message);

    // Expect
    EXPECT_FALSE(ret);
}
