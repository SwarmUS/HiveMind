set(TEST_NAME test-interloc)
 
set(TEST_SOURCES
    InterlocUnitTests.cpp
    InterlocMessageHandlerUnitTests.cpp
)

set(TEST_HEADERS
    mocks/LoggerInterfaceMock.h
    mocks/InterlocManagerInterfaceMock.h
    mocks/CircularQueueInterfaceMock.h
    mocks/BSPInterfaceMock.h
)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::Interloc
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
