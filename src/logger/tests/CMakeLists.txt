set(TEST_NAME test_logger)
 
set(TEST_SOURCES
    LoggerUnitTests.cpp
)

set(TEST_HEADERS
    mocks/UserInterfaceMock.h
)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::FreeRTOS
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
