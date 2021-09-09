set(TEST_NAME test-application-interface)

set(TEST_SOURCES
  ApplicationInterfaceTests.cpp
)

set(TEST_HEADERS
  mocks/MutexInterfaceMock.h
  mocks/UserInterfaceMock.h
)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::ApplicationInterface
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
