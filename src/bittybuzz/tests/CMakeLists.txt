set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    BittyBuzzBytecodeUnitTests.cpp
    BittyBuzzVmUnitTests.cpp
)

set(TEST_HEADERS
    mocks/BittyBuzzBytecodeMock.h
)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::BittyBuzz
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
