set(TEST_NAME test-message-handler)
 
set(TEST_SOURCES
    MessageSenderUnitTests.cpp
)

set(TEST_HEADERS
)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::MessageHandler
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
