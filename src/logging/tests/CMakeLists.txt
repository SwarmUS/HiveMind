set(TEST_NAME test_logger)
 
set(TEST_SOURCES
    logger_unittests.cpp
)

add_executable(${TEST_NAME} ${TEST_SOURCES})

target_link_libraries(${TEST_NAME}
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::BSP
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
