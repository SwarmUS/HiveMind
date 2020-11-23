set(TEST_NAME test_stm32)
 
set(TEST_SOURCES
    stm32f4/test.cpp
)

add_executable(${TEST_NAME} ${TEST_SOURCES})

target_link_libraries(${TEST_NAME}
        gtest
        gtest_main
        gmock
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
