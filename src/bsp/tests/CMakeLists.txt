set(TEST_SOURCES
    stm32f4/test.cpp
)

add_executable(test_stm32 ${TEST_SOURCES})

target_link_libraries(test_stm32
    gtest
    gtest_main
    gmock
)

add_test(NAME test_stm32
         COMMAND test_stm32)