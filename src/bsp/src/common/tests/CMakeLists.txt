set(TEST_NAME test-bsp-common)
 
set(TEST_SOURCES
    CircularBuffTests.cpp
)

set(TEST_HEADERS

)

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock
        SwarmUS::HiveMind::BSP::Common
)

target_compile_options(${LIB_NAME}
    PRIVATE
        $<$<BOOL:${ENABLE_WARNINGS}>:${WARNING_FLAG}>
        $<$<BOOL:${ENABLE_WARNINGS_AS_ERROR}>:${WARNING_AS_ERROR_FLAG}>
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})

