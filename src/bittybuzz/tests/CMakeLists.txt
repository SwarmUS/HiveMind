set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    BittyBuzzBytecodeUnitTests.cpp
    vm_tests/BittyBuzzVmInitAddTests.cpp
    vm_tests/BittyBuzzVmLogIntTests.cpp
    vm_tests/BittyBuzzVmNoStepTests.cpp
    vm_tests/BittyBuzzVmSwarmTests.cpp
)

set(TEST_HEADERS
    mocks/BSPInterfaceMock.h
    mocks/LoggerInterfaceMock.h
    
    vm_tests/BittyBuzzVmFixture.h
)

# bzz target definitions
set(BZZ_TEST_SCRIPTS_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/vm_tests/buzz_tests_scripts)
bittybuzz_generate_bytecode(log-int 
    ${BZZ_TEST_SCRIPTS_LOCATION}/logInt.bzz 
    ${BZZ_TEST_SCRIPTS_LOCATION})

bittybuzz_generate_bytecode(init-add 
    ${BZZ_TEST_SCRIPTS_LOCATION}/initAdd.bzz 
    ${BZZ_TEST_SCRIPTS_LOCATION})

bittybuzz_generate_bytecode(no-step 
    ${BZZ_TEST_SCRIPTS_LOCATION}/noStep.bzz 
    ${BZZ_TEST_SCRIPTS_LOCATION})

bittybuzz_generate_bytecode(swarm 
    ${BZZ_TEST_SCRIPTS_LOCATION}/swarm.bzz 
    ${BZZ_TEST_SCRIPTS_LOCATION})

add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::BittyBuzz

        # BittyBuzz code
        log-int
        init-add
        no-step
        swarm
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
