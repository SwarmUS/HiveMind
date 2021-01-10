set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    BittyBuzzBytecodeUnitTests.cpp
    BittyBuzzStringResolverUnitTests.cpp

    vm_tests/BittyBuzzVmInitAddTests.cpp
    vm_tests/BittyBuzzVmLogIntTests.cpp
    vm_tests/BittyBuzzVmNoStepTests.cpp
    vm_tests/BittyBuzzVmUndefinedFunctionTests.cpp
    vm_tests/BittyBuzzVmSwarmTests.cpp
)

set(TEST_HEADERS
    mocks/BSPInterfaceMock.h
    mocks/LoggerInterfaceMock.h
    
    vm_tests/BittyBuzzVmFixture.h
)

# bzz target definitions
set(BZZ_TEST_SCRIPTS_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/vm_tests/buzz_tests_scripts)

set(LOGINT_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/logInt.bst)
list(LENGTH LOGINT_BST TEST)
message("WOOW ${LOGINT_BST}")
message("WOOW3 ${TEST}")
bittybuzz_generate_bytecode(log-int 
    ${BZZ_TEST_SCRIPTS_LOCATION}/logInt.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${LOGINT_BST}")

bittybuzz_generate_bytecode(init-add 
    ${BZZ_TEST_SCRIPTS_LOCATION}/initAdd.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(no-step 
    ${BZZ_TEST_SCRIPTS_LOCATION}/noStep.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

set(UNDEFINED_FUNCTION_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/undefinedFunction.bst)
bittybuzz_generate_bytecode(undefined-function 
    ${BZZ_TEST_SCRIPTS_LOCATION}/undefinedFunction.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${UNDEFINED_FUNCTION_BST}")

set(SWARM_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/swarm.bst)
bittybuzz_generate_bytecode(swarm 
    ${BZZ_TEST_SCRIPTS_LOCATION}/swarm.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${SWARM_BST}")


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
        undefined-function 
        swarm
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
