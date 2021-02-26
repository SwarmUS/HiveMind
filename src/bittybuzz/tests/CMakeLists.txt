set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    
    BittyBuzzBytecodeUnitTests.cpp
    BittyBuzzStringResolverUnitTests.cpp
    BittyBuzzFunctionRegisterUnitTests.cpp
    BittyBuzzMessageHandlerUnitTests.cpp

    vm_tests/BittyBuzzVmTestsUtils.cpp

    vm_tests/BittyBuzzVmLogTests.cpp
    vm_tests/BittyBuzzVmInitAddTests.cpp
    vm_tests/BittyBuzzVmLogIntTests.cpp
    vm_tests/BittyBuzzVmNoStepTests.cpp
    vm_tests/BittyBuzzVmSwarmTests.cpp
    vm_tests/BittyBuzzVmUndefinedFunctionTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureFunctionTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureLambdaTests.cpp

    vm_tests/BittyBuzzVmIsNilTests.cpp
    vm_tests/BittyBuzzVmIsIntTests.cpp
    vm_tests/BittyBuzzVmIsFloatTests.cpp
    vm_tests/BittyBuzzVmIsStringTests.cpp
    vm_tests/BittyBuzzVmIsTableTests.cpp
    vm_tests/BittyBuzzVmIsClosureTests.cpp
    vm_tests/BittyBuzzVmIsLambdaClosureTests.cpp
)

set(TEST_HEADERS
    mocks/BSPInterfaceMock.h
    mocks/LoggerInterfaceMock.h
    mocks/CircularQueueInterfaceMock.h
    mocks/BittyBuzzFunctionRegisterInterfaceMock.h
    mocks/BittyBuzzMessageHandlerInterfaceMock.h
    
    vm_tests/BittyBuzzVmFixture.h
    vm_tests/BittyBuzzVmTestsUtils.h
)

# bzz target definitions
set(BZZ_TEST_SCRIPTS_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/vm_tests/buzz_tests_scripts)


bittybuzz_generate_bytecode(log
    ${BZZ_TEST_SCRIPTS_LOCATION}/log.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

set(LOGINT_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/logInt.bst)
list(LENGTH LOGINT_BST TEST)
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

bittybuzz_generate_bytecode(isNil 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isNil.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isInt 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isInt.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isFloat 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isFloat.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isString 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isString.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isTable 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isTable.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isClosure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isClosure.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isLambdaClosure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isLambdaClosure.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(registerClosure_function 
    ${BZZ_TEST_SCRIPTS_LOCATION}/registerClosure_function.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(registerClosure_lambda 
    ${BZZ_TEST_SCRIPTS_LOCATION}/registerClosure_lambda.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")


add_executable(${TEST_NAME} ${TEST_SOURCES} ${TEST_HEADERS})

target_link_libraries(${TEST_NAME}
    PUBLIC
        gtest
        gtest_main
        gmock

        SwarmUS::HiveMind::BittyBuzz

        # BittyBuzz code
        log
        log-int
        init-add
        no-step
        undefined-function 
        swarm
        isNil
        isInt
        isFloat
        isString
        isTable
        isClosure
        isLambdaClosure
        registerClosure_function
        registerClosure_lambda
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
