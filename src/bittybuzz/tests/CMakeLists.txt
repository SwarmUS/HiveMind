set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    
    BittyBuzzBytecodeUnitTests.cpp
    BittyBuzzStringResolverUnitTests.cpp
    BittyBuzzClosureRegisterUnitTests.cpp
    BittyBuzzMessageHandlerUnitTests.cpp
    BittyBuzzMessageServiceUnitTests.cpp
    BittyBuzzFunctionDescriptionUnitTests.cpp

    vm_tests/BittyBuzzVmTestsUtils.cpp

    vm_tests/BittyBuzzVmLogTests.cpp
    vm_tests/BittyBuzzVmInitAddTests.cpp
    vm_tests/BittyBuzzVmNoStepTests.cpp
    vm_tests/BittyBuzzVmSwarmTests.cpp
    vm_tests/BittyBuzzVmGlobalInitTests.cpp
    vm_tests/BittyBuzzVmUndefinedFunctionTests.cpp
    vm_tests/BittyBuzzVmStigmergyPutTests.cpp
    vm_tests/BittyBuzzVmStigmergyGetTests.cpp

    vm_tests/BittyBuzzVmRegisterClosureFunctionTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureFunctionSelfTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureLambdaTests.cpp
    vm_tests/BittyBuzzVmCallHostFunctionValidTests.cpp
    vm_tests/BittyBuzzVmCallHostFunctionInvalidArgsTests.cpp
    vm_tests/BittyBuzzVmCallHostFunctionInvalidKeysTests.cpp

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
    mocks/BittyBuzzClosureRegisterInterfaceMock.h
    mocks/BittyBuzzMessageHandlerInterfaceMock.h
    mocks/BittyBuzzMessageServiceInterfaceMock.h
    
    vm_tests/BittyBuzzVmFixture.h
    vm_tests/BittyBuzzVmTestsUtils.h
)

# bzz target definitions
set(BZZ_TEST_SCRIPTS_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/vm_tests/buzz_tests_scripts)


bittybuzz_generate_bytecode(log
    ${BZZ_TEST_SCRIPTS_LOCATION}/log.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(init-add 
    ${BZZ_TEST_SCRIPTS_LOCATION}/init_add.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(no-step 
    ${BZZ_TEST_SCRIPTS_LOCATION}/no_step.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

set(UNDEFINED_FUNCTION_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/undefined_function.bst)
bittybuzz_generate_bytecode(undefined-function 
    ${BZZ_TEST_SCRIPTS_LOCATION}/undefined_function.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${UNDEFINED_FUNCTION_BST}")

set(SWARM_BST ${BITTYBUZZ_HIVEMIND_BST} vm_tests/buzz_tests_scripts/swarm.bst)
bittybuzz_generate_bytecode(swarm 
    ${BZZ_TEST_SCRIPTS_LOCATION}/swarm.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${SWARM_BST}")

bittybuzz_generate_bytecode(global-init 
    ${BZZ_TEST_SCRIPTS_LOCATION}/global_init.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${SWARM_BST}")

bittybuzz_generate_bytecode(is-nil 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_nil.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-int 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_int.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-float 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_float.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-string 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_string.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-table 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_table.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-closure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_closure.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(is-lambda-closure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/is_lambda_closure.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(register-closure_function 
    ${BZZ_TEST_SCRIPTS_LOCATION}/register_closure_function.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(register-closure_function_self
    ${BZZ_TEST_SCRIPTS_LOCATION}/register_closure_function_self.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(register-closure_lambda 
    ${BZZ_TEST_SCRIPTS_LOCATION}/register_closure_lambda.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-host-function-valid
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_host_function_valid.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-host-function-invalid-args
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_host_function_invalidArgs.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-host-function-invalid-keys
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_host_function_invalidKeys.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(stigmergy-put
    ${BZZ_TEST_SCRIPTS_LOCATION}/stigmergy_put.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(stigmergy-get
    ${BZZ_TEST_SCRIPTS_LOCATION}/stigmergy_get.bzz
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
        init-add
        no-step
        undefined-function 
        swarm
        global-init
        is-nil
        is-int
        is-float
        is-string
        is-table
        is-closure
        is-lambda-closure
        register-closure_function
        register-closure_function_self
        register-closure_lambda
        call-host-function-valid
        call-host-function-invalid-args
        call-host-function-invalid-keys
        stigmergy-put
        stigmergy-get
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
