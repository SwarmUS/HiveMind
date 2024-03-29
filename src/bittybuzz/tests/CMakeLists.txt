set(TEST_NAME test-bittybuzz)
 
set(TEST_SOURCES
    
    BittyBuzzBytecodeUnitTests.cpp
    BittyBuzzStringResolverUnitTests.cpp
    BittyBuzzClosureRegisterUnitTests.cpp
    BittyBuzzMessageHandlerUnitTests.cpp
    BittyBuzzMessageServiceUnitTests.cpp
    BittyBuzzFunctionDescriptionUnitTests.cpp
    BittyBuzzNeighborsManagerUnitTests.cpp

    vm_tests/BittyBuzzVmTestsUtils.cpp

    vm_tests/BittyBuzzVmLogTests.cpp
    vm_tests/BittyBuzzVmInitAddTests.cpp
    vm_tests/BittyBuzzVmNoStepTests.cpp
    vm_tests/BittyBuzzVmSwarmTests.cpp
    vm_tests/BittyBuzzVmGlobalInitTests.cpp
    vm_tests/BittyBuzzVmUndefinedFunctionTests.cpp
    vm_tests/BittyBuzzVmStigmergyPutTests.cpp
    vm_tests/BittyBuzzVmStigmergyGetTests.cpp
    vm_tests/BittyBuzzVmStigmergyOnConflictTests.cpp
    vm_tests/BittyBuzzVmNeighborsDistanceTests.cpp
    vm_tests/BittyBuzzVmNeighborsBroadcastTests.cpp
    vm_tests/BittyBuzzVmNeighborsListenTests.cpp
    vm_tests/BittyBuzzVmTableLibraryTests.cpp
    vm_tests/BittyBuzzVmUITests.cpp
    vm_tests/BittyBuzzVmUIInvalidHexTests.cpp

    vm_tests/BittyBuzzVmRegisterClosureFunctionTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureFunctionSelfTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureInvalidArgsTests.cpp
    vm_tests/BittyBuzzVmRegisterClosureLambdaTests.cpp


    vm_tests/BittyBuzzVmCallBuzzFunctionValidTests.cpp
    vm_tests/BittyBuzzVmCallBuzzFunctionInvalidArgsTests.cpp
    vm_tests/BittyBuzzVmCallBuzzFunctionInvalidKeysTests.cpp

    vm_tests/BittyBuzzVmCallHostFunctionValidTests.cpp
    vm_tests/BittyBuzzVmCallHostFunctionInvalidArgsTests.cpp
    vm_tests/BittyBuzzVmCallHostFunctionInvalidKeysTests.cpp

    vm_tests/BittyBuzzVmCastFloat.cpp
    vm_tests/BittyBuzzVmCastInt.cpp
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
    mocks/BittyBuzzNeighborsManagerInterfaceMock.h
    mocks/UserUIInterfaceMock.h
    mocks/InterlocInterfaceMock.h
    
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

bittybuzz_generate_bytecode(isnil 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isnil.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isint 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isint.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isfloat 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isfloat.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isstring 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isstring.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(istable 
    ${BZZ_TEST_SCRIPTS_LOCATION}/istable.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(isclosure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/isclosure.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(islambda-closure 
    ${BZZ_TEST_SCRIPTS_LOCATION}/islambda_closure.bzz 
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

bittybuzz_generate_bytecode(register-closure_invalid_args
    ${BZZ_TEST_SCRIPTS_LOCATION}/register_closure_invalid_args.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(register-closure_lambda 
    ${BZZ_TEST_SCRIPTS_LOCATION}/register_closure_lambda.bzz 
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")


bittybuzz_generate_bytecode(cast-float
    ${BZZ_TEST_SCRIPTS_LOCATION}/cast_float.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(cast-int
    ${BZZ_TEST_SCRIPTS_LOCATION}/cast_int.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-buzz-function-valid
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_buzz_function_valid.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-buzz-function-invalid-args
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_buzz_function_invalidArgs.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(call-buzz-function-invalid-keys
    ${BZZ_TEST_SCRIPTS_LOCATION}/call_buzz_function_invalidKeys.bzz
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

bittybuzz_generate_bytecode(stigmergy-onconflict
    ${BZZ_TEST_SCRIPTS_LOCATION}/stigmergy_onconflict.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(neighbors-distance
    ${BZZ_TEST_SCRIPTS_LOCATION}/neighbors_distance.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(neighbors-broadcast
    ${BZZ_TEST_SCRIPTS_LOCATION}/neighbors_broadcast.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(neighbors-listen
    ${BZZ_TEST_SCRIPTS_LOCATION}/neighbors_listen.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(table-library
    ${BZZ_TEST_SCRIPTS_LOCATION}/table_library.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(user-interface
    ${BZZ_TEST_SCRIPTS_LOCATION}/user_interface.bzz
    "${BZZ_TEST_SCRIPTS_LOCATION}"
    "${BITTYBUZZ_HIVEMIND_BST}")

bittybuzz_generate_bytecode(user-interface-invalid-hex
    ${BZZ_TEST_SCRIPTS_LOCATION}/user_interface_invalid_hex.bzz
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
        isnil
        isint
        isfloat
        isstring
        istable
        isclosure
        islambda-closure
        register-closure_function
        register-closure_function_self
        register-closure_invalid_args
        register-closure_lambda
        call-buzz-function-valid
        call-buzz-function-invalid-args
        call-buzz-function-invalid-keys
        call-host-function-valid
        call-host-function-invalid-args
        call-host-function-invalid-keys
        cast-float
        cast-int
        stigmergy-put
        stigmergy-get
        stigmergy-onconflict
        neighbors-distance
        neighbors-broadcast
        neighbors-listen
        table-library
        user-interface
        user-interface-invalid-hex
)

add_test(NAME ${TEST_NAME}
         COMMAND ${TEST_NAME})
