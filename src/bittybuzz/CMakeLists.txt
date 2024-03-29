set(LIB_NAME "swarmus-hivemind-bittybuzz")
set(LIB_ALIAS "SwarmUS::HiveMind::BittyBuzz")

include(bittybuzz/common)
include(bittybuzz_config.cmake) # not a module, need to specify .cmake
include(buzz_scripts/user_bst_files.cmake)

# Settings
configure_file(include/bittybuzz/BittyBuzzSettings.in.h include/bittybuzz/BittyBuzzSettings.h)

# Deps
bittybuzz_fetch()
find_package(BittyBuzz)
# System bst file
set(BITTYBUZZ_HIVEMIND_BST ${CMAKE_CURRENT_SOURCE_DIR}/bittybuzz.bst)

# Concat with user bst file
set(BITTYBUZZ_BST_FILES ${USER_BUZZ_BST_FILES} ${BITTYBUZZ_HIVEMIND_BST})
set(BITTYBUZZ_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/buzz_scripts:${USER_BUZZ_INCLUDE_DIRS})

# BittyBuzz script bytecode generation
bittybuzz_generate_bytecode(bittybuzz-target
        ${CMAKE_CURRENT_SOURCE_DIR}/buzz_scripts/main.bzz
        "${BITTYBUZZ_INCLUDE_DIRS}"
        "${BITTYBUZZ_BST_FILES}")

set(LIB_HEADERS
    include/bittybuzz/BittyBuzzVm.h
    include/bittybuzz/IBittyBuzzVm.h

    include/bittybuzz/BittyBuzzBytecode.h
    include/bittybuzz/IBittyBuzzBytecode.h

    include/bittybuzz/BittyBuzzStringResolver.h
    include/bittybuzz/IBittyBuzzStringResolver.h

    include/bittybuzz/BittyBuzzClosureRegister.h
    include/bittybuzz/IBittyBuzzClosureRegister.h

    include/bittybuzz/BittyBuzzMessageHandler.h
    include/bittybuzz/IBittyBuzzMessageHandler.h

    include/bittybuzz/BittyBuzzMessageService.h
    include/bittybuzz/IBittyBuzzMessageService.h

    include/bittybuzz/BittyBuzzNeighborsManager.h
    include/bittybuzz/IBittyBuzzNeighborsManager.h

    include/bittybuzz/BittyBuzzMathFunctions.h

    include/bittybuzz/BittyBuzzUIFunctions.h

    include/bittybuzz/BittyBuzzLib.h
    include/bittybuzz/BittyBuzzLib.tpp
    include/bittybuzz/IBittyBuzzLib.h
    include/bittybuzz/BittyBuzzLibMemberRegister.h

    include/bittybuzz/BittyBuzzUtils.h
    include/bittybuzz/BittyBuzzSystem.h
    include/bittybuzz/BittyBuzzFactory.h
    include/bittybuzz/BittyBuzzContainer.h
    include/bittybuzz/BittyBuzzUserFunctions.h
    include/bittybuzz/BittyBuzzFunctionDescription.h
)

set(LIB_SOURCES
    src/BittyBuzzVm.cpp
    src/BittyBuzzBytecode.cpp
    src/BittyBuzzStringResolver.cpp
    src/BittyBuzzClosureRegister.cpp
    src/BittyBuzzMessageHandler.cpp
    src/BittyBuzzMessageService.cpp
    src/BittyBuzzNeighborsManager.cpp

    src/BittyBuzzUtils.cpp
    src/BittyBuzzSystem.cpp
    src/BittyBuzzFactory.cpp
    src/BittyBuzzContainer.cpp
    src/BittyBuzzUserFunctions.cpp
    src/BittyBuzzFunctionDescription.cpp
    src/BittyBuzzMathFunctions.cpp
    src/BittyBuzzUIFunctions.cpp
    src/BittyBuzzLibMemberRegister.cpp
)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>

        #Include the generated BittyBuzzSettings.h
        ${CMAKE_CURRENT_BINARY_DIR}/include
    PRIVATE
        src
        include/bittybuzz
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        BittyBuzz
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::Interloc
        SwarmUS::HiveMind::MessageHandler
        SwarmUS::HiveMind::ApplicationInterface
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::Propolis::Pheromones

    PRIVATE
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::OS
        bittybuzz-target

)


if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

