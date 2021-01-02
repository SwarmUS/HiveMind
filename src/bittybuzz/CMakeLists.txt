set(LIB_NAME "swarmus-hivemind-bittybuzz")
set(LIB_ALIAS "SwarmUS::HiveMind::BittyBuzz")

include(bittybuzz/common)
include(bittybuzz_config.cmake) # not a module, need to specify "".cmake"

# Deps
bittybuzz_fetch()
find_package(BittyBuzz)

# BittyBuzz script bytecode generation
bittybuzz_generate_bytecode(bittybuzz-target
        ${CMAKE_CURRENT_SOURCE_DIR}/buzz_scripts/main.bzz
        ${CMAKE_CURRENT_SOURCE_DIR}/buzz_scripts)


set(LIB_HEADERS
    include/bittybuzz/BittyBuzzVm.h
    include/bittybuzz/IBittyBuzzVm.h

    include/bittybuzz/BittyBuzzBytecode.h
    include/bittybuzz/IBittyBuzzBytecode.h

    include/bittybuzz/BittyBuzzFactory.h
    include/bittybuzz/BittyBuzzSystem.h
    include/bittybuzz/BittyBuzzUserFunctions.h

)

set(LIB_SOURCES
    src/BittyBuzzVm.cpp
    src/BittyBuzzBytecode.cpp
    src/BittyBuzzFactory.cpp

    src/BittyBuzzSystem.cpp
    src/BittyBuzzUserFunctions.cpp
)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        src
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        BittyBuzz
        SwarmUS::HiveMind::Logger

    PRIVATE
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::FreeRTOS
        bittybuzz-target

)


if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

