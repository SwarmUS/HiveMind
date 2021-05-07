include(FindPackageHandleStandardArgs)
if(NOT BITTYBUZZ_SRC_PATH)
    message(ERROR "BITTYBUZZ_SRC_PATH not specified")
endif()

find_path(
    BittyBuzz_SOURCE_DIR
    NAMES bbzvm.h
    PATHS "${BITTYBUZZ_SRC_PATH}/src/bittybuzz"
    NO_DEFAULT_PATH
)

include(${BITTYBUZZ_SRC_PATH}/src/cmake/BittyBuzzGeneratorFunctions.cmake)
include(${BITTYBUZZ_SRC_PATH}/src/cmake/BittyBuzzFindBuzzPrgms.cmake)

# Sources and headers are located at the same place
list(APPEND BittyBuzz_INCLUDE_DIRS "${BittyBuzz_SOURCE_DIR}")

# Create bittybuzz config file
configure_file(${BittyBuzz_SOURCE_DIR}/config.h.in ${BittyBuzz_SOURCE_DIR}/config.h @ONLY)

set(BBZ_HEADERS
        ${BittyBuzz_SOURCE_DIR}/bbzdarray.h
        ${BittyBuzz_SOURCE_DIR}/bbzfloat.h
        ${BittyBuzz_SOURCE_DIR}/bbzheap.h
        ${BittyBuzz_SOURCE_DIR}/bbzinclude.h
        ${BittyBuzz_SOURCE_DIR}/bbzinmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzneighbors.h
#        ${BittyBuzz_SOURCE_DIR}/bbzobjringbuf.h Not supported for now on bittybuzz
        ${BittyBuzz_SOURCE_DIR}/bbzoutmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzringbuf.h
        ${BittyBuzz_SOURCE_DIR}/bbzstrids.h
        ${BittyBuzz_SOURCE_DIR}/bbzswarm.h
        ${BittyBuzz_SOURCE_DIR}/bbztable.h
        ${BittyBuzz_SOURCE_DIR}/bbztype.h
        ${BittyBuzz_SOURCE_DIR}/bbzutil.h
        ${BittyBuzz_SOURCE_DIR}/bbzvm.h
        ${BittyBuzz_SOURCE_DIR}/bbzvstig.h

        ${BittyBuzz_SOURCE_DIR}/config.h
        ${BittyBuzz_SOURCE_DIR}/bbzenums.h
)
set(BBZ_SOURCES
        ${BittyBuzz_SOURCE_DIR}/bbzdarray.c
        ${BittyBuzz_SOURCE_DIR}/bbzfloat.c
        ${BittyBuzz_SOURCE_DIR}/bbzheap.c
        ${BittyBuzz_SOURCE_DIR}/bbzinmsg.c
        ${BittyBuzz_SOURCE_DIR}/bbzmsg.c
        ${BittyBuzz_SOURCE_DIR}/bbzneighbors.c
        ${BittyBuzz_SOURCE_DIR}/bbzoutmsg.c
        ${BittyBuzz_SOURCE_DIR}/bbzringbuf.c
        ${BittyBuzz_SOURCE_DIR}/bbzswarm.c
        ${BittyBuzz_SOURCE_DIR}/bbztable.c
        ${BittyBuzz_SOURCE_DIR}/bbztype.c
        ${BittyBuzz_SOURCE_DIR}/bbzutil.c
        ${BittyBuzz_SOURCE_DIR}/bbzvm.c
        ${BittyBuzz_SOURCE_DIR}/bbzvstig.c
)


add_library(BittyBuzz STATIC ${BBZ_SOURCES} ${BBZ_HEADERS})

target_include_directories(BittyBuzz
    SYSTEM
    PUBLIC
        ${BittyBuzz_INCLUDE_DIRS}
        ${BITTYBUZZ_SRC_PATH}/src
)

# Adding utils
find_program(HOST_C_COMPILER gcc)
set(BBZ_BINARY_PATH ${CMAKE_CURRENT_BINARY_DIR})
set(BBZ_BINARY_PATH ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
set(BBZ_UTILS_SOURCES
        ${BittyBuzz_SOURCE_DIR}/exec/bo2bbo.c
        ${BittyBuzz_SOURCE_DIR}/exec/kilo_bcodegen.c
        ${BittyBuzz_SOURCE_DIR}/exec/zooids_bcodegen.c
        ${BittyBuzz_SOURCE_DIR}/exec/crazyflie_bcodegen.c
)
foreach (bbz_exec_src ${BBZ_UTILS_SOURCES})
    get_filename_component(bbz_executable ${bbz_exec_src} NAME_WE)
    add_custom_target(${bbz_executable} 
        COMMAND ${HOST_C_COMPILER} ${bbz_exec_src} ${BittyBuzz_SOURCE_DIR}/bbzfloat.c -I ${BittyBuzz_SOURCE_DIR} -I ${BittyBuzz_SOURCE_DIR}/.. -o ${CMAKE_CURRENT_BINARY_DIR}/${bbz_executable})
    set(BBZ_${bbz_executable} ${CMAKE_CURRENT_BINARY_DIR}/${bbz_executable})
    set(BBZ_${bbz_executable} ${CMAKE_CURRENT_BINARY_DIR}/${bbz_executable} PARENT_SCOPE)
endforeach ()


# Disable compiler warnings
if(DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(BittyBuzz PRIVATE -w)
endif()

find_package_handle_standard_args(BittyBuzz
    REQUIRED_VARS BittyBuzz_SOURCE_DIR
)

