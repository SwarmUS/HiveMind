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

# Sources and headers are located at the same place
list(APPEND BittyBuzz_INCLUDE_DIRS "${BittyBuzz_SOURCE_DIR}")

# bzzinclude.h include files from "bittybuzz/" and not directly from folder
configure_file(${BittyBuzz_SOURCE_DIR}/config.h.in ${BittyBuzz_SOURCE_DIR}/bittybuzz/config.h @ONLY)
configure_file(${BittyBuzz_SOURCE_DIR}/bbzenums.h ${BittyBuzz_SOURCE_DIR}/bittybuzz/bbzenums.h COPYONLY)

set(BBZ_HEADERS
        ${BittyBuzz_SOURCE_DIR}/bbzdarray.h
        ${BittyBuzz_SOURCE_DIR}/bbzfloat.h
        ${BittyBuzz_SOURCE_DIR}/bbzheap.h
        ${BittyBuzz_SOURCE_DIR}/bbzinclude.h
        ${BittyBuzz_SOURCE_DIR}/bbzinmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzneighbors.h
#        ${BittyBuzz_SOURCE_DIR}/bbzobjringbuf.h
        ${BittyBuzz_SOURCE_DIR}/bbzoutmsg.h
        ${BittyBuzz_SOURCE_DIR}/bbzringbuf.h
        ${BittyBuzz_SOURCE_DIR}/bbzstrids.h
        ${BittyBuzz_SOURCE_DIR}/bbzswarm.h
        ${BittyBuzz_SOURCE_DIR}/bbztable.h
        ${BittyBuzz_SOURCE_DIR}/bbztype.h
        ${BittyBuzz_SOURCE_DIR}/bbzutil.h
        ${BittyBuzz_SOURCE_DIR}/bbzvm.h
        ${BittyBuzz_SOURCE_DIR}/bbzvstig.h

        ${BittyBuzz_SOURCE_DIR}/bittybuzz/config.h
        ${BittyBuzz_SOURCE_DIR}/bittybuzz/bbzenums.h
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
)

# Disable compiler warnings and clang-tidy
if(DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(BittyBuzz PRIVATE -w)
    set_target_properties(BittyBuzz PROPERTIES C_CLANG_TIDY "")
endif()

find_package_handle_standard_args(BittyBuzz
    REQUIRED_VARS BittyBuzz_SOURCE_DIR
)
