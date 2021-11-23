set(LIB_NAME "swarmus-hivemind-bsp-common")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Common")


set(LIB_SOURCES
    src/circular_buff.c
)

set(LIB_HEADERS
    include/common/circular_buff.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include/common
    PRIVATE
        src
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()
