set(LIB_NAME "bsp")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP")

get_filename_component(BSP_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

add_subdirectory(src)

set(LIB_HEADERS
    include/bsp/bsp.h
)

add_library(${LIB_NAME}  ${LIB_HEADERS})
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
        SwarmUS::HiveMind::BSP::Platform
)


if (ENABLE_TESTS)
    add_subdirectory(tests)
endif()
