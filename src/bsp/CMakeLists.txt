set(LIB_NAME "swarmus-hivemind-bsp")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP")

add_subdirectory(src)

set(LIB_HEADERS
  include/bsp.h
)


# find_package(FreeRTOS REQUIRED)

add_library(${LIB_NAME}  ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include/
    PRIVATE
        src
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::BSP::Platform
)


if (NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(tests)
endif()
