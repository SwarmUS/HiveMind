set(LIB_NAME "swarmus-hivemind-application-interface")
set(LIB_ALIAS "SwarmUS::HiveMind::ApplicationInterface")

set(LIB_SOURCES
)

set(LIB_HEADERS
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>

    PRIVATE
       include/user-interface
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::Logger
)

if(ENABLE_TESTS)
    # add_subdirectory(tests)
endif()