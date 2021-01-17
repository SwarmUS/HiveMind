set(LIB_NAME "swarmus-hivemind-comm")
set(LIB_ALIAS "SwarmUS::HiveMind::Comm")

set(LIB_SOURCES
    src/TCPDataStream.cpp
)

set(LIB_HEADERS
    include/comm/IDataStream.h
    include/comm/TCPDataStream.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::BSP
        SwarmUS::HiveMind::Logger
)

if(ENABLE_TESTS)
    #add_subdirectory(tests)
endif()
