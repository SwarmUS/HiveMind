set(LIB_NAME "swarmus-hivemind-logger")
set(LIB_ALIAS "SwarmUS::HiveMind::Logger")

set(LIB_SOURCES
    src/Logger.cpp
    src/LoggerContainer.cpp
)

set(LIB_HEADERS
    include/logger/ILogger.h
    include/logger/Logger.h
    include/logger/LoggerContainer.h
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
        SwarmUS::HiveMind::OS
)

if(ENABLE_TESTS)
    add_subdirectory(tests)
endif()

